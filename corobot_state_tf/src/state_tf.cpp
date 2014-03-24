/*
 * Copyright (c) 2009, CoroWare
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "math.h"
#include "corobot_msgs/PosMsg.h"
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <corobot_state_tf/corobot_state_tfConfig.h>
#include "utility.h"

// Utility object for geometry functions
Utility u;

// To calculate - distance in meter for one count of the encoders.The Phidget C API gives the number of encoder ticks *4.
double DistancePerCount; 

// Distance in meters between the left and right wheels, measured from the middle of the wheels
double lengthBetweenTwoWheels; 

// X, y, and orientation of the robot
double x, y, th; 

// True if we are waiting for first encoder measurement
bool firstTime = true; 

// True if 4-W robot
bool is4WheelDrive;

// If true, publish the transform from odom to base_footprint
bool publish_odom_tf; 

// True if we are publishing odometry
bool odometry_activated = true;


// Variables used to calculate the velocity. 
// The velocity is calculated at time of publishing, 
// every 20ms and position and timestamp need to be saved
double previous_x = 0;
double previous_y = 0;
double previous_th = 0;

double previous_vx = 0;
double previous_vy = 0;
double previous_vth = 0;


// Want to only update x,y,th after a certain amount of PosMsgs are received
corobot_msgs::PosMsg previous;
corobot_msgs::PosMsg current;

std::vector<float> dists;


bool x_startError = false;
bool y_startError = false;
bool th_startError = false;



void errorAdjustment(const double dx, const double dth) {
  //std::cout<<"\n\ndx: "<<dx<<" dth: "<<dth;

  // Adjust theta
  float th_error = -0.042555431f*dth;
  th = u.displaceAngle(th, th_error);
  
  float tempx = dx*cos(th);
  float tempy = dx*sin(th);

  // Calculate x error
  float x_error = -0.0181546483f*tempx;// + 0.0008176502;
  
  // Calculate y error
  float y_error = -0.0181546483f*tempy;// + 0.0008176502;


  // Set the new x,y values
  x += (tempx + x_error);
  y += (tempy + y_error);


  // Check if acceleration is higher than some threshold
  double v_x = (x - previous_x) / (current.header.stamp.toSec() - previous.header.stamp.toSec());
  double v_y = (y - previous_y) / (current.header.stamp.toSec() - previous.header.stamp.toSec());
  double v_th = (th - previous_th) / (current.header.stamp.toSec() - previous.header.stamp.toSec());

  double a_x = (v_x - previous_vx) / (current.header.stamp.toSec() - previous.header.stamp.toSec());
  double a_y = (v_y - previous_vy) / (current.header.stamp.toSec() - previous.header.stamp.toSec());
  double a_th = (v_th - previous_vth) / (current.header.stamp.toSec() - previous.header.stamp.toSec());
  //std::cout<<"\n\nv_th: "<<v_th<<" previous_vth: "<<previous_vth<<" a_th: "<<a_th;

  if( a_x > 0.6 && !x_startError && current.header.stamp != previous.header.stamp) {
    x += 0.009810897f/2.f;
    x_startError = true;
  }

  if(a_y > 0.6 && !y_startError && current.header.stamp != previous.header.stamp) {
    y += 0.009810897f/2.f;
    y_startError = true;
  }

  if( (a_th > 0.1 || a_th < -0.1) && !th_startError && current.header.stamp != previous.header.stamp) {
    th -= (0.005077337f / 2.f);
    th_startError = true;
  }
  else if(v_th > 0.001) {
    th_startError = false;
  }
}


/** This function sets the x, y, th, dx, and dth values */
void setValues() {
  
  // Set previous time encoder data was measured
  ros::Time last_time_encoder = previous.header.stamp;
  
  // Time corresponding to the encoder measurement
  ros::Time current_time_encoder = current.header.stamp; 

  // Get time difference
  double delta_time = (current_time_encoder - last_time_encoder).toSec();
  
  // Check for if initialization is needed
  if(firstTime == false) { 

    // Distance made by the left and right wheels
    double distance_left = ((double)(current.px - previous.px) * DistancePerCount);     	    
    double distance_right = ((double)(current.py - previous.py) * DistancePerCount); 

    // Get the overall distance traveled
    double dx = (distance_left + distance_right)/2.0;
      
    // Length of the arc formed by wheel turning
    double arc_length = (distance_right - distance_left);

    // Central angle formed by arc is proportionate to arc length
    // length / circumference = theta / 2*PI, which is 2PI*l/2PI*r = l/r
    double dth = (arc_length)/(lengthBetweenTwoWheels); 

    // Displace th by dth
    th = u.displaceAngle(th, dth);

    // Adjust for error
    errorAdjustment(dx, dth);
  } // end if not first time
  // Else, set firstTime=false
  else {
    firstTime = false;
  } // end else
} // End setValues


/** This function sets the current position message */
void WheelCallback(const corobot_msgs::PosMsg& pos) {

  current = pos;

} // End WheelCallback



void dynamic_reconfigureCallback(corobot_state_tf::corobot_state_tfConfig &config, uint32_t level) {
		odometry_activated = config.camera_state_tf;
}



/** Publish the odometry and tf messages */
void publish_odometry(ros::Publisher& odom_pub, tf::TransformBroadcaster& odom_broadcaster, tf::TransformBroadcaster& broadcaster) {

  // Set values needed to calculate odometry
  setValues();

	// Publish the transform between the base_link and the laser range finder
	broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.15, 0, 0)),ros::Time::now(),"base_link", "laser"));

	// All odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(0);

	// First, publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	geometry_msgs::TransformStamped odom_trans2;
	odom_trans2.header.stamp = current.header.stamp;
	odom_trans2.header.frame_id = "base_footprint";
	odom_trans2.child_frame_id = "base_link";

	odom_trans2.transform.translation.x = 0;
	odom_trans2.transform.translation.y = 0;
	odom_trans2.transform.translation.z = 0.0;
	odom_trans2.transform.rotation = odom_quat2;


	// Send the transform
	if(publish_odom_tf)
		odom_broadcaster.sendTransform(odom_trans);
	odom_broadcaster.sendTransform(odom_trans2);

	// Next, publish the odometry message 
	nav_msgs::Odometry odom;
	odom.header.stamp = current.header.stamp;
	odom.header.frame_id = "odom";

	// Set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	
  // These covariance values are "random". Some better values could be found after experiments and calculations
	odom.pose.covariance[0] = 0.01;
	odom.pose.covariance[7] = 0.01;
	odom.pose.covariance[14] = 10000;
	odom.pose.covariance[21] = 10000;
	odom.pose.covariance[28] = 10000;
	odom.pose.covariance[35] = 0.1;


	/** Set the velocity values */
	
  odom.child_frame_id = "base_link";

  // If robot is moving
  if (firstTime == false && current.header.stamp != previous.header.stamp) {
  
  
    // Velocity values are current - previous / time_difference
    // X
    odom.twist.twist.linear.x = (x - previous_x)/((current.header.stamp - previous.header.stamp).toSec());

    // Y
    odom.twist.twist.linear.y = (y - previous_y)/((current.header.stamp - previous.header.stamp).toSec());

    // Theta
    odom.twist.twist.angular.z = (u.findDistanceBetweenAngles(th, previous_th)) / ((current.header.stamp - previous.header.stamp).toSec());

    // Set the previous values
    previous_x = x;
    previous_y = y;
    previous_th = th;
    previous.header.stamp = current.header.stamp;

    previous_vx = odom.twist.twist.linear.x;
    previous_vy = odom.twist.twist.linear.y;
    previous_vth = odom.twist.twist.angular.z;
  }

  // Else, set velocities to zero
  else {
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    previous.header.stamp = ros::Time::now();

    previous_vx = 0;
    previous_vy = 0;
    previous_vth = 0;
    x_startError = false;
    y_startError = false;
    th_startError = false;
  }
	
  // These covariance values are "random". Some better values could be found after experiments and calculations
  odom.twist.covariance[0] = 0.01;
  odom.twist.covariance[7] = 0.01;
  odom.twist.covariance[14] = 10000;
  odom.twist.covariance[21] = 10000;
	odom.twist.covariance[28] = 10000;
	odom.twist.covariance[35] = 0.1;

	// Publish the message
	odom_pub.publish(odom);

  // Set previous
  previous = current;
} //End publish_odometry


int main(int argc, char** argv) {
  int ticks_meter; 

  ros::init(argc, argv, "corobot_state_tf");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
 

  // True if we have four wheel robot
  nh.param("FourWheelDrive", is4WheelDrive, false);
  
  // The length between the left and right wheel
  nh.param("base_width", lengthBetweenTwoWheels, 0.254); 

  // Number of encoder ticks per metet
  nh.param("ticks_meter", ticks_meter, 9400); 

  // True if we are publishing odometry and tf
  nh.param("publish_odom_tf", publish_odom_tf, true);

  dynamic_reconfigure::Server<corobot_state_tf::corobot_state_tfConfig> server;
  dynamic_reconfigure::Server<corobot_state_tf::corobot_state_tfConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigureCallback, _1, _2);
  server.setCallback(f);

  //Set up the distance per encoder ticks and the length between two wheels variable
  DistancePerCount = 1.0/(float)ticks_meter;

  // If 4 wheels, change the distance between the wheels
  // This is to compensate the fact that we don't have a differential drive but a skid system
  if(is4WheelDrive) {
    lengthBetweenTwoWheels *= 1.5;   
  }

  //Setup the subscriber and publisher of topics
  ros::Subscriber sub = n.subscribe("position_data", 1000, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 50);   
  tf::TransformBroadcaster odom_broadcaster;

  // Rate at which to check for encoder updates when odometry is activated
  ros::Rate r(10); 

  // Rate at which to check for encoder updates when odometry is deactivated by dynamic_reconfigure
  ros::Rate r_deactivated(2); 

  tf::TransformBroadcaster broadcaster;


  for(unsigned int i=0;i<3;i++) {
    dists.push_back(0);
  }

  while(n.ok()) {
    ros::spinOnce();
    if (odometry_activated) {
      publish_odometry(odom_pub, odom_broadcaster, broadcaster);
      r.sleep();
    }
    else
        r_deactivated.sleep();
  } //end while

  return 0;
} 
