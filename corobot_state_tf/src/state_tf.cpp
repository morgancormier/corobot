#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "math.h"
#include "corobot_msgs/PosMsg.h"
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <corobot_state_tf/corobot_state_tfConfig.h>

long _PreviousLeftEncoderCounts = 0; //save the last left encoder value
long _PreviousRightEncoderCounts = 0; //save the last right encoder value
ros::Time current_time_encoder, last_time_encoder; //save the time for the current and the previous encoder time
double DistancePerCount; //to calculate - distance in meter for one count of the encoders.The Phidget C API gives the number of encoder ticks *4.
double lengthBetweenTwoWheels; //distance in meters between the left and right wheels, taken in the middle.
 


double x, y, th; // the position of the robot in the plan, and the orientation
double dx = 0, dy = 0, dth = 0; // the difference of position and orientation between the two last encoder measurements, in the robot's frame
double dela_time; //time difference between the two last encoder measurement received

bool firstTime = true; // true if we are waiting for our first encoder measurement
bool is4WheelDrive;
bool publish_odom_tf; //if true publish the transform from odom to base_footprint
bool odometry_activated = true;

void WheelCallback(const corobot_msgs::PosMsg::ConstPtr& pos)
/**
 * Calculate the velocity and the odometry
 * Called everytime a new encoder position has been received.
 */
{
  current_time_encoder = pos->header.stamp; // the time corresponding to the encoder measurement
  double distance_left = 0.0,distance_right = 0.0; // the distance made by the left and the right wheel

  //current_time_encoder = ros::Time::now();
  dela_time = (current_time_encoder - last_time_encoder).toSec();
  if (dela_time>0.001){
	  if(firstTime == false)
	  { 
    	    distance_left = ((double)(pos->px - _PreviousLeftEncoderCounts) * DistancePerCount); //distance made by the left wheel
    	    distance_right = ((double)(pos->py - _PreviousRightEncoderCounts) * DistancePerCount); //distance made by the right wheel
	    
	    dx = (distance_left + distance_right)/2.0;
	    dth = (distance_right - distance_left)/lengthBetweenTwoWheels; //rotation made by the robot
	    double delta_x = 0.0, delta_y = 0.0, delta_th = 0.0; //in the frame of reference
  	    delta_x = (dx * cos(th) - dy * sin(th));
    	    delta_y = (dx * sin(th) + dy * cos(th));
    	    delta_th = dth;
            x += delta_x;
            y += delta_y;
	    // we need to make sure that the new orientation is between 0 and 2pi and not outside of this range
	    int mod = (int) ((th+delta_th)/(2*M_PI)); 
            th = (th+ delta_th) -(mod*2*M_PI);
	  }
	  else
	    firstTime = false;

	  //save the encoder position and time to use it when the next encoder measurement is received.
	  _PreviousLeftEncoderCounts = pos->px;
	  _PreviousRightEncoderCounts = pos->py;
	  last_time_encoder = current_time_encoder;
    }
}

void dynamic_reconfigureCallback(corobot_state_tf::corobot_state_tfConfig &config, uint32_t level) {
		odometry_activated = config.camera_state_tf;
}

void publish_odometry(ros::Publisher& odom_pub, tf::TransformBroadcaster& odom_broadcaster, tf::TransformBroadcaster& broadcaster)
/**
 * Publish the odometry and tf messages
 */
{
	//publish the transform between the base_link and the laser range finder
	broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.15, 0, 0)),ros::Time::now(),"base_link", "laser"));

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(0);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	geometry_msgs::TransformStamped odom_trans2;
	odom_trans2.header.stamp = current_time_encoder;
	odom_trans2.header.frame_id = "base_footprint";
	odom_trans2.child_frame_id = "base_link";

	odom_trans2.transform.translation.x = 0;
	odom_trans2.transform.translation.y = 0;
	odom_trans2.transform.translation.z = 0.0;
	odom_trans2.transform.rotation = odom_quat2;


	//send the transform
	if(publish_odom_tf)
		odom_broadcaster.sendTransform(odom_trans);
	odom_broadcaster.sendTransform(odom_trans2);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time_encoder;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	//These covariance values are "random". Some better values could be found after experiments and calculations
	odom.pose.covariance[0] = 0.01;
	odom.pose.covariance[7] = 0.01;
	odom.pose.covariance[14] = 10000;
	odom.pose.covariance[21] = 10000;
	odom.pose.covariance[28] = 10000;
	odom.pose.covariance[35] = 0.1;


	//set the velocity
	odom.child_frame_id = "base_link";
	if (dela_time!=0)
	{
		odom.twist.twist.linear.x = dx/dela_time;
		odom.twist.twist.linear.y = dy/dela_time;
		odom.twist.twist.angular.z = dth/dela_time;
	}
	else
	{
		odom.twist.twist.linear.x = 0;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = 0;
	}
	//These covariance values are "random". Some better values could be found after experiments and calculations
	odom.twist.covariance[0] = 0.01;
	odom.twist.covariance[7] = 0.01;
	odom.twist.covariance[14] = 10000;
	odom.twist.covariance[21] = 10000;
	odom.twist.covariance[28] = 10000;
	odom.twist.covariance[35] = 0.1;

	//publish the message
	odom_pub.publish(odom);
}


int main(int argc, char** argv){
  int ticks_meter; 

  ros::init(argc, argv, "corobot_state_tf");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  
  nh.param("4WheelDrive", is4WheelDrive, false);
  nh.param("base_width", lengthBetweenTwoWheels, 0.25); // The length between the left and right wheel
  nh.param("ticks_meter", ticks_meter, 9400); // the number of the encoder ticks per meter
  nh.param("publish_odom_tf", publish_odom_tf, true);

  dynamic_reconfigure::Server<corobot_state_tf::corobot_state_tfConfig> server;
  dynamic_reconfigure::Server<corobot_state_tf::corobot_state_tfConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigureCallback, _1, _2);
  server.setCallback(f);

  //Set up the distance per encoder ticks and the length between two wheels variable
  
  DistancePerCount = 1.0/(float)ticks_meter;
  if(is4WheelDrive)
  {
    lengthBetweenTwoWheels *= 1.5; // This is to compensate the fact that we don't have a differential drive but a skid system
  }


  //Setup the subscriber and publisher of topics
  ros::Subscriber sub = n.subscribe("/position_data", 1000, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 50);   
  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate r(50); //rate at which the ros spin function is called when the odometry is activated
  ros::Rate r_deactivated(2); //rate at which the ros spin function is called when the odometry is innactivated by dynamic reconfigure

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
	ros::spinOnce();
	if (odometry_activated)
	{
		publish_odometry(odom_pub, odom_broadcaster, broadcaster);
	    	r.sleep();
    }
	else
	    r_deactivated.sleep();
  }
}
