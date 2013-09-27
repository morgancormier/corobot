#include "ros/ros.h"
#include <stdio.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "corobot_msgs/MotorCommand.h"
#include "corobot_gazebo/ModelStates.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "math.h"

ros::Publisher left_rear_wheel_pub, right_rear_wheel_pub,left_front_wheel_pub, right_front_wheel_pub, odom_pub;

void setVelocity(const corobot_msgs::MotorCommand &msg)
{
	std_msgs::Float64 msg_left, msg_right;
	msg_left.data = msg.leftSpeed/3;
	msg_right.data = msg.rightSpeed/3;

	if(left_rear_wheel_pub)
		left_rear_wheel_pub.publish(msg_left);
	if(right_rear_wheel_pub)
		right_rear_wheel_pub.publish(msg_right);
	if(left_front_wheel_pub)
		left_front_wheel_pub.publish(msg_left);
	if(right_front_wheel_pub)
		right_front_wheel_pub.publish(msg_right);
}

void setLeftWheelVelocity(const std_msgs::Float32 &msg)
{
	std_msgs::Float64 msg_left;
	msg_left.data = msg.data / (0.105*M_PI / (2*M_PI));// the msg data is a value in meter per second but the gazebo command takes a value of radian per second for the motor. This is just the convertion.

	if(left_rear_wheel_pub)
		left_rear_wheel_pub.publish(msg_left);
	if(left_front_wheel_pub)
		left_front_wheel_pub.publish(msg_left);
}

void setRightWheelVelocity(const std_msgs::Float32 &msg)
{
	std_msgs::Float64 msg_right;
	msg_right.data = msg.data / (0.105*M_PI / (2*M_PI));// the msg data is a value in meter per second but the gazebo command takes a value of radian per second for the motor. This is just the convertion.

	if(right_rear_wheel_pub)
		right_rear_wheel_pub.publish(msg_right);
	if(right_front_wheel_pub)
		right_front_wheel_pub.publish(msg_right);
}

void getModelState(const corobot_gazebo::ModelStates &msg)
{
	//get the gazebo model state and publish odometry and tf instead
	nav_msgs::Odometry odom;
	static tf::TransformBroadcaster broadcaster;

	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";


	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	odom_trans.transform.translation.x = msg.pose[1].position.x;
	odom_trans.transform.translation.y = msg.pose[1].position.y;
	odom_trans.transform.translation.z = msg.pose[1].position.z;
	odom_trans.transform.rotation = msg.pose[1].orientation;

	geometry_msgs::TransformStamped odom_trans2;
	odom_trans2.header.stamp = ros::Time::now();
	odom_trans2.header.frame_id = "base_footprint";
	odom_trans2.child_frame_id = "base_link";

	odom_trans2.transform.translation.x = 0;
	odom_trans2.transform.translation.y = 0;
	odom_trans2.transform.translation.z = 0.0;
	odom_trans2.transform.rotation = tf::createQuaternionMsgFromYaw(0);

	broadcaster.sendTransform(odom_trans);
	broadcaster.sendTransform(odom_trans2);
	

	//Publish Odometry
	odom.pose.pose = msg.pose[1];
	//These covariance values are "random". Some better values could be found after experiments and calculations
	odom.pose.covariance[0] = 0.01;
	odom.pose.covariance[7] = 0.01;
	odom.pose.covariance[14] = 0.01;
	odom.pose.covariance[21] = 0.01;
	odom.pose.covariance[28] = 0.01;
	odom.pose.covariance[35] = 0.1;


	odom.twist.twist = msg.twist[1];
	//These covariance values are "random". Some better values could be found after experiments and calculations
	odom.twist.covariance[0] = 0.01;
	odom.twist.covariance[7] = 0.01;
	odom.twist.covariance[14] = 0.01;
	odom.twist.covariance[21] = 0.01;
	odom.twist.covariance[28] = 0.01;
	odom.twist.covariance[35] = 0.1;

	if (odom_pub)
		odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "corobot_gazebo_control");
	ros::NodeHandle n;

	left_rear_wheel_pub = n.advertise<std_msgs::Float64>("corobot_left_rear_wheel_controller/command", 100);
	right_rear_wheel_pub = n.advertise<std_msgs::Float64>("corobot_right_rear_wheel_controller/command", 100);
	left_front_wheel_pub = n.advertise<std_msgs::Float64>("corobot_left_front_wheel_controller/command", 100);
	right_front_wheel_pub = n.advertise<std_msgs::Float64>("corobot_right_front_wheel_controller/command", 100);
	odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 50); 

	ros::Subscriber robotVelocity= n.subscribe("PhidgetMotor", 100, setVelocity); //Command received by corobot_teleop or any other controlling node
	ros::Subscriber modelStates = n.subscribe("/gazebo/model_states",100, getModelState);
	ros::Subscriber leftWheelVelocity = n.subscribe("lwheel_vtarget",100, setLeftWheelVelocity);
	ros::Subscriber rightWheelVelocity = n.subscribe("rwheel_vtarget",100, setRightWheelVelocity);	

        ros::spin();


	return 0;
}


