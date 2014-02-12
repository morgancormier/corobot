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
#include <sensor_msgs/Joy.h>
#include <corobot_msgs/MoveArm.h>
#include <corobot_msgs/MotorCommand.h>
#include <corobot_msgs/takepic.h>
#include <corobot_msgs/PanTilt.h>
#include <std_msgs/Int32.h>
#include <math.h>

/**
 * This node gets the information on the buttons and joystick of the gamepad used by the user and process it.
 */


using namespace corobot_msgs;

// Publishers that send commands to other packages after processing the joystick actions
ros::Publisher driveControl_pub,takepic_pub,pan_tilt_control, move_arm_pub;

// used to save some interesting valuesm such as speed, camera position... 
int speed_left, speed_right, speed_value;
bool turningLeft, turningRight;
int pan_value,tilt_value;
double ory, orz, orx;

//0 = open, 1 = closed
int gripper_state; 
int save_image_state = 0;

void velocityCallback(const std_msgs::Int32::ConstPtr& msg)
{
	speed_value = msg->data;
}

// Receives the joystick actions and process them
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//********************************************
//Motor control

	// the speed_left and speed_right parameters have to be between -100 and +100. The joy->axes value is between -1 and 1.
	speed_left = joy->axes[1] * 100;
	speed_right = joy->axes[1] * 100;
	speed_left += -joy->axes[0] * 100;
	speed_right += joy->axes[0] * 100;

	if(speed_right >100)
		speed_right = 100;
	if(speed_right < -100)
		speed_right = -100;
	if(speed_left >100)
		speed_left = 100;
	if(speed_left < -100)
		speed_left = -100;

    corobot_msgs::MotorCommand msg;
    msg.leftSpeed = speed_left;
    msg.rightSpeed = speed_right;
    // we want to drive for 3s only, so that if the connection is lost the robot will automatically stop after that time
    msg.secondsDuration = 3;
    msg.acceleration = 50;
    driveControl_pub.publish(msg);
 //stop
  if(joy->buttons[8]) 
  {
        corobot_msgs::MotorCommand msg;
        msg.leftSpeed = 0;
        msg.rightSpeed = 0;
        msg.secondsDuration = 0;
        msg.acceleration = 50;
        driveControl_pub.publish(msg);
  }
//*********************************************************
//Take picture
// right Stick click Take Picture
 if(joy->buttons[11]) 
  {
    if(save_image_state == 0)
    {
        takepic msg;
        msg.camera_index = 1;
        msg.take = true;
        takepic_pub.publish(msg);
	save_image_state = 1;
    }
  }
  else
	save_image_state = 0;

//********************************************************
// Pan Tilt Control	
// Pan control going to left
  if(joy->axes[2]>0.5) 
  {
	if(pan_value > -70)
	{
    		pan_value -= 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }
// Pan control going to right
  if(joy->axes[2]<-0.5) 
  {
	if(pan_value < 70)
	{
    	pan_value += 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }
// Tilt control going down
  if(joy->axes[3]<-0.5) 
  {
	if(tilt_value > -30)
	{
    	tilt_value -= 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }
// Tilt control going up
  if(joy->axes[3]>0.5) 
  {
	if(tilt_value < 30)
	{
    	tilt_value += 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }
// reset the pan and tilt position
 if(joy->buttons[9]) 
  {
    tilt_value = 0;
    pan_value = 0;
    PanTilt msg;
    msg.pan = pan_value;
    msg.tilt = tilt_value;
    msg.reset = 0;
    pan_tilt_control.publish(msg);
  }

//*****************************************************

//*****************************************************
//Arm control

// Shoulder control
if(joy->axes[4]>0.5) 
  {
	if(ory > 0.7)
	{
    		ory -= M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::SHOULDER;
		// The value has to be in degrees
		msg1.position = ory * 180 / M_PI; 

		move_arm_pub.publish(msg1);
	}
  }
// Shoulder control
  if(joy->axes[4]<-0.5) 
  {
    
	if( ory < 2)
	{
		ory += M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::SHOULDER;
		// The value has to be in degrees
		msg1.position = ory * 180 / M_PI; 

		move_arm_pub.publish(msg1);
	}
  }
// Elbow control
  if(joy->axes[5]<-0.5) 
  {
    
    if( orz > 0.8)
    {
		orz -= M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::ELBOW;
		// The value has to be in degrees
		msg1.position = orz * 180 / M_PI; 

		move_arm_pub.publish(msg1);
	}
  }
// Elbow control
  if(joy->axes[5]>0.5) 
  {
    
    if( orz < 2.5)
    {
		orz += M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::ELBOW;
		// The value has to be in degrees
		msg1.position = orz * 180 / M_PI; 

		move_arm_pub.publish(msg1);
	}
  }
// arm reset
 if(joy->buttons[7]) 
  {
    	ory = orz = M_PI/2;

	MoveArm msg1;
	msg1.index = MoveArm::ELBOW;
	// The value has to be in degrees
	msg1.position = orz * 180 / M_PI; 

	move_arm_pub.publish(msg1);

	msg1.index = MoveArm::SHOULDER;
	// The value has to be in degrees
	msg1.position = ory * 180 / M_PI; 

	move_arm_pub.publish(msg1);
  }
//******************************************
//wrist control

// Wrist Left
 if(joy->buttons[4]) 
  {
    
    if(orx < 4.5)
    {
	  orx += 0.5;

	  MoveArm msg1;
	  msg1.index = MoveArm::WRIST_ROTATION;
	  // The value has to be in degrees
	  msg1.position = orx * 180 / M_PI; 

	  move_arm_pub.publish(msg1);
	}
  }
// Wrist Right
 if(joy->buttons[5]) 
  {
    if(orx > 0.1)
    {
	  orx -= 0.5;

   	  MoveArm msg1;
	  msg1.index = MoveArm::WRIST_ROTATION;
	  // The value has to be in degrees
	  msg1.position = orx * 180 / M_PI; 
	  move_arm_pub.publish(msg1);
	}
  }
//****************************************************
//gripper control
  if(joy->buttons[6])
  {

    if(gripper_state == 0)
	gripper_state = 1;
    else if(gripper_state == 1)
	gripper_state = 0;

    MoveArm msg1;
    msg1.index = MoveArm::GRIPPER;
    // The value has to be in degrees
    msg1.position = (gripper_state * 180); 

    move_arm_pub.publish(msg1);

  }

}


int main(int argc, char** argv)
{
  
  pan_value = 0;
  tilt_value = 0;
  //initial value for wrist
  orx = 2.3;
  ory = M_PI/2;
  orz = M_PI/2;
  gripper_state = 0;
  speed_left = 0;
  speed_right = 0;
  speed_value = 75;
  turningLeft = false;
  turningRight = false;

  ros::init(argc, argv, "corobot_joystick");
  
  ros::NodeHandle n;

// Advertize topics used to control the robot
  driveControl_pub = n.advertise<corobot_msgs::MotorCommand>("PhidgetMotor", 100);
  takepic_pub = n.advertise<takepic>("takepicture",100);
  pan_tilt_control = n.advertise<PanTilt>("pantilt",10);
  move_arm_pub = n.advertise<MoveArm>("armPosition",10);

  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);
  //choose the maximum speed for the robot. value is between 0 and 100, 100 beeing the fastest possible
  ros::Subscriber velocity = n.subscribe<std_msgs::Int32>("velocityValue", 1000, velocityCallback);
  
  ros::spin();
}
