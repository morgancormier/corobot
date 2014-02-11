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


ros::Publisher driveControl_pub,takepic_pub,pan_tilt_control, move_arm_pub;

int speed_left, speed_right, speed_value;
bool turningLeft, turningRight;
int pan_value,tilt_value;
double ory, orz, orx;
int gripper_state; //0 = open, 1 = closed
int save_image_state = 0;

void velocityCallback(const std_msgs::Int32::ConstPtr& msg)
{
	speed_value = msg->data;
}
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//********************************************
//Motor control

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
    msg.secondsDuration = 3;
    msg.acceleration = 50;
    driveControl_pub.publish(msg);

  if(joy->buttons[8]) // STOP
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
 if(joy->buttons[11]) // right Stick click Take Picture
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
  if(joy->axes[2]>0.5) // Pan control
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

  if(joy->axes[2]<-0.5) // Pan control
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

  if(joy->axes[3]<-0.5) // Tilt control
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

  if(joy->axes[3]>0.5) // Tilt control
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

 if(joy->buttons[9]) // PTZ reset
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
if(joy->axes[4]>0.5) // Shoulder control
  {
	if(ory > 0.7)
	{
    		ory -= M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::SHOULDER;
		msg1.position = ory * 180 / M_PI; // The value has to be in degrees

		move_arm_pub.publish(msg1);
	}
  }

  if(joy->axes[4]<-0.5) // Shoulder control
  {
    
	if( ory < 2)
	{
		ory += M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::SHOULDER;
		msg1.position = ory * 180 / M_PI; // The value has to be in degrees

		move_arm_pub.publish(msg1);
	}
  }

  if(joy->axes[5]<-0.5) // Elbow control
  {
    
    if( orz > 0.8)
    {
		orz -= M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::ELBOW;
		msg1.position = orz * 180 / M_PI; // The value has to be in degrees

		move_arm_pub.publish(msg1);
	}
  }

  if(joy->axes[5]>0.5) // Elbow control
  {
    
    if( orz < 2.5)
    {
		orz += M_PI/8;

		MoveArm msg1;
		msg1.index = MoveArm::ELBOW;
		msg1.position = orz * 180 / M_PI; // The value has to be in degrees

		move_arm_pub.publish(msg1);
	}
  }

 if(joy->buttons[7]) // arm reset
  {
    	ory = orz = M_PI/2;

	MoveArm msg1;
	msg1.index = MoveArm::ELBOW;
	msg1.position = orz * 180 / M_PI; // The value has to be in degrees

	move_arm_pub.publish(msg1);

	msg1.index = MoveArm::SHOULDER;
	msg1.position = ory * 180 / M_PI; // The value has to be in degrees

	move_arm_pub.publish(msg1);
  }
//******************************************
//wrist control
 if(joy->buttons[4]) // Wrist Left
  {
    
    if(orx < 4.5)
    {
	  orx += 0.5;

	  MoveArm msg1;
	  msg1.index = MoveArm::WRIST_ROTATION;
	  msg1.position = orx * 180 / M_PI; // The value has to be in degrees

	  move_arm_pub.publish(msg1);
	}
  }

 if(joy->buttons[5]) // Wrist Right
  {
    if(orx > 0.1)
    {
	  orx -= 0.5;

   	  MoveArm msg1;
	  msg1.index = MoveArm::WRIST_ROTATION;
	  msg1.position = orx * 180 / M_PI; // The value has to be in degrees

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
    msg1.position = (gripper_state * 180); // The value has to be in degrees

    move_arm_pub.publish(msg1);

  }

}


int main(int argc, char** argv)
{
  
  pan_value = 0;
  tilt_value = 0;
  orx = 2.3;//initial value for wrist
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
  ros::Subscriber velocity = n.subscribe<std_msgs::Int32>("velocityValue", 1000, velocityCallback); //choose the maximum speed for the robot. value is between 0 and 100, 100 beeing the fastest possible

  ros::spin();
}
