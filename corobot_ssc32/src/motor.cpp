#include "ros/ros.h"
#include "ssc32.hpp"
#include "corobot_msgs/ssc32_info.h"
#include "corobot_msgs/MotorCommand.h"
#include "corobot_msgs/ServoPosition.h"
#include <math.h>
#include <errno.h>
#include <sys/stat.h>


using namespace Servo;

SSC32 m_ssc32;

std::string SSC32_PORT;
ros::Timer timer;

bool first_time_command[20] = {false};




void timerCallback(const ros::TimerEvent&)
{
	m_ssc32.SendMessage(1,0,0);
	m_ssc32.SendMessage(0,0,0);
}

void SetSpeedTopic(const corobot_msgs::MotorCommand::ConstPtr &msg)
{
	ros::NodeHandle n;
	m_ssc32.SendMessage(1,(msg->leftSpeed*-5)+1500,(100-msg->acceleration)*10);
	m_ssc32.SendMessage(0,(msg->rightSpeed*-5)+1500,(100-msg->acceleration)*10);
	timer = n.createTimer(ros::Duration(msg->secondsDuration), timerCallback, true);
}


/**
 * @brief Callback for the topic /phidgetServo_setPosition
	  The position of the concerned servo motor is set each time a message is received on this topic.
 * @param corobot_msgs::ServoPosition Message
 */ 
void setPositionCallback(const corobot_msgs::ServoPosition &msg)
{
	int amount =  (int) (msg.position / 180 * 1500) + 750;  //ideally, msg.position is between 0 and 150 degrees

	// The first command should not contain any speed, so we are not sending any
	bool ok;
	if(first_time_command[msg.index] == false)
	{
		ok = m_ssc32.SendMessage(msg.index,amount,-1);
		first_time_command[msg.index] = true;
	}
	else
	{
		ok = m_ssc32.SendMessage(msg.index,amount,2000);
	}
	
	if (!ok)
		ROS_ERROR("Could not set the servo motor number %d to the position %f",msg.index, msg.position);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssc32_node");
	ros::NodeHandle n("~");

	//Advertise topics
	ros::Publisher ssc32_info_pub = n.advertise<corobot_msgs::ssc32_info>("ssc32_info", 1000);

	//Subscribe to topics
	ros::Subscriber velocity=n.subscribe<corobot_msgs::MotorCommand>("/ssc32_velocity",1000, SetSpeedTopic);
	ros::Subscriber position_sub = n.subscribe("/setPositionServo",100, &setPositionCallback);

	std::string ssc32_port_;

   	n.param<std::string>("ssc32_port", ssc32_port_, "/dev/ttyS1");

    	if(!n.getParam("ssc32_port", ssc32_port_))
    		ROS_INFO("can not get parameter ssc32_port!");

	SSC32_PORT = ssc32_port_;

	// Let's give permission to do anything on the ssc32 port, or else we won't be able to use it.
	/*char mode[] = "0777";
    	int i;
    	i = strtol(mode, 0, 8);
    	if (chmod (SSC32_PORT.c_str(),i) < 0)
    	{
        	ROS_ERROR("%s: error in chmod(%s, %s) - %d (%s)\n", argv[0], SSC32_PORT.c_str(), mode, errno, strerror(errno));
    	}
	*/
	//Let's now initialize the ssc32 and connect to it
	m_ssc32.initSerial();

	if(m_ssc32.startSerial(SSC32_PORT))
	{
	  ROS_INFO("Success!!! Connect to serial port");
	  corobot_msgs::ssc32_info info;
	  info.connected = 1;
	  ssc32_info_pub.publish(info);
	}

	//main ros function
        ros::spin();

        m_ssc32.stopSerial();
        
	return 0;
}
