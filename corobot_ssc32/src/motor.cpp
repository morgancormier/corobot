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

#include "ros/ros.h"
#include "ssc32.hpp"
#include "corobot_msgs/MotorCommand.h"
#include "corobot_msgs/ServoPosition.h"
#include <math.h>
#include <errno.h>
#include <sys/stat.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <corobot_diagnostics/diagnostics.h>

using namespace Servo; 

SSC32 m_ssc32;

std::string SSC32_PORT;
ros::Timer timer;

// use for diagnostics purpose
int ssc32Error = 0; 

// Inform us on which servo has already been moved at least once.
// The size has been set to 20 to make sure there is enough place for all the servos. 
// We want to avoid dynamic memory control to be faster
bool first_time_command[20] = {false};



// stop the motors after the requested time
// This is used only if some dc motors are connected to the ssc32 board
void timerCallback(const ros::TimerEvent&)
{
	bool ret;	

	ret = m_ssc32.SendMessage(1,0,0);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;

	ret = m_ssc32.SendMessage(0,0,0);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;
}

// set the motors requested speed. 
// This is used only if some dc motors are connected to the ssc32 board
void SetSpeedTopic(const corobot_msgs::MotorCommand::ConstPtr &msg)
{
	ros::NodeHandle n;
	bool ret;

	ret = m_ssc32.SendMessage(1,(msg->leftSpeed*-5)+1500,(100-msg->acceleration)*10);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;

	m_ssc32.SendMessage(0,(msg->rightSpeed*-5)+1500,(100-msg->acceleration)*10);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;

	timer = n.createTimer(ros::Duration(msg->secondsDuration), timerCallback, true);
}


/**
 * @brief Callback for the topic /setPositionServo
	  The position of the concerned servo motor is set each time a message is received on this topic.
 * @param corobot_msgs::ServoPosition Message
 */ 
void setPositionCallback(const corobot_msgs::ServoPosition &msg)
{
	//ideally, msg.position is between 0 and 150 degrees
	int amount =  (int) (msg.position / 180 * 1500) + 750;  

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
	{
		ROS_ERROR("Could not set the servo motor number %d to the position %f",msg.index, msg.position);
		ssc32Error = 2;	
	}
}

/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
void ssc32_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (!ssc32Error)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "initialized");
	else if (ssc32Error == 1)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "ssc32 cannot be initialized");
		stat.addf("Recommendation",SSC32_ERROR_CONNECTION);
	}
	else if (ssc32Error == 2)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot move arm");
		stat.addf("Recommendation", ERROR_MOVING_ARM);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssc32_node");
	ros::NodeHandle n("~");


	//Subscribe to topics
	ros::Subscriber velocity=n.subscribe<corobot_msgs::MotorCommand>("/ssc32_velocity",1000, SetSpeedTopic);
	ros::Subscriber position_sub = n.subscribe("/setPositionServo",100, &setPositionCallback);


	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("SSC32");
	//function that will be executed with updater.update()
	updater.add("SSC32", ssc32_diagnostic); 

	std::string ssc32_port_;

   	n.param<std::string>("ssc32_port", ssc32_port_, "/dev/ttyS1");

    	if(!n.getParam("ssc32_port", ssc32_port_))
    		ROS_INFO("can not get parameter ssc32_port!");

	SSC32_PORT = ssc32_port_;

	// Let's give permission to do anything on the ssc32 port, or else we won't be able to use it. Commented because the node would need to be executed in super user. 
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
	  ssc32Error = 0;
	}
	else
	  ssc32Error = 1; 

	//main ros function
	while (ros::ok())
        {
                ros::spinOnce();
		// Diagnostic 
		updater.update();
	}

        m_ssc32.stopSerial();
        
	return 0;
}
