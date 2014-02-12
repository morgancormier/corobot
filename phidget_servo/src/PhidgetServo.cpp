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
#include <stdio.h>
#include <phidget21.h>

#include "corobot_msgs/ServoPosition.h"
#include "corobot_msgs/ServoType.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <corobot_diagnostics/diagnostics.h>


//Declare a servo handle
CPhidgetAdvancedServoHandle servo = 0;

// Servo position publisher
ros::Publisher position_pub;

// used for diagnostics purpose
int servoError = 0; 


/**
 * @brief Each time a servo motor changes position, this function is called and a message is publish on the /phidgetServo_getPosition topic
 */ 
int PositionChangeHandler(CPhidgetAdvancedServoHandle phid, void *userPtr, int index, double position)
{
	if(position_pub)
	{
		corobot_msgs::ServoPosition position_msg;
		position_msg.index = index;
		position_msg.position = position;
		position_pub.publish(position_msg);
	}
	return 0;
}

/**
 * @brief Callback for the topic /phidgetServo_setPosition
	  The position of the concerned servo motor is set each time a message is received on this topic.
 * @param corobot_msgs::ServoPosition Message
 */ 
void setPositionCallback(const corobot_msgs::ServoPosition &msg)
{
	ROS_INFO("phidget_servo, servo: %d, angle: %f", msg.index, msg.position);
	int err = CPhidgetAdvancedServo_setPosition(servo, msg.index, msg.position);
	if (err != 0)
	{
		ROS_ERROR("Could not set the servo motor number %d to the position %f",msg.index, msg.position);
		servoError = 2;
	}
}

/**
 * @brief Phidget error Callback
 */ 
int ErrorHandler(CPhidgetHandle SERV, void *userptr, int ErrorCode, const char *Description)
{
	ROS_ERROR("Error Phidget Servo motor %d : %s",ErrorCode, Description);
	return 0;
}



/**
 * @brief Callback for the topic /phidgetServo_setType
	  The type of the concerned servo motor is set each time a message is received on this topic.
 * @param corobot_msgs::ServoType Message
 */ 
void setTypeCallback(const corobot_msgs::ServoType &msg)
{	
	CPhidgetAdvancedServo_setServoType(servo, msg.index, (CPhidget_ServoType)msg.type);
}


/**
 * @brief Function that will report the status of the hardware to the diagnostic topic
 */
void servo_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (!servoError)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "intialized");
	else if(servoError == 1)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Phidget servo controller cannot be initialized");
		stat.addf("Recommendation", PHIDGET_SERVO_ERROR_CONNECTION);
	}
	else if(servoError == 2)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "The arm cannot be moved");
		stat.addf("Recommendation", ERROR_MOVING_ARM);
	}

}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "phidget_servomotors");
        ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	int serialNumber = -1;
	double minAccel, maxVel;
	int err;

	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("Phidget");
	//function that will be executed with updater.update()
	updater.add("Servo", servo_diagnostic); 


	//create the servo object
	err = CPhidgetAdvancedServo_create(&servo);
	if (err != 0)
	{
		ROS_ERROR("error create the Phidget servo controller device");
		servoError = 1;
	}

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo,PositionChangeHandler,NULL);

	//serial number of the phidget servo board, -1 if not specified
	n_private.param("serialNumber", serialNumber, -1); 

	//open the servo for device connections
	err = CPhidget_open((CPhidgetHandle)servo, serialNumber);
	if (err != 0)
	{
		ROS_ERROR("error opening the Phidget servo controller device");
		servoError = 1;
	}
	if((err = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
	{
		ROS_ERROR("Problem waiting for attachment of Phidget servo controller");
		servoError = 1;
	}

	//Engage every servo to make sure they are powered on, setup the acceleration value and setup the default position value. 
	for(int i=0;i<8;i++)
	{
		err = CPhidgetAdvancedServo_getAccelerationMin(servo, i, &minAccel);
		if (err != 0)
			ROS_ERROR("error getting the acceleration min of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_setAcceleration(servo, i, minAccel*2);
		if (err != 0)
			ROS_ERROR("error setting the acceleration of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_getVelocityMax(servo, 0, &maxVel);
		if (err != 0)
			ROS_ERROR("error getting the max velocity of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_setVelocityLimit(servo, 0, maxVel/2);
		if (err != 0)
			ROS_ERROR("error setting the lvelocity limit of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_setEngaged(servo,i,1);
		if (err != 0)
			ROS_ERROR("error engaging the servo motor number %d",i);
	}	
	
	//setup the default type of servo motors. It corresponds to 2 old corobot arms connected to the phidget servo board
	err = CPhidgetAdvancedServo_setServoType(servo, 0, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 0");
	err = CPhidgetAdvancedServo_setServoType(servo, 1, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 1");
	err = CPhidgetAdvancedServo_setServoType(servo, 2, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 2");
	err = CPhidgetAdvancedServo_setServoType(servo, 3, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 3");
	err = CPhidgetAdvancedServo_setServoType(servo, 4, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 4");
	err = CPhidgetAdvancedServo_setServoType(servo, 5, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 5");
	err = CPhidgetAdvancedServo_setServoType(servo, 6, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 6");
	err = CPhidgetAdvancedServo_setServoType(servo, 7, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 7");



	//Declare every topics
	position_pub = n.advertise<corobot_msgs::ServoPosition>("phidgetServo_getPosition", 100);

	//Subscribe to every necessary topics
        ros::Subscriber position_sub = n.subscribe("phidgetServo_setPosition",100, &setPositionCallback);
        ros::Subscriber type_sub = n.subscribe("phidgetServo_setType",100, &setTypeCallback);
	
	while (ros::ok())
            {
		updater.update();
                ros::spinOnce();
            }


	//Go back to the default position and disengage every servo motor before closing the servo handle
	for(int i=0;i<8;i++)
	{
		err = CPhidgetAdvancedServo_setPosition (servo, i, 0.00);
		err += CPhidgetAdvancedServo_setEngaged(servo,i,0);
		if (err != 0)
			ROS_ERROR("error closing the servo motor number : %d", i);
	}

	CPhidget_close((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);

}
