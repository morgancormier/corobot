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
#include "orientation.h"
#include <stdio.h>
#include "corobot_msgs/PosMsg.h"
#include "corobot_msgs/PowerMsg.h"
#include "corobot_msgs/SensorMsg.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int16.h"
#include <phidget21.h>
#include <tf/transform_datatypes.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <corobot_diagnostics/diagnostics.h>
#include <std_srvs/Empty.h>

CPhidgetInterfaceKitHandle ifKit = 0;
CPhidgetSpatialHandle spatial = 0;

Orientation orientation_calculation;

// tells if the rear Bumper is present on the robot or not.
bool m_rearBumperPresent = false; 

//tells if some sonars are connected
bool sonarsPresent = false; 

//tells if the imu are connected
bool imu = true; 

//topics where we want to publish to
ros::Publisher powerdata_pub,irData_pub,bumper_pub, imu_pub, mag_pub, sonar_pub, other_pub; 
	
// Service that when called calibrates the gyroscope
// The service is an empty one, so no data is transmited
ros::ServiceServer calibrate_gyroscope_service;
  
//Output bw for the sonars. -1 if no sonars are present
int bwOutput = -1; 

//Output strobe for the sonars. -1 if no sonars are present
int strobeOutput = -1; 

//last input index for the sonars. -1 if no sonars are present
int lastSonarInput  = -1;

//first input index for the sonars. -1 if no sonars are present 
int firstSonarInput = -1; 
int batteryPort = 0;
int irFrontPort = 1;
int irBackPort = 2;

//for diagnostics purpose
int interfaceKitError = 0, spatialError = 0; 




int ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	ROS_ERROR("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}



/** @brief Description:
Convert a voltage from the Phidgets 8/8/8 board into a distance, for the Infrared sensors
return 25.4 meters as a distance if the value is out of range
otherwise returns a valid distance in meters
**/
static float irVoltageToDistance(float volts)
{
  int sensorValue=int(volts*200.0+0.5);
  float distanceInCm;
//Outside of those bonds, the value is incorrect as our sensor can detect froom 10cm to 80cm only
  if (sensorValue>=80 && sensorValue <= 530) 
    {
      distanceInCm= 2076/(sensorValue-11);
    }
  //out of bonds
  else 
    {
      // 1000 inches in cm
      distanceInCm = 2540; 
    }
  return distanceInCm/100.0;
}


/** 
 * @Brief Voltage to distance function that is used for the sonar sensors
 * @param value value from 0 to 1000 representing a voltage value from 0 to vcc (5V)
 */
static double sonarVoltageToMeters(int value)
{
	// The reported voltage is on the scale Vcc/512 per inch.
	// Because we are using the phidget board, Vcc is always 5
	const double vcc = 5.0;
	double voltage = value * vcc / 1000.0;
	double range_inches = voltage * 512 / vcc;
    
	return range_inches * 2.54 / 100;
}

/** @brief Function that will use use the sonar to later acquire their value
 */
int sendSonarResult()
{
	
	CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 1);
	// sleep for 2ms
	ros::Duration(0.002).sleep(); 
	CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 0);
}


/** @brief callback that will run if an input changes.
 * Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
 */
int DigitalInputHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{   	 
	corobot_msgs::SensorMsg msg;
    	msg.value = ((State == PTRUE)?1:0);
    	msg.index = Index;
	
	//Bumper sensor, front or rear
	if (Index <= 3 && m_rearBumperPresent == true && bumper_pub) 
	{
        	msg.type = msg.BUMPER;
		bumper_pub.publish(msg);
	}
	//Bumper sensor, front
	else if (Index <= 1 && bumper_pub)  
	{
		msg.type = msg.BUMPER;
		bumper_pub.publish(msg);
	}
	// we don't know what it is but we publish
	else if(other_pub) 
	{
		msg.type = msg.OTHER;
		other_pub.publish(msg);
	}
        

	interfaceKitError = 0;
	return 0;
}

//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
int AnalogInputHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
	//sensorValue 0-1000 ==> 0-5V


    // Update power data    
    if(batteryPort == Index && powerdata_pub) 
    {
        corobot_msgs::PowerMsg powerdata;
        powerdata.volts = (float) (Value - 500) * 0.0734;
        //The Min and Max present here are for the Nimh battery as it is the only one type of battery sold with a Corobot at the moment
        powerdata.min_volt = 10.0;
        powerdata.max_volt = 14.2;
        powerdata_pub.publish(powerdata);
    }

    else if(irFrontPort == Index && irData_pub) // Update IR data
    {
	corobot_msgs::SensorMsg data;
	data.type = data.INFRARED_FRONT;
	data.index = Index;

        data.value = irVoltageToDistance((float) Value  / 200.0);	
        irData_pub.publish(data);
    }
        
    else if(irBackPort == Index && irData_pub) // Update IR data
    {
        corobot_msgs::SensorMsg data;
        data.type = data.INFRARED_REAR;
        data.index = Index;
        
        data.value = irVoltageToDistance((float) Value  / 200.0);
	irData_pub.publish(data);
    }
    //sonar
    else if(Index >= firstSonarInput && Index <= lastSonarInput && sonar_pub)
    {
	    corobot_msgs::SensorMsg data;
        data.type = data.ULTRASOUND;
        data.index = Index;
        
        data.value = sonarVoltageToMeters((float) Value  / 200.0);
	sonar_pub.publish(data);
    }
    
    else if(other_pub) // We don't know what sensor it is, but we publish
    {
        corobot_msgs::SensorMsg data;
	    data.type = data.OTHER;
    	data.value = Value;
    	data.index = Index;
        other_pub.publish(data);
    }
    
    
	interfaceKitError = 0;
	return 0;
}


/** 
 * Receive new IMU data, calculate the new orientation and publish data on the topic
 */
int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	sensor_msgs::Imu imu;
	sensor_msgs::MagneticField mag;

	imu.header.frame_id = "base_link";
	imu.header.stamp = ros::Time::now();

	mag.header.frame_id = "base_link";
	mag.header.stamp = ros::Time::now();

	for(int i = 0; i< count; i++)
	{
		if (data[i]->angularRate[0] != 0 || data[i]->angularRate[1] != 0 || data[i]->angularRate[2] != 0)
			orientation_calculation.updateAngles((float*)&(data[i]->angularRate[0]), (float*)&(data[i]->acceleration), data[i]->timestamp.seconds + ((float)data[i]->timestamp.microseconds)/1000000);

        // Save info into the message and publish
		imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(orientation_calculation.get_roll(),orientation_calculation.get_pitch(), orientation_calculation.get_yaw());
		
		// Set up the angular velocity field. the data->angularRate is in deg/sec while we need the unit to be in rad/s
		imu.angular_velocity.x = data[i]->angularRate[0] * (3.14 / 180);
		imu.angular_velocity.y = data[i]->angularRate[1] * (3.14 / 180);
		imu.angular_velocity.z = data[i]->angularRate[2] * (3.14 / 180);
		// Set up the acceleration field. The data->acceleration is in g while we need the unit to be in m/s^2
		imu.linear_acceleration.x = data[i]->acceleration[0] * 9.81;
		imu.linear_acceleration.y = data[i]->acceleration[1] * 9.81;
		imu.linear_acceleration.z = data[i]->acceleration[2] * 9.81;
    // Set up the magnetic_field field. The data->magneticField is in Gauss while we need the unit to be in Tesla.
		mag.magnetic_field.x = data[i]->magneticField[0] * 0.0001;
		mag.magnetic_field.y = data[i]->magneticField[1] * 0.0001;
		mag.magnetic_field.z = data[i]->magneticField[2] * 0.0001;

		if(imu_pub)
			imu_pub.publish(imu);
		if(mag_pub)
			mag_pub.publish(mag);
	}

	spatialError = 0;
	return 0;
}

// Zero the gyroscope when the service is called
// The procedure takes 1-2 seconds and the IMU should not move during the process
bool calibrate_gyroscope(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  CPhidgetSpatial_zeroGyro(spatial);
}


int interfacekit_simple()
// initialize the phidget interface kit board and phidget spatial (imu)
{
	int result, num_analog_inputs, num_digital_inputs;
	const char *err;
	ros::NodeHandle n;

	//create the InterfaceKit object
	CPhidgetInterfaceKit_create(&ifKit);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
	CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, DigitalInputHandler, NULL);
	CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, AnalogInputHandler, NULL);


	//open the interfacekit and spatial for device connections
	CPhidget_open((CPhidgetHandle)ifKit, -1);
	

	CPhidgetInterfaceKit_getInputCount(ifKit, &num_digital_inputs);
	CPhidgetInterfaceKit_getSensorCount(ifKit, &num_analog_inputs);


	printf("Waiting for interface kit to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 1000)))
	{
		CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Phidget IK: Problem waiting for attachment: %s\n", err);
		interfaceKitError = 1;
	}
	else
	{
        	irData_pub = n.advertise<corobot_msgs::SensorMsg>("infrared_data", 100);
        	powerdata_pub = n.advertise<corobot_msgs::PowerMsg>("power_data", 100);
        	bumper_pub = n.advertise<corobot_msgs::SensorMsg>("bumper_data", 100);
		// sensors connected to the phidget interface kit other than bumpers, voltage sensor, ir sensor and sonars. 
        	other_pub = n.advertise<corobot_msgs::SensorMsg>("sensor_data", 100); 
	}	
	

	//Initialize the phidget spatial board, if any
	if (imu)
	{
		CPhidgetSpatial_create(&spatial);
		CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);
		CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);
		CPhidget_open((CPhidgetHandle)spatial, -1);

		// attach the devices
		printf("Waiting for spatial to be attached.... \n");
		if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 1000)))
		{
			CPhidget_getErrorDescription(result, &err);
			ROS_ERROR("Phidget Spatial: Problem waiting for attachment: %s\n", err);
			spatialError = 1;
		}
		else
		{
			imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",100);
			mag_pub = n.advertise<sensor_msgs::MagneticField>("magnetic_data",100);
      calibrate_gyroscope_service = n.advertiseService("calibrate_gyroscope",calibrate_gyroscope);
		}

		CPhidgetSpatial_setDataRate(spatial, 4);
	}

	CPhidgetInterfaceKit_setRatiometric(ifKit, 0);

	//Initialize the sonars, if any are present
	if(sonarsPresent)
	{
		CPhidgetInterfaceKit_setOutputState(ifKit, bwOutput, 1);
		CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 0);
		// sleep for 250ms
		ros::Duration(0.250).sleep(); 
		CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 1);
		// sleep for 2ms
		ros::Duration(0.002).sleep(); 
		CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 0);
		// sleep for 150ms
		ros::Duration(0.150).sleep(); 

		sonar_pub = n.advertise<corobot_msgs::SensorMsg>("sonar_data", 100);
	}
	return 0;
}

void phidget_ik_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
{
	if (!interfaceKitError)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "initialized");
	else
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Phidget 8/8/8 cannot be initialized");
		stat.addf("Recommendation", PhidgetIK_INIT_ERROR);
	}
}

void phidget_spatial_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
{
	if (!spatialError)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "initialized");
	else
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU cannot be initialized");
		stat.addf("Recommendation", PhidgetIMU_INIT_ERROR);
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "phidget_component");
        ros::NodeHandle n;
        ros::NodeHandle nh("~");
        nh.param("rearBumper", m_rearBumperPresent, false);
	nh.param("bwOutput", bwOutput, -1);
	nh.param("strobeOutput", strobeOutput, -1);
	nh.param("lastSonarInput", lastSonarInput, -1);
	nh.param("firstSonarInput", firstSonarInput, -1);
	//index of the battery voltage sensor
	nh.param("battery", batteryPort, 0); 
	// index of the front ir sensor
	nh.param("irFront", irFrontPort, 1); 
	//index of the back ir sensor
	nh.param("irBack", irBackPort, 2); 
	nh.param("imu", imu, true);

	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("Phidget");
	//function that will be executed with updater.update()
	updater.add("Interface Kit", phidget_ik_diagnostic); 
	//function that will be executed with updater.update()
	updater.add("Spatial", phidget_spatial_diagnostic); 	

	interfacekit_simple();
    ros::Rate rate(70);
	while (ros::ok())
    	{
		// ROS loop
        	ros::spinOnce(); 
        
		// acquire new sonar data if sonar sensors are present
		if(sonarsPresent) 
			sendSonarResult();
		//update diagnostics
		updater.update(); 
		rate.sleep();
    	}

	
	// close all the phidget devices
	CPhidget_close((CPhidgetHandle)ifKit);
	CPhidget_delete((CPhidgetHandle)ifKit);
	
	if (imu)
	{
		CPhidget_close((CPhidgetHandle)spatial);
		CPhidget_delete((CPhidgetHandle)spatial);
	}

	return 0;
}
