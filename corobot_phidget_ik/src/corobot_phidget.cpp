// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/


#include "ros/ros.h"
#include "orientation.h"
#include <stdio.h>
#include "corobot_msgs/PosMsg.h"
#include "corobot_msgs/PowerMsg.h"
#include "corobot_msgs/SensorMsg.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include <phidget21.h>
#include <tf/transform_datatypes.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

CPhidgetEncoderHandle m_rightEncoder;
CPhidgetEncoderHandle m_leftEncoder;
CPhidgetInterfaceKitHandle ifKit = 0;
CPhidgetSpatialHandle spatial = 0;

Orientation orientation_calculation;

bool m_encoder1Seen=false, m_encoder2Seen=false,m_encodersGood = false;//useful to set up the phidget
int m_leftEncoderNumber,m_rightEncoderNumber;

bool m_rearBumperPresent = false; // tells if the rear Bumper is present on the robot or not.
bool sonarsPresent = false; //tells if some sonars are connected

ros::Publisher posdata_pub,powerdata_pub,irData_pub,bumper_pub,spatial_pub, imu_pub, mag_pub, sonar_pub, other_pub; //topics where we want to publish to

int bwOutput = -1; //Output bw for the sonars. -1 if no sonars are present
int strobeOutput = -1; //Output strobe for the sonars. -1 if no sonars are present
int lastSonarInput  = -1; //last input index for the sonars. -1 if no sonars are present
int firstSonarInput = -1; //first input index for the sonars. -1 if no sonars are present
int batteryPort = 0;
int irFrontPort = 1;
int irBackPort = 2;

bool motors_inverted; //specify if the wiring of the robot (motors + encoders) has been inverted
bool encoders_inverted; //specify if the wiring of the encoders has been inverted

int interfaceKitError = 0, spatialError = 0, encoderError = 0; //for diagnostics purpose


int RightEncoderAttach(CPhidgetHandle phid, void *userPtr);


int ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	ROS_ERROR("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

void encoderAttach(const int which)
        /**
         * @brief function that handles registering the an encoder has been attached
         * @param which indicates which encoder was seen (1 or 2)
         **/
{
    if (which == 1)
    {
        m_encoder1Seen = true;
        // this could either be the left encoder board, or it's a single board that
        // supports both encoders. Check how many inputs it has
        int count;
        CPhidgetEncoder_getEncoderCount(m_leftEncoder, &count);
        if (count > 1)
        {
            // we have only one board that supports both motor encoders
            m_leftEncoderNumber = 0;
            m_rightEncoderNumber = 1;
		
            if(motors_inverted || encoders_inverted){
            	m_leftEncoderNumber = 1;
            	m_rightEncoderNumber = 0;
	    }
		
            m_rightEncoder = m_leftEncoder;
            m_encoder2Seen = true;
            m_encodersGood = true;
        }
        else
        {
            m_leftEncoderNumber = 0;
            m_rightEncoderNumber = 0;
            // open the second encoder (assume it's the right one)
            CPhidgetEncoder_create(&m_rightEncoder);
            CPhidget_set_OnAttach_Handler((CPhidgetHandle) m_rightEncoder,
                                          RightEncoderAttach, NULL);
            CPhidget_open((CPhidgetHandle) m_rightEncoder, -1);
        }
    }
    else
    {
        m_encoder2Seen = true;
    }
    // have we seen both of the encoders?
    if (m_encoder1Seen && m_encoder2Seen)
    {
        // now check the assumption we have the encoders correct
        int leftSerial;
        int rightSerial;

        CPhidget_getSerialNumber((CPhidgetHandle) m_leftEncoder, &leftSerial);
        CPhidget_getSerialNumber((CPhidgetHandle) m_rightEncoder, &rightSerial);

        if (leftSerial > rightSerial)
        {
            // oops, we go them backwards so swap them
            CPhidgetEncoderHandle tempEncoder = m_leftEncoder;
            m_leftEncoder = m_rightEncoder;
            m_rightEncoder = tempEncoder;

            int tempNumber = m_leftEncoderNumber;
            m_leftEncoderNumber = m_rightEncoderNumber;
            m_rightEncoderNumber = tempNumber;
        }
	CPhidgetEncoder_setEnabled(m_leftEncoder, m_leftEncoderNumber, PTRUE);
	CPhidgetEncoder_setEnabled(m_rightEncoder, m_rightEncoderNumber, PTRUE);
        m_encodersGood = true;

    }
}
/**
 * @brief callback invoked automatically when the right encoder is detected

 **/
int RightEncoderAttach(CPhidgetHandle phid, void *userPtr)

{
	encoderAttach(2);
	return 0;
}
/**
 * @brief callback invoked automatically when the left encoder is detected
 **/
int LeftEncoderAttach(CPhidgetHandle phid, void *userPtr)

{
	encoderAttach(1);
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
  if (sensorValue>=80 && sensorValue <= 500) //Outside of those bonds, the value is incorrect as our sensor can detect froom 10cm to 80cm only
    {
      distanceInCm= 4800/(sensorValue-20);
    }
  else //out of bonds
    {
      distanceInCm = 2540; // 1000 inches in cm
    }
  return distanceInCm/100.0;
}


/**
 * @brief Publish the encoder data on the topic
 */
int publish_encoder(){


    if(m_encodersGood) //encoder data
    {
        corobot_msgs::PosMsg posdata;

        // prepare to read the encoder status
        int phidgetEncoderStatus = 0;

        CPhidget_getDeviceStatus((CPhidgetHandle) m_leftEncoder,
                                 &phidgetEncoderStatus);
        if (phidgetEncoderStatus != 0)
        {
            int value;

            CPhidgetEncoder_getPosition(m_leftEncoder, m_leftEncoderNumber, &value);
                    posdata.px = -value;
	    printf("Left encoder value = %d", value);

        }
	else
	{
	    encoderError = 2;
	}
        phidgetEncoderStatus = 0;
        CPhidget_getDeviceStatus((CPhidgetHandle) m_rightEncoder,
                                 &phidgetEncoderStatus);
        if (phidgetEncoderStatus != 0)
        {
            int value;

            CPhidgetEncoder_getPosition(m_rightEncoder, m_rightEncoderNumber, &value);
            // we negate this value since the right side motors turn the opposite direction
            // to the left side
                    posdata.py = value;
        }
	else
	{
	    encoderError = 2;
	}

	posdata.header.stamp = ros::Time::now();
        posdata_pub.publish(posdata);
	encoderError = 0;
    }
    else
    {
	encoderError = 1;
    }

    return 0;
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
	ros::Duration(0.002).sleep(); // sleep for 2ms
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

	if (Index <= 3 && m_rearBumperPresent == true) //Bumper sensor, front or rear
	{
        	msg.type = msg.BUMPER;
		bumper_pub.publish(msg);
	}
	else if (Index <= 1) //Bumper sensor, front 
	{
		msg.type = msg.BUMPER;
		bumper_pub.publish(msg);
	}
	else // we don't know what it is but we publish
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


        
    if(batteryPort == Index) //// Update power data
    {
        corobot_msgs::PowerMsg powerdata;
        powerdata.volts = (float) (Value - 500) * 0.0734;
        //The Min and Max present here are for the Nimh battery as it is the only one type of battery sold with a Corobot at the moment
        powerdata.min_volt = 10.0;
        powerdata.max_volt = 14.2;
        powerdata_pub.publish(powerdata);
    }

    else if(irFrontPort == Index) // Update IR data
    {
	corobot_msgs::SensorMsg data;
	data.type = data.INFRARED_FRONT;
	data.index = Index;

        data.value = irVoltageToDistance((float) Value  / 200.0);	
        irData_pub.publish(data);
    }
        
    else if(irBackPort == Index) // Update IR data
    {
        corobot_msgs::SensorMsg data;
        data.type = data.INFRARED_REAR;
        data.index = Index;
        
        data.value = irVoltageToDistance((float) Value  / 200.0);
	irData_pub.publish(data);
    }
    else if(Index >= firstSonarInput && Index <= lastSonarInput)//sonar
    {
	corobot_msgs::SensorMsg data;
        data.type = data.ULTRASOUND;
        data.index = Index;
        
        data.value = sonarVoltageToMeters((float) Value  / 200.0);
	sonar_pub.publish(data);
    }
    
    else // We don't know what sensor it is, but we publish
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
		imu.angular_velocity.x = data[i]->angularRate[0];
		imu.angular_velocity.y = data[i]->angularRate[1];
		imu.angular_velocity.z = data[i]->angularRate[2];
		imu.linear_acceleration.x = data[i]->acceleration[0];
		imu.linear_acceleration.y = data[i]->acceleration[1];
		imu.linear_acceleration.z = data[i]->acceleration[2];

		mag.magnetic_field.x = data[i]->magneticField[0];
		mag.magnetic_field.y = data[i]->magneticField[1];
		mag.magnetic_field.z = data[i]->magneticField[2];

		imu_pub.publish(imu);
		mag_pub.publish(mag);
	}

	spatialError = 0;
	return 0;
}




int interfacekit_simple()
// initialize the phidget interface kit board and phidget spatial (imu)
{
	int result, num_analog_inputs, num_digital_inputs;
	const char *err;

	//create the InterfaceKit object
	CPhidgetInterfaceKit_create(&ifKit);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
	CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, DigitalInputHandler, NULL);
	CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, AnalogInputHandler, NULL);

	//Initialize the phidget spatial board, if any
	CPhidgetSpatial_create(&spatial);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);


	//open the interfacekit and spatial for device connections
	CPhidget_open((CPhidgetHandle)ifKit, -1);
	CPhidget_open((CPhidgetHandle)spatial, -1);


	CPhidgetInterfaceKit_getInputCount(ifKit, &num_digital_inputs);
	CPhidgetInterfaceKit_getSensorCount(ifKit, &num_analog_inputs);


	// attach the devices
	printf("Waiting for spatial to be attached.... \n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 1000)))
	{
		CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Phidget Spatial: Problem waiting for attachment: %s\n", err);
		spatialError = 1;
	}

	printf("Waiting for interface kit to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 1000)))
	{
		CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Phidget IK: Problem waiting for attachment: %s\n", err);
		interfaceKitError = 1;
	}
	
	CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
	CPhidgetSpatial_setDataRate(spatial, 16);	

	// create and attach the encoders
	 CPhidgetEncoder_create(&m_leftEncoder);
         CPhidget_set_OnAttach_Handler((CPhidgetHandle) m_leftEncoder,LeftEncoderAttach, NULL);
         CPhidget_open((CPhidgetHandle) m_leftEncoder, -1);

	//Initialize the sonars, if any are present
	if(sonarsPresent)
	{
		CPhidgetInterfaceKit_setOutputState(ifKit, bwOutput, 1);
		CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 0);
		ros::Duration(0.250).sleep(); // sleep for 250ms
		CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 1);
		ros::Duration(0.002).sleep(); // sleep for 2ms
		CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 0);
		ros::Duration(0.150).sleep(); // sleep for 150ms
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
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot be attached");
		stat.addf("Recommendation", "Please unplug and replug the Phidget Interface Kit Board USB cable from the Motherboard.");
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
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot be attached");
		stat.addf("Recommendation", "Please verify that the robot has a Phidget Spatial board. If present, please unplug and replug the Phidget Spatial Board USB cable from the Motherboard.");
	}
}

void phidget_encoder_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
{
	if (!encoderError)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "intialized");
	else if(encoderError == 1)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot be attached");
		stat.addf("Recommendation", "Please verify that the robot has a Phidget Encoder board. If present, please unplug and replug the Phidget Spatial Board USB cable from the Motherboard.");
	}
	else if(encoderError == 2)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot read");
		stat.addf("Recommendation", "Please verify that the two encoders are connected to the Phidget Encoder Board.");
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
	nh.param("battery", batteryPort, 0); //index of the battery voltage sensor
	nh.param("irFront", irFrontPort, 1); // index of the front ir sensor
	nh.param("irBack", irBackPort, 2); //index of the back ir sensor
	nh.param("motors_inverted", motors_inverted, false);
	nh.param("encoders_inverted", encoders_inverted, false);

	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("Phidget");
	updater.add("Interface Kit", phidget_ik_diagnostic); //function that will be executed with updater.update()
	updater.add("Spatial", phidget_spatial_diagnostic); //function that will be executed with updater.update()
	updater.add("Encoders", phidget_encoder_diagnostic); //function that will be executed with updater.update()
	interfacekit_simple(); 

	posdata_pub = n.advertise<corobot_msgs::PosMsg>("position_data", 100);
        irData_pub = n.advertise<corobot_msgs::SensorMsg>("infrared_data", 100);
        powerdata_pub = n.advertise<corobot_msgs::PowerMsg>("power_data", 100);
        bumper_pub = n.advertise<corobot_msgs::SensorMsg>("bumper_data", 100);
        sonar_pub = n.advertise<corobot_msgs::SensorMsg>("sonar_data", 100);
	imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",100);
	mag_pub = n.advertise<sensor_msgs::MagneticField>("magnetic_data",100);
        other_pub = n.advertise<corobot_msgs::SensorMsg>("sensor_data", 100); // sensors connected to the phidget interface kit other than bumpers, voltage sensor, ir sensor and sonars. 


	while (ros::ok())
    {
        ros::spinOnce(); // ROS loop
        
		if(sonarsPresent) // acquire new sonar data if sonar sensors are present
			sendSonarResult();
		publish_encoder(); // acquire and publish encoder data
		updater.update(); //update diagnostics
    }

	
	// close all the phidget devices
	CPhidget_close((CPhidgetHandle)ifKit);
	CPhidget_delete((CPhidgetHandle)ifKit);
	if (m_rightEncoder != m_leftEncoder)
	{
		CPhidget_close((CPhidgetHandle)m_leftEncoder);
		CPhidget_delete((CPhidgetHandle)m_leftEncoder);
	}
	CPhidget_close((CPhidgetHandle)m_rightEncoder);
	CPhidget_delete((CPhidgetHandle)m_rightEncoder);
	CPhidget_close((CPhidgetHandle)spatial);
	CPhidget_delete((CPhidgetHandle)spatial);


	return 0;
}
