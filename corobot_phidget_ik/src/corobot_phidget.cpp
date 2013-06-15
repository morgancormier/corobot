// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/



#include "ros/ros.h"
#include <stdio.h>
#include "corobot_msgs/PosMsg.h"
#include "corobot_msgs/PowerMsg.h"
#include "corobot_msgs/IrMsg.h"
#include "corobot_msgs/BumperMsg.h"
#include "corobot_msgs/GripperMsg.h"
#include "corobot_msgs/phidget_info.h"
#include "corobot_msgs/spatial.h"
#include "corobot_msgs/RangeSensor.h"
#include "sensor_msgs/Imu.h"
#include <phidget21.h>
#include <tf/transform_datatypes.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define GRAVITY 9.81

CPhidgetEncoderHandle m_rightEncoder;
CPhidgetEncoderHandle m_leftEncoder;
CPhidgetInterfaceKitHandle ifKit = 0;
CPhidgetSpatialHandle spatial = 0;

int  analog_inputs_value [8] = {0,0,0,0,0,0,0,0};
char digital_inputs_value = 0;
float acc [3] = {0,0,0};
float ang [3] = {0,0,0};
float mag [3] = {0,0,0};

bool m_encoder1Seen=false, m_encoder2Seen=false,m_encodersGood = false;//usefull to set up the phidget
bool m_validAnalogs = false,m_validDigitals=false; //tells if we received any data from the phidget or not.
int m_leftEncoderNumber,m_rightEncoderNumber;

bool m_rearBumperPresent = false; // tells if the rear Bumper is present on the robot or not.
bool sonarsPresent = false; //tells if some sonars are connected

bool phidget888_connected, phidget_encoder_connected;

bool spatial_good = false;

ros::Publisher posdata_pub,powerdata_pub,irData_pub,bumper_pub,gripper_pub,spatial_pub, imu_pub, sonar_pub; //topics where we want to publish to

int bwOutput = -1; //Output bw for the sonars. -1 if no sonars are present
int strobeOutput = -1; //Output strobe for the sonars. -1 if no sonars are present
int lastSonarInput  = -1; //last input index for the sonars. -1 if no sonars are present
int firstSonarInput = -1; //first input index for the sonars. -1 if no sonars are present
int batteryPort = 0;
int irFrontPort = 1;
int irBackPort = 2;
int gripperPort = 3;

bool motors_inverted; //specify if the wiring of the robot (motors + encoders) has been inverted
bool encoders_inverted; //specify if the wiring of the encoders has been inverted

int interfaceKitError = 0, spatialError = 0, encoderError = 0;

int RightEncoderAttach(CPhidgetHandle phid, void *userPtr);
int LeftEncoderAttach(CPhidgetHandle phid, void *userPtr);

double dCMMatrix[3][3]; //Matrix essential for calculating the orientation with IMU data
int timestampPreviousCall; //timestamp of the measurement got at the last call of the function
double pitchOffset; //offset necessary go from angle in the intial device position coordinate system to the User coordinate system
double rollOffset; //offset necessary go from angle in the intial device position coordinate system to the User coordinate system
double gravity[3]; //We save the gravity to know if the gravity we got is different from before or not
double proportionalVector[3];//Proportional correction
double integratorVector[3];//Omega Integrator
double gyroscopeVectorCorrected[3]; //Corrected GyroVector data
double kpRollpitch = 0.015; //Used in a PID controller to calculate the roll and pitch angles
double kiRollpitch = 0.000010; //Used in a PID controller to calculate the roll and pitch angles
double gravityEpsilon = 0.07;
int validGravityCounter = 0; //Counter the number of valid gravity vector(valid in the sense that the norm is near 1) that follow each other
int validAccelerationVectorsNecessaryToDetectGravity = 10;
double lastGravityVectorDetected[3] = {0,0,0}; //save the value of the last gravity vector detected with the algorithm values


int AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	//printf("Error handled. %d - %s", ErrorCode, unknown);  //Do not print error info
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
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

/**
 * @brief Publish the data on their corresponding topics
 */
int publish_data(){

    if(spatial_good)
    {
	corobot_msgs::spatial spatial;
	sensor_msgs::Imu imu;

	spatial.acc1 = acc[0];
	spatial.acc2 = acc[1];
	spatial.acc3 = acc[2];
	spatial.ang1 = ang[0];
	spatial.ang2 = ang[1];
	spatial.ang3 = ang[2];
	spatial.mag1 = mag[0];
	spatial.mag2 = mag[1];
	spatial.mag3 = mag[2];

	imu.header.frame_id = "base_link";
	imu.header.stamp = ros::Time::now();
	imu.orientation = tf::createQuaternionMsgFromRollPitchYaw((atan2(dCMMatrix[2][1],dCMMatrix[2][2]) + rollOffset),(-asin(dCMMatrix[2][0]) + pitchOffset), (atan2(dCMMatrix[1][0],dCMMatrix[0][0])));
	imu.angular_velocity.x = ang[0];
	imu.angular_velocity.y = ang[1];
	imu.angular_velocity.z = ang[2];
	imu.linear_acceleration.x = acc[0];
	imu.linear_acceleration.y = acc[1];
	imu.linear_acceleration.z = acc[2];

	spatial_pub.publish(spatial);
	imu_pub.publish(imu);

	spatialError = 0;
    }

    if(m_encodersGood) //the position is 4times the number of encoder counts.
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
    if (m_validAnalogs){
        ////////////////////////////
        // Update power data
	if(batteryPort != -1)
	{
        	corobot_msgs::PowerMsg powerdata;
        	powerdata.volts = (float) (analog_inputs_value[batteryPort] - 500) * 0.0734;
		//The Min and Max present here are for the Nimh battery as it is the only one type of battery sold with a Corobot at the moment
		powerdata.min_volt = 10.0;
		powerdata.max_volt = 14.2;
        	powerdata_pub.publish(powerdata);
	}

        ////////////////////////////
        // Update IR data
	if(irFrontPort != -1 || irBackPort != -1)
	{
        	corobot_msgs::IrMsg irData;
		if(irFrontPort != -1)
			irData.voltage1=(float) analog_inputs_value[irFrontPort]  / 200.0;
		else
			irData.voltage1=0;
		if(irBackPort != -1)
			irData.voltage2=(float) analog_inputs_value[irBackPort]  / 200.0;
		else
			irData.voltage2=0;
		irData.range1=irVoltageToDistance(irData.voltage1);
		irData.range2=irVoltageToDistance(irData.voltage2);
		irData_pub.publish(irData);
	}

	if(gripperPort != -1)
	{
		corobot_msgs::GripperMsg gripperData;  // Gripper connect to the 4th analog input port?
		gripperData.state=analog_inputs_value[gripperPort];
		gripper_pub.publish(gripperData);
	}
	interfaceKitError = 0;
    }

    if (m_validDigitals)
      {
        ////////////////////////////
        // Update the bumper data
        corobot_msgs::BumperMsg bumper_data;

        if (m_rearBumperPresent)
          {
            bumper_data.bumpers_count=4;
            bumper_data.value0=(digital_inputs_value) & (0x01<<0);
            bumper_data.value1=(digital_inputs_value) & (0x01<<1);
            bumper_data.value2=(digital_inputs_value) & (0x01<<2);
            bumper_data.value3=(digital_inputs_value) & (0x01<<3);
          } 
	else {
          bumper_data.bumpers_count=2;
          bumper_data.value0=(digital_inputs_value) & (0x01<<0);
          bumper_data.value1=(digital_inputs_value) & (0x01<<1);
        }

        bumper_pub.publish(bumper_data);
	interfaceKitError = 0;

      }

    return 0;
}

/** @brief Function that will manage the sonars and acquire their data
 * this function is called every around 50ms
 */
int sendSonarResult()
{
	corobot_msgs::RangeSensor data;
	data.type = data.ULTRASOUND;
	data.numberSensors = lastSonarInput + 1 - firstSonarInput;	

	CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 1);
	ros::Duration(0.002).sleep(); // sleep for 2ms
	CPhidgetInterfaceKit_setOutputState(ifKit, strobeOutput, 0);

	//Acquire the data and transform them into values in meters
	for(int i = firstSonarInput; i<= lastSonarInput;i++)
	{
		data.range.push_back(sonarVoltageToMeters(analog_inputs_value[i]));
	}
	 sonar_pub.publish(data);
	return 0;
}


/** @brief callback that will run if an input changes.
 * Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
 */
int InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	printf("Digital Input: %d > State: %d\n", Index, State);

 	if(State==PTRUE)
       	  digital_inputs_value = digital_inputs_value|(0x01<<Index);
    	else if(State == PFALSE)
     	  digital_inputs_value = digital_inputs_value&(~(0x01<<Index));
   	 m_validDigitals = true;
  
	return 0;
}

//callback that will run if an output changes.
//Index - Index of the output that generated the event, State - boolean (0 or 1) representing the output state (on or off)
int OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	printf("Digital Output: %d > State: %d\n", Index, State);
	return 0;
}

//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
	printf("Sensor: %d > Value: %d\n", Index, Value);

	//sensorValue 0-1000 ==> 0-5V

    	analog_inputs_value[Index]=Value;      //SEGMENTATION FAULT!!

   	m_validAnalogs = true;

	return 0;
}

void vectorScale(double vectorOut[3],double vectorIn[3], double scale2)
{
    for(int c=0; c<3; c++)
    {
        vectorOut[c]=vectorIn[c]*scale2; 
    }
}



void vectorCrossProduct(double vectorOut[3], double vectorIn1[3],double vectorIn2[3])
{
    vectorOut[0]= (vectorIn1[1]*vectorIn2[2]) - (vectorIn1[2]*vectorIn2[1]);
    vectorOut[1]= (vectorIn1[2]*vectorIn2[0]) - (vectorIn1[0]*vectorIn2[2]);
    vectorOut[2]= (vectorIn1[0]*vectorIn2[1]) - (vectorIn1[1]*vectorIn2[0]);
}

void vectorAddition(double vectorOut[3],double vectorIn1[3], double vectorIn2[3])
{
    for(int c=0; c<3; c++)
    {
        vectorOut[c]=vectorIn1[c]+vectorIn2[c];
    }
}

void matrixMultiply(double MatrixOut[3][3],double MatrixIn1[3][3], double MatrixIn2[3][3])
{
    double op[3]; 
    for(int x=0; x<3; x++)
    {
        for(int y=0; y<3; y++)
        {
            for(int w=0; w<3; w++)
            {
                op[w]=MatrixIn1[x][w]*MatrixIn2[w][y];
            } 
            MatrixOut[x][y]=0;
            MatrixOut[x][y]=op[0]+op[1]+op[2];
            
        }
    }
}

double ToRad(double value)
{
    return value*0.01745329252;  // *pi/180
}

double vectorDotProduct(double vector1[3],double vector2[3])
{
    double op=0;
    
    for(int c=0; c<3; c++)
    {
        op+=vector1[c]*vector2[c];
    }
    
    return op; 
}

double constrain(double value, double min, double max)
{
    if((value>=min) && (value<= max))
        return value;
    else if(value<min)
        return min;
    else //a>max
        return max;
}

bool isEqual(double vector1[3], double vector2[3])
{
    if(abs(vector1[0] - (vector2[0]/GRAVITY) + vector1[1] - (vector2[1]/GRAVITY) + vector1[2] - (vector2[2]/GRAVITY))>0.0001)
        return false;
    else
        return true;
}

// Update the dCM matrix to caculate the orientation
void AnglesMatrixUpdate(float gyroscopeVector[3], int gyroscopeTimestamp)
{

    
    double period;
        if(timestampPreviousCall==0) 
            period = 0;
        else
            period = (gyroscopeTimestamp - timestampPreviousCall); //time in seconds between the two last measures we got
    
    double temporaryVector[3]= {0,0,0};
    double updateMatrix[3][3]; //temporary matrix we use to update the dCMMatrix
    double temporaryMatrix[3][3];
    double gyroscopeVectorInvertedSystem[3]; //We call here Inverted System the Device coordinate system with Dx = Dy and Dy = Dx( the axis are exchanged)
    
    
    /****** We have to Exchange the x and y value of our vector to correspond with the system used by the algorithm *******/
    gyroscopeVectorInvertedSystem[0] =  gyroscopeVector[1];
    gyroscopeVectorInvertedSystem[1] =  gyroscopeVector[0];
    gyroscopeVectorInvertedSystem[2] =  gyroscopeVector[2];
    
    
    
    /******Calculate the GyroscopeVectorCorrected which is the gyroscope vector measure plus some vectors we get with the drift correction ( corresponding to the PID of the algorithm ********/
    vectorAddition(temporaryVector, gyroscopeVectorInvertedSystem, integratorVector);  //adding proportional term
    vectorAddition(gyroscopeVectorCorrected, temporaryVector, proportionalVector); //adding Integrator term
    
    
    /******We calculate the changement that the new measurement will imply*******/
    updateMatrix[0][0]=0;
    updateMatrix[0][1]=-period*gyroscopeVectorCorrected[2];
    updateMatrix[0][2]=period*gyroscopeVectorCorrected[1];
    updateMatrix[1][0]=period*gyroscopeVectorCorrected[2];
    updateMatrix[1][1]=0;
    updateMatrix[1][2]=-period*gyroscopeVectorCorrected[0];
    updateMatrix[2][0]=-period*gyroscopeVectorCorrected[1];
    updateMatrix[2][1]=period*gyroscopeVectorCorrected[0];
    updateMatrix[2][2]=0;
    
    
    
    /******We update the DCM Matrix *******/
    matrixMultiply(temporaryMatrix,dCMMatrix, updateMatrix); 
    
    for(int x=0; x<3; x++) //Matrix Addition (update)
    {
        for(int y=0; y<3; y++)
        {
            dCMMatrix[x][y]+=temporaryMatrix[x][y];
        } 
    }
    
    timestampPreviousCall = gyroscopeTimestamp;
    
}

//Normalize thed DCM matrix to caculate the orientation
void AnglesNormalize()
{
    double error=0;
    double temporaryMatrix[3][3];
    double vectorNorm=0;
    bool problem=false;
    
    /*******We want to make sure that the rows of our DCM Matrix are orthogonal. If not, we make them orthogonal.*******/
    
    error= -vectorDotProduct(dCMMatrix[0],dCMMatrix[1])*.5f; //eq.19
    
    vectorScale(temporaryMatrix[0], dCMMatrix[1], error); //eq.19
    vectorScale(temporaryMatrix[1], dCMMatrix[0], error); //eq.19
    
    vectorAddition(temporaryMatrix[0], temporaryMatrix[0], dCMMatrix[0]);//eq.19
    vectorAddition(temporaryMatrix[1], temporaryMatrix[1], dCMMatrix[1]);//eq.19
    
    vectorCrossProduct(temporaryMatrix[2],temporaryMatrix[0],temporaryMatrix[1]);  //eq.20
    
    
    /******We make sure that the norm of our vector is 1*******/
    
    vectorNorm= vectorDotProduct(temporaryMatrix[0],temporaryMatrix[0]); 
    if (vectorNorm < 1.5625f && vectorNorm > 0.64f) {
        vectorNorm= .5f * (3-vectorNorm);                                                 //eq.21
    } else if (vectorNorm < 100.0f && vectorNorm > 0.01f) {
        vectorNorm= 1. / sqrt(vectorNorm);
    } else {
        problem = true;
    }
    vectorScale(dCMMatrix[0], temporaryMatrix[0], vectorNorm);
    
    vectorNorm= vectorDotProduct(temporaryMatrix[1],temporaryMatrix[1]); 
    if (vectorNorm < 1.5625f && vectorNorm > 0.64f) {
        vectorNorm= .5f * (3-vectorNorm);                                                 //eq.21
    } else if (vectorNorm < 100.0f && vectorNorm > 0.01f) {
        vectorNorm= 1. / sqrt(vectorNorm);  
    } else {
        problem = true;
    }
    vectorScale(dCMMatrix[1], temporaryMatrix[1], vectorNorm);
    
    vectorNorm= vectorDotProduct(temporaryMatrix[2],temporaryMatrix[2]); 
    if (vectorNorm < 1.5625f && vectorNorm > 0.64f) {
        vectorNorm= .5f * (3-vectorNorm);                                                 //eq.21
    } else if (vectorNorm < 100.0f && vectorNorm > 0.01f) {
        vectorNorm= 1. / sqrt(vectorNorm);   
    } else {
        problem = true;  
    }
    vectorScale(dCMMatrix[2], temporaryMatrix[2], vectorNorm);
    
    
    /******If we can't renormalize ou matrix, then we reset it.*******/ 
    if (problem) {          // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
        dCMMatrix[0][0]= 1.0f;
        dCMMatrix[0][1]= 0.0f;
        dCMMatrix[0][2]= 0.0f;
        dCMMatrix[1][0]= 0.0f;
        dCMMatrix[1][1]= 1.0f;
        dCMMatrix[1][2]= 0.0f;
        dCMMatrix[2][0]= 0.0f;
        dCMMatrix[2][1]= 0.0f;
        dCMMatrix[2][2]= 1.0f;
        problem = false;  
    }
    
}

//Correct the drift or the gyroscope to get a more accurate result
void AnglesDriftCorrection(float AccelerationVector[3])
{
    
    double scaledIntegratorVector[3];
    double accelerationMagnitude;
    double accelerationWeight;
    double integratorMagnitude;
    double AccelerationVectorInvertedDystem[3];//We call here Inverted System the Device coordinate system with Dx = Dy and Dy = Dx( the axis are exchanged)
    double errorRollPitch[3];
    
    
    
    /******We calculate a vector  proportionalVector and integratorVector to add to the gyroscopeVector to cancel the drift. Those two vectors are calculated with the accelerometer vector. It doesn't cancel the drift for the yaw angle. *******/
    
    /****** Calculate the magnitude of the accelerometer vector***********/
    accelerationMagnitude = sqrt(AccelerationVector[0]*AccelerationVector[0] + AccelerationVector[1]*AccelerationVector[1] + AccelerationVector[2]*AccelerationVector[2]);
    accelerationMagnitude = accelerationMagnitude / GRAVITY; // We know have value of 1 = 1g
    
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0- , >1.5G = 0.0)
    accelerationWeight = constrain(1 - 2*abs(1 - accelerationMagnitude),0,1);   
    
    
    /****We make sure that the acceleration vector has the same system as the one we use in the algorithm *******/
    AccelerationVectorInvertedDystem[0] =  AccelerationVector[1];
    AccelerationVectorInvertedDystem[1] =  AccelerationVector[0];
    AccelerationVectorInvertedDystem[2] =  AccelerationVector[2];
    
    
    /*****We calculate the weights using the fact that 1g = 101********/
    vectorScale(AccelerationVectorInvertedDystem,AccelerationVectorInvertedDystem,101/9.81);
    
    /******We calculate our two vectors proportionalVector and integratorVector********/
    vectorCrossProduct(errorRollPitch,AccelerationVectorInvertedDystem,dCMMatrix[2]); //adjust the ground of reference
    vectorScale(proportionalVector,errorRollPitch,kpRollpitch*accelerationWeight);
    
    vectorScale(scaledIntegratorVector,errorRollPitch,kiRollpitch*accelerationWeight);
    vectorAddition(integratorVector,integratorVector,scaledIntegratorVector);     
    
    
    
    //  Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
    integratorMagnitude = sqrt(vectorDotProduct(integratorVector,integratorVector));
    if (integratorMagnitude > ToRad(300)) {
        vectorScale(integratorVector,integratorVector,0.5f*ToRad(300)/integratorMagnitude);
    }
    
    
}

void detectGravity(double gravity[3], float acceleration[3])
{
    double norm = sqrt(acceleration[0] * acceleration[0] + acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2]);
    
    /***** If the norm is near 1, then the vector is valid******/
    if(norm >= (GRAVITY*(1-gravityEpsilon)) && norm<= (GRAVITY*(1+gravityEpsilon)))
    {
        validGravityCounter++;
        
        /******After validAccelerationVectorsNecessaryToDetectGravity vectors, we say that the last valid vector is really the gravity and not the gravity + a certain acceleration*********/
        if(validGravityCounter == validAccelerationVectorsNecessaryToDetectGravity)
        {
            lastGravityVectorDetected[0] = acceleration[0];
            lastGravityVectorDetected[1] = acceleration[1];
            lastGravityVectorDetected[2] = acceleration[2];
            
            validGravityCounter = 0;
        }
    }
    else
        validGravityCounter =0;
    
    gravity[0] = lastGravityVectorDetected[0];
    gravity[1] = lastGravityVectorDetected[1];
    gravity[2] = lastGravityVectorDetected[2];
    
}


int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	int i;
	//printf("Number of Data Packets in this event: %d\n", count);
	/*for(i = 0; i < count; i++)
	{
		printf("=== Data Set: %d ===\n", i);
		printf("Acceleration> x: %6f  y: %6f  x: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
		printf("Angular Rate> x: %6f  y: %6f  x: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
		printf("Magnetic Field> x: %6f  y: %6f  x: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
		printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);
	}

	printf("---------------------------------------------\n");*/
	for(i = 0; i< count; i++)
	{
		acc[0] = data[i]->acceleration[0];
		acc[1] = data[i]->acceleration[1];
		acc[2] = data[i]->acceleration[2];
		ang[0] = data[i]->angularRate[0];
		ang[1] = data[i]->angularRate[1];
		ang[2] = data[i]->angularRate[2];
		mag[0] = data[i]->magneticField[0];
		mag[1] = data[i]->magneticField[1];
		mag[2] = data[i]->magneticField[2];

		if (ang[0] != 0 || ang[1] != 0 || ang[2] != 0)
			AnglesMatrixUpdate(ang, data[i]->timestamp.seconds + ((float)data[i]->timestamp.microseconds)/1000000);
		AnglesNormalize();
		AnglesDriftCorrection(acc);

		double gravity_[3];
		detectGravity(gravity_, acc);

		if(!isEqual(gravity, gravity_))
		{

		/*****We save the new gravity******/

		gravity[0] = gravity_[0]/GRAVITY;
		gravity[1] = gravity_[1]/GRAVITY;
		gravity[2] = gravity_[2]/GRAVITY;

		/****** This will give us the pitch and the yaw relative to the User axis and not the Device axis at t0
		 Calculations are made using the formulation of the rotation matrix*******/
		pitchOffset = gravity[1];
		if(pitchOffset>1.0f)
		    pitchOffset = 1.0f;
		if(pitchOffset<-1.0f)
		    pitchOffset = -1.0f;

		pitchOffset = asin(-pitchOffset);

		rollOffset = gravity[2]/cos(pitchOffset);
		if(rollOffset>1.0f)
		    rollOffset = 1.0f;
		if(rollOffset<-1.0f)
		    rollOffset = -1.0f;

		pitchOffset -= (-asin(dCMMatrix[2][0]));//We substract the actual pitch value from the algorithm
		rollOffset = acos(rollOffset) - (atan2(dCMMatrix[2][1],dCMMatrix[2][2]));//We substract the actual roll value from the algorithm
		}

	}

	ROS_INFO("publishing IMU data in ROS!");
	
	spatial_good = true;	

	return 0;
}




int interfacekit_simple()
{
	int result, num_analog_inputs, num_digital_inputs;
	const char *err;

	//create the InterfaceKit object
	CPhidgetInterfaceKit_create(&ifKit);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.

	CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
           
	CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, InputChangeHandler, NULL);

	CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);

	CPhidgetInterfaceKit_set_OnOutputChange_Handler (ifKit, OutputChangeHandler, NULL);

	//For phidget spatial
	//CPhidgetSpatialHandle spatial = 0;
	CPhidgetSpatial_create(&spatial);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);


	//open the interfacekit for device connections
	CPhidget_open((CPhidgetHandle)ifKit, -1);


	CPhidget_open((CPhidgetHandle)spatial, -1);


	CPhidgetInterfaceKit_getInputCount(ifKit, &num_digital_inputs);
	CPhidgetInterfaceKit_getSensorCount(ifKit, &num_analog_inputs);


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
		phidget888_connected = false;
		interfaceKitError = 1;
	}

	phidget888_connected = true;
	
	CPhidgetInterfaceKit_setRatiometric(ifKit, 0);//

	CPhidgetSpatial_setDataRate(spatial, 16);	

	 CPhidgetEncoder_create(&m_leftEncoder);
         CPhidget_set_OnAttach_Handler((CPhidgetHandle) m_leftEncoder,LeftEncoderAttach, NULL);
         CPhidget_open((CPhidgetHandle) m_leftEncoder, -1);
        
	if (m_encoder1Seen && m_encoder2Seen)
	    phidget_encoder_connected = true;
	else phidget_encoder_connected = false;

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
	nh.param("battery", batteryPort, 0);
	nh.param("irFront", irFrontPort, 1);
	nh.param("irBack", irBackPort, 2);
	nh.param("gripper", gripperPort, 3);
	nh.param("motors_inverted", motors_inverted, false);
	nh.param("encoders_inverted", encoders_inverted, false);

	dCMMatrix[0][0]=1;
	dCMMatrix[0][1]=0;
	dCMMatrix[0][2]=0;
	dCMMatrix[1][0]=0;
	dCMMatrix[1][1]=1;
	dCMMatrix[1][2]=0;
	dCMMatrix[2][0]=0;
	dCMMatrix[2][1]=0;
	dCMMatrix[2][2]=1;

	timestampPreviousCall = 0;
   	pitchOffset = 0;
    	rollOffset = 0;
	proportionalVector[0] = 0;
	proportionalVector[1] = 0;
	proportionalVector[2] = 0;

	integratorVector[0] = 0;
	integratorVector[1] = 0;
	integratorVector[2] = 0;
	gyroscopeVectorCorrected[0] = 0;
	gyroscopeVectorCorrected[1] = 0;
	gyroscopeVectorCorrected[2] = 0;

	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("Phidget");
	updater.add("Interface Kit", phidget_ik_diagnostic); //function that will be executed with updater.update()
	updater.add("Spatial", phidget_spatial_diagnostic); //function that will be executed with updater.update()
	updater.add("Encoders", phidget_encoder_diagnostic); //function that will be executed with updater.update()
	interfacekit_simple(); 

	posdata_pub = n.advertise<corobot_msgs::PosMsg>("position_data", 100);
        irData_pub = n.advertise<corobot_msgs::IrMsg>("infrared_data", 100);
        powerdata_pub = n.advertise<corobot_msgs::PowerMsg>("power_data", 100);
        bumper_pub = n.advertise<corobot_msgs::BumperMsg>("bumper_data", 100);
	gripper_pub = n.advertise<corobot_msgs::GripperMsg>("gripper_data",100);  //gripper as an analog input?
        sonar_pub = n.advertise<corobot_msgs::RangeSensor>("sonar_data", 100);
	spatial_pub = n.advertise<corobot_msgs::spatial>("spatial_data",100);
	imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",100);

	ros::Publisher phidget_info_pub = n.advertise<corobot_msgs::phidget_info>("phidget_info", 1000);

	corobot_msgs::phidget_info info;
	if(phidget888_connected)
	  info.phidget888_connected = 1;
	else info.phidget888_connected = 0;
	if(phidget_encoder_connected)
	  info.phidget_encoder_connected = 1;
	else info.phidget_encoder_connected = 0;
	phidget_info_pub.publish(info);

	ros::Rate loop_rate(20);  //20 Hz

	while (ros::ok())
            {
                ros::spinOnce();
		if(sonarsPresent)
			sendSonarResult();
		publish_data();
                loop_rate.sleep();
		updater.update();
            }
	printf("Out of ros spin");
	
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
