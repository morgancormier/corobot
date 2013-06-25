#ifndef ORIENTATION_H
#define ORIENTATION_H

#define GRAVITY 9.81

#include "orientation.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

class Orientation
{

private:

	double dCMMatrix[3][3]; //Matrix essential for calculating the orientation with IMU data
	int timestampPreviousCall; //timestamp of the measurement got at the last call of the function
	double pitchOffset; //offset necessary go from angle in the intial device position coordinate system to the User coordinate system
	double rollOffset; //offset necessary go from angle in the intial device position coordinate system to the User coordinate system
	double gravity[3]; //We save the gravity to know if the gravity we got is different from before or not
	double proportionalVector[3];//Proportional correction
	double integratorVector[3];//Omega Integrator
	double gyroscopeVectorCorrected[3]; //Corrected GyroVector data
	double kpRollpitch; //Used in a PID controller to calculate the roll and pitch angles
	double kiRollpitch; //Used in a PID controller to calculate the roll and pitch angles
	double gravityEpsilon;
	int validGravityCounter; //Counter the number of valid gravity vector(valid in the sense that the norm is near 1) that follow each other
	int validAccelerationVectorsNecessaryToDetectGravity;
	double lastGravityVectorDetected[3]; //save the value of the last gravity vector detected with the algorithm values

	void vectorScale(double vectorOut[3],double vectorIn[3], double scale2);
	void vectorCrossProduct(double vectorOut[3], double vectorIn1[3],double vectorIn2[3]);
	void matrixMultiply(double MatrixOut[3][3],double MatrixIn1[3][3], double MatrixIn2[3][3]);
	void vectorAddition(double vectorOut[3],double vectorIn1[3], double vectorIn2[3]);
	double ToRad(double value);
	double vectorDotProduct(double vector1[3],double vector2[3]);
	double constrain(double value, double min, double max);
	bool isEqual(double vector1[3], double vector2[3]);
	void AnglesNormalize();
	void AnglesDriftCorrection(float AccelerationVector[3]);
	void detectGravity(double gravity[3], float acceleration[3]);
	void AnglesMatrixUpdate(float gyroscopeVector[3], int gyroscopeTimestamp);
	void calculate_new_gravity(float acceleration[3]);

public:

	Orientation();
	~Orientation(){};
	void updateAngles(float gyroscopeVector[3], float AccelerationVector[3], int gyroscopeTimestamp);
	double get_roll();
	double get_pitch();
	double get_yaw();
};
#endif
