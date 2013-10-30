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

	double timestampPreviousCall; //timestamp of the measurement got at the last call of the function
	double pitchOffset; //offset necessary go from angle in the intial device position coordinate system to the User coordinate system
	double rollOffset; //offset necessary go from angle in the intial device position coordinate system to the User coordinate system
	double roll;
	double pitch;
	double yaw;
	double gravity[3]; //We save the gravity to know if the gravity we got is different from before or not
	double gravityEpsilon;
	int validGravityCounter; //Counter the number of valid gravity vector(valid in the sense that the norm is near 1) that follow each other
	int validAccelerationVectorsNecessaryToDetectGravity;
	double lastGravityVectorDetected[3]; //save the value of the last gravity vector detected with the algorithm values

	double ToRad(double value);
	bool isEqual(double vector1[3], double vector2[3]);
	void detectGravity(double gravity[3], float acceleration[3]);
	void calculate_new_gravity(float acceleration[3]);

public:

	Orientation();
	~Orientation(){};
	void updateAngles(float gyroscopeVector[3], float AccelerationVector[3], double gyroscopeTimestamp);
	double get_roll();
	double get_pitch();
	double get_yaw();
};
#endif
