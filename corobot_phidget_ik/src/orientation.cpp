#include "orientation.h"
#include <stdio.h>

Orientation::Orientation()
{
	kpRollpitch = 0.015;
	kiRollpitch = 0.000010;
	gravityEpsilon = 0.07;
	validGravityCounter = 0;
	validAccelerationVectorsNecessaryToDetectGravity = 10;
	lastGravityVectorDetected[0] = 0;
	lastGravityVectorDetected[1] = 0;
	lastGravityVectorDetected[2] = 0;

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
}

void Orientation::vectorScale(double vectorOut[3],double vectorIn[3], double scale2)
// scaling vector calculation
{
    for(int c=0; c<3; c++)
    {
        vectorOut[c]=vectorIn[c]*scale2; 
    }
}



void Orientation::vectorCrossProduct(double vectorOut[3], double vectorIn1[3],double vectorIn2[3])
// cross product calculation
{
    vectorOut[0]= (vectorIn1[1]*vectorIn2[2]) - (vectorIn1[2]*vectorIn2[1]);
    vectorOut[1]= (vectorIn1[2]*vectorIn2[0]) - (vectorIn1[0]*vectorIn2[2]);
    vectorOut[2]= (vectorIn1[0]*vectorIn2[1]) - (vectorIn1[1]*vectorIn2[0]);
}

void Orientation::vectorAddition(double vectorOut[3],double vectorIn1[3], double vectorIn2[3])
// vector addition calculation
{
    for(int c=0; c<3; c++)
    {
        vectorOut[c]=vectorIn1[c]+vectorIn2[c];
    }
}

void Orientation::matrixMultiply(double MatrixOut[3][3],double MatrixIn1[3][3], double MatrixIn2[3][3])
//3x3 matrix multiplication
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

double Orientation::ToRad(double value)
//  degree to radian conversion
{
    return value*0.01745329252;  // *pi/180
}

double Orientation::vectorDotProduct(double vector1[3],double vector2[3])
// dot product calculation between two vectors
{
    double op=0;
    
    for(int c=0; c<3; c++)
    {
        op+=vector1[c]*vector2[c];
    }
    
    return op; 
}

double Orientation::constrain(double value, double min, double max)
{
    if((value>=min) && (value<= max))
        return value;
    else if(value<min)
        return min;
    else //a>max
        return max;
}

bool Orientation::isEqual(double vector1[3], double vector2[3])
{
    if(abs(vector1[0] - (vector2[0]/GRAVITY) + vector1[1] - (vector2[1]/GRAVITY) + vector1[2] - (vector2[2]/GRAVITY))>0.0001)
        return false;
    else
        return true;
}

void Orientation::updateAngles(float gyroscopeVector[3], float AccelerationVector[3], int gyroscopeTimestamp)
{

	AnglesMatrixUpdate(gyroscopeVector, gyroscopeTimestamp);
	AnglesNormalize();
    	AnglesDriftCorrection(AccelerationVector);
	calculate_new_gravity(AccelerationVector);
}

// Update the dCM matrix to caculate the orientation
void Orientation::AnglesMatrixUpdate(float gyroscopeVector[3], int gyroscopeTimestamp)
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
void Orientation::AnglesNormalize()
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
void Orientation::AnglesDriftCorrection(float AccelerationVector[3])
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

void Orientation::detectGravity(double gravity[3], float acceleration[3])
// detect the acceleration measure is the gravity, and if yes save it
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

void Orientation::calculate_new_gravity(float acceleration[3])
{
	double gravity_[3];
	detectGravity(gravity_, acceleration); // Check if the acceleration vector is the gravity
	if(!isEqual(gravity, gravity_)) // If it is, we update out gravity vector and calcule the new angle offset due to gravity. Indeed, the gravity measure give us an error on our orientation if not taken into account.
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

double Orientation::get_roll()
{
	return (atan2(dCMMatrix[2][1],dCMMatrix[2][2]) + rollOffset);
}

double Orientation::get_pitch()
{
	return (-asin(dCMMatrix[2][0]) + pitchOffset);
}

double Orientation::get_yaw()
{
	return (atan2(dCMMatrix[1][0],dCMMatrix[0][0]));
}
