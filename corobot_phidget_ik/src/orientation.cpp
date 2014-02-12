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

#include "orientation.h"
#include <stdio.h>

Orientation::Orientation()
{
	gravityEpsilon = 0.07;
	validGravityCounter = 0;
	validAccelerationVectorsNecessaryToDetectGravity = 10;
	lastGravityVectorDetected[0] = 0;
	lastGravityVectorDetected[1] = 0;
	lastGravityVectorDetected[2] = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
	timestampPreviousCall = 0;
   	pitchOffset = 0;
    	rollOffset = 0;
}

double Orientation::ToRad(double value)
//  degree to radian conversion
{
    // *pi/180
    return value*0.01745329252;  
}

bool Orientation::isEqual(double vector1[3], double vector2[3])
{
    if(abs(vector1[0] - (vector2[0]/GRAVITY) + vector1[1] - (vector2[1]/GRAVITY) + vector1[2] - (vector2[2]/GRAVITY))>0.0001)
        return false;
    else
        return true;
}

void Orientation::updateAngles(float gyroscopeVector[3], float AccelerationVector[3], double gyroscopeTimestamp)
{
  if (timestampPreviousCall != 0)
  {
    roll = gyroscopeVector[0] * (gyroscopeTimestamp - timestampPreviousCall);
    pitch = gyroscopeVector[1] * (gyroscopeTimestamp - timestampPreviousCall);
    yaw = gyroscopeVector[2] * (gyroscopeTimestamp - timestampPreviousCall);
  } 
	calculate_new_gravity(AccelerationVector);
	timestampPreviousCall = gyroscopeTimestamp;
}

// detect the acceleration measure is the gravity, and if yes save it
void Orientation::detectGravity(double gravity[3], float acceleration[3])
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
        // Check if the acceleration vector is the gravity
	detectGravity(gravity_, acceleration); 

// If it is, we update out gravity vector and calcule the new angle offset due to gravity. Indeed, the gravity measure give us an error on our orientation if not taken into account.
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

		//We substract the actual pitch value from the algorithm
		pitchOffset -= (pitch);
		//We substract the actual roll value from the algorithm
		rollOffset = acos(rollOffset) - (roll);
	}
}

double Orientation::get_roll()
{
	return ToRad(roll + rollOffset);
}

double Orientation::get_pitch()
{
	return ToRad(pitch + pitchOffset);
}

double Orientation::get_yaw()
{
	return ToRad(yaw);
}
