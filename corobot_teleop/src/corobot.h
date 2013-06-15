/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2010 - CoroWare Technologies Inc
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef COROBOT_H
#define COROBOT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <limits.h>


/** constants sent on a topic to describe the gripper state **/
#define MSG_GRIPPER_STATE_OPEN   1
#define MSG_GRIPPER_STATE_CLOSED   2
#define MSG_GRIPPER_STATE_MOVING   3

/** constants for the ptz camera **/
#define TICKS_PER_DEGREE  64
#define TICKS_PER_RADIAN (180/M_PI)*TICKS_PER_DEGREE

/** diameter of the logitech orbit camera **/
#define ORBITAF_DIAMETER 0.08255
/** height of the logitech orbit camera **/
#define ORBITAF_HEIGHT   0.1143


/** the length in inches of the lower arm **/
#define LOWER_ARM_LENGTH_INCHES 6.25
/** the length in inches of the upper arm **/
#define UPPER_ARM_LENGTH_INCHES 8.00

/** Constant for converting inches to meters **/
#define INCHES_TO_METERS 0.0254

/** approximate diameter of the robot (assumed as circle) **/
#define COROBOT_DIAMETER 0.30

/** the pulse width considered to be STOP by the servo **/
#define ANALOGARMSERVO_CENTER	1500
/** the range of the pulse with acceptable, 
  centered at ANALOGARMSERVO_CENTER **/
#define ANALOGARMSERVO_RANGE	2000

/** the pulse width considered to be STOP by the gripper servo **/
#define GRIPPERSERVO_CENTER	1500

/** the range of the pulse with acceptable, 
  centered at GRIPPERSERVO_CENTER **/
#define GRIPPERSERVO_RANGE	1000

/** the pulse width considered to be STOP by the servo **/
#define DIGITALARMSERVO_CENTER	1500

/** the range of the pulse with acceptable, 
  centered at DIGITALARMSERVO_CENTER **/
#define DIGITALARMSERVO_RANGE	1500

/** the pulse width considered to be STOP by the base **/
#define DRIVEMOTOR_CENTER	1470

/** the range of the pulse with acceptable, centered at DRIVEMOTOR_CENTER **/
#define DRIVEMOTOR_RANGE	1000

/** your standard MIN macro **/
#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif

/** your standard MAX macro **/
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif

#ifdef __cplusplus
}
#endif

#endif
