#!/usr/bin/env python
"""Phidgets HC Motor Control ROS service for CoroBot

A ROS service to request a specific acceleration and individual
side drive wheels speed.

"""

__author__ = 'Bill Mania <bmania@coroware.com>'
__version__ = '1'

import roslib; roslib.load_manifest('PhidgetMotor')
from corobot_msgs.msg import *
import rospy
from  threading import Timer
from ctypes import *
from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import EncoderPositionUpdateEventArgs

motors_inverted = False

phidget1065 = False
motorControl = 0
motorControlRight = 0
leftWheels = 0
rightWheels = 1
minAcceleration = 0
maxAcceleration = 0
minSpeed = -100
maxSpeed = 100
timer = 0
posdataPub = 0
leftPosition = 0
rightPosition = 0

def stop():
    try:
        motorControl.setVelocity(leftWheels,0);

        if phidget1065 == True:
            motorControlRight.setVelocity(rightWheels,0);
        else:
        	motorControl.setVelocity(rightWheels, 0);
    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        return(False)
    return(True)


def move(request):
    """Cause the CoroBot to move or stop moving

    Request a common acceleration, wheel directions and wheel speeds

    """
    global timer
    if timer:
        timer.cancel();
    rospy.logdebug(
            "Left: %d:%d, Right: %d:%d, Acceleration: %d", 
            leftWheels,
            request.leftSpeed,
            rightWheels,
            request.rightSpeed,
            request.acceleration
            )
    if request.acceleration > maxAcceleration:
        acceleration = float(maxAcceleration)
    elif request.acceleration < minAcceleration:
        acceleration = float(minAcceleration)
    else: 
        acceleration = float(request.acceleration)

    if request.leftSpeed < minSpeed:
        leftSpeed = float(minSpeed)
    elif request.leftSpeed > maxSpeed:
        leftSpeed = float(maxSpeed)
    else:
        leftSpeed = float(request.leftSpeed)
    if request.rightSpeed < minSpeed:
        rightSpeed = float(minSpeed)
    elif request.rightSpeed > maxSpeed:
        rightSpeed = float(maxSpeed)
    else:
        rightSpeed = float(request.rightSpeed)


    if motors_inverted == True:
	temp = rightSpeed
        rightSpeed = -leftSpeed
	leftSpeed = -temp

    rospy.logdebug(
            "Left: %d:%d, Right: %d:%d, Acceleration: %d", 
            leftWheels,
            leftSpeed,
            rightWheels,
            rightSpeed,
            acceleration
            )

    try:
        motorControl.setAcceleration(leftWheels, acceleration);
		
        if phidget1065 == True:
            motorControlRight.setAcceleration(rightWheels, acceleration);
        else:
            motorControl.setAcceleration(rightWheels, acceleration);
    except PhidgetException as e:
        rospy.logerr("Failed in setAcceleration() %i: %s", e.code, e.details)
        return(False)

    try:
        motorControl.setVelocity(leftWheels, leftSpeed);

        if phidget1065 == True:
            motorControlRight.setVelocity(rightWheels, rightSpeed);
        else:
            motorControl.setVelocity(rightWheels, rightSpeed);
    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        return(False)

    if request.secondsDuration != 0:
        timer = Timer(request.secondsDuration, stop)
        timer.start()
    return(True)

def mcAttached(e):
    return

def mcDetached(e):
    return

def mcError(e):
    return

def mcCurrentChanged(e):
    return

def mcInputChanged(e):
    return

def mcVelocityChanged(e):
    return

def leftEncoderUpdated(e):

    global leftPosition, rightPosition
	
    leftPosition += e.positionChange
    if motorControlRight:
        rightPosition = motorControlRight.getEncoderPosition(rightWheels) # update the right encoder so that we have a correct value of both encoders at a given time.

	# send message on the position topic
    msg = PosMsg()
    msg.px = leftPosition
    msg.py = rightPosition
    msg.header.stamp = rospy.Time.now()
    posdataPub.publish(msg)

    return

def rightEncoderUpdated(e):

    global leftPosition, rightPosition

    rightPosition += e.positionChange
    if motorControl:
        leftPosition = motorControl.getEncoderPosition(leftWheels) # update the left encoder so that we have a correct value of both encoders at a given time.

	# send message on the position topic
    msg = PosMsg()
    msg.px = leftPosition
    msg.py = rightPosition
    msg.header.stamp = rospy.Time.now()
    posdataPub.publish(msg)

    return

def setupMoveService():
    """Initialize the PhidgetMotor service

    Establish a connection with the Phidget HC Motor Control and
    then with the ROS Master as the service PhidgetMotor

    """

    rospy.init_node(
            'PhidgetMotor',
            log_level = rospy.DEBUG
            )

    global motorControl, motorControlRight, minAcceleration, maxAcceleration, timer, motors_inverted, phidget1065, rightWheels, posdataPub
    timer = 0
    try:
        motorControl = MotorControl()
    except:
        rospy.logerr("Unable to connect to Phidget HC Motor Control")
     	return

    try:
        motorControl.setOnAttachHandler(mcAttached)
        motorControl.setOnDetachHandler(mcDetached)
        motorControl.setOnErrorhandler(mcError)
        motorControl.setOnCurrentChangeHandler(mcCurrentChanged)
        motorControl.setOnInputChangeHandler(mcInputChanged)
        motorControl.setOnVelocityChangeHandler(mcVelocityChanged)
        motorControl.openPhidget()

        #attach the board
        motorControl.waitForAttach(10000)
        """use the function getMotorCount() to know how many motors the Phidget board can take

        if the result is more than 1, we have a 1064 board and we take care of both motors with one motorControl variable. The corobot_phidgetIK handles the Phidget encoder board that is 
        separated of the phidget 1064.
        if the result is equal to 1, we have two phidget 1065 boards. The one with serial number that is the lowest will be the left will, the other the right weel. The encoder has to be handled 
        in this file as it is part of the 1065 board. 

        """
        if motorControl.getMotorCount() == 1:
            phidget1065 = True 
            rightWheels = 0
            motorControlRight.setOnAttachHandler(mcAttached)
            motorControlRight.setOnDetachHandler(mcDetached)
            motorControlRight.setOnErrorhandler(mcError)
            motorControlRight.setOnCurrentChangeHandler(mcCurrentChanged)
            motorControlRight.setOnInputChangeHandler(mcInputChanged)
            motorControlRight.setOnVelocityChangeHandler(mcVelocityChanged)
			
            if motorControl.getSerialNum() > motorControlRight.getSerialNum(): 
                """ As a rule, we need the serial number of the left board to be lower than for the right board. This is for consistancy for all the robots
                """
                motorControlTemp = motorControl
                motorControl = motorControlRight
                motorControlRight = motorControlTemp

			#Set up the encoders handler
            motorControl.setOnPositionUpdateHandler(leftEncoderUpdated)
            motorControlRight.setOnPositionUpdateHandler(rightEncoderUpdated)

            #attach the board
            motorControlRight.waitForAttach(10000)
    except PhidgetException as e:
        rospy.logerr("Unable to register the handlers: %i: %s", e.code, e.details)
        return
    except:
        rospy.logerr("Unable to register the handlers")
        return
     

    if motorControl.isAttached():
        rospy.loginfo(
                "Device: %s, Serial: %d, Version: %d",
                motorControl.getDeviceName(),
                motorControl.getSerialNum(),
                motorControl.getDeviceVersion()
                )
    if phidget1065 == True:
        if motorControlRight.isAttached():
            rospy.loginfo(
		            "Device: %s, Serial: %d, Version: %d",
		            motorControlRight.getDeviceName(),
		            motorControlRight.getSerialNum(),
		            motorControlRight.getDeviceVersion()
		            )

    minAcceleration = motorControl.getAccelerationMin(leftWheels)
    maxAcceleration = motorControl.getAccelerationMax(leftWheels)

    motors_inverted = rospy.get_param('~motors_inverted', False)

    phidgetMotorTopic = rospy.Subscriber("PhidgetMotor", MotorCommand ,move)
    if phidget1065 == True:
        posdataPub = rospy.Publisher("position_data", PosMsg)
    rospy.spin()

if __name__ == "__main__":
    setupMoveService()

    try:
        motorControl.closePhidget()
        if phidget1065 == True:
            motorControlRight.closePhidget()
    except:
        pass
