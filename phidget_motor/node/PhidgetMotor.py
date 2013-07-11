#!/usr/bin/env python
"""Phidgets HC Motor Control ROS service for CoroBot

A ROS service to request a specific acceleration and individual
side drive wheels speed.

"""

__author__ = 'Bill Mania <bmania@coroware.com>'
__version__ = '1'

import roslib; roslib.load_manifest('phidget_motor')
from corobot_msgs.msg import *
from std_msgs.msg import *
import rospy
from  threading import Timer
from ctypes import *
from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import EncoderPositionUpdateEventArgs
from Phidgets.Devices.Encoder import Encoder

motors_inverted = False

phidget1065 = False # True if we have a phidget1065, false if 1064
motorControl = 0
motorControlRight = 0 # we have two motor controllers in can we use 1065 devices
encoders = 0 # encoder board, in case we have a phidget 1064 motor controller board
leftEncoderPosition = 0; #in an encoder board, the index of the left motor encoder. Generally is 0 but in some robots this has been inverted by mistake
rightEncoderPosition = 1; #in an encoder board, the index of the right motor encoder. Generally is 1 but in some robots this has been inverted by mistake
leftWheels = 0 # index of the left wheel on the motor controller
rightWheels = 1 # index of the right wheel on the motor controller
minAcceleration = 0 # minimum acceleration that can be send to the phidget device
maxAcceleration = 0 # maximum acceleration that can be send to the phidget device
minSpeed = -100 # minimum speed that can be send to the phidget device
maxSpeed = 100 # maximum speed that can be send to the phidget device
timer = 0 # timer used to stop the motors after the number of seconds given in the message. This is interesting in case communication with a controlling computer is lost. 
posdataPub = 0 # publish encoder data if we have phidget 1065 device
leftEnconderPub = 0# publish left encoder data if we have phidget 1065 device
rightEnconderPub = 0# publish right encoder data if we have phidget 1065 device
leftPosition = 0 # value of the left encoder, if we have a 1065
rightPosition = 0 # value of the right encoder, if we have a 1065


def stop():
    # stop the motors
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
    # move the motors as requested
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
    # Make sure the acceleration and the speed is within the limits
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

    # on some motors the left and right motors have been inverted by mistake and the robot goes backward instead of forward. This is the solution for this problem
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

    # set the acceleration
    try:
        motorControl.setAcceleration(leftWheels, acceleration);
		
        if phidget1065 == True:
            motorControlRight.setAcceleration(rightWheels, acceleration);
        else:
            motorControl.setAcceleration(rightWheels, acceleration);
    except PhidgetException as e:
        rospy.logerr("Failed in setAcceleration() %i: %s", e.code, e.details)
        return(False)

    # set the velocity
    try:
        motorControl.setVelocity(leftWheels, leftSpeed);

        if phidget1065 == True:
            motorControlRight.setVelocity(rightWheels, rightSpeed);
        else:
            motorControl.setVelocity(rightWheels, rightSpeed);
    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        return(False)

    # start the timer to stop the motors after the requested duration
    if request.secondsDuration != 0:
        timer = Timer(request.secondsDuration, stop)
        timer.start()
    return(True)


def left_move(request):
    """move the left wheel as requested. This function doesn't use timer, and doesn't setup acceleration. 
       However it is used by the differential drive package to be able to command the robot with twist messages.
    """
    leftSpeed = float(request.data)

    if leftSpeed < minSpeed:
        leftSpeed = float(minSpeed)
    elif leftSpeed > maxSpeed:
        leftSpeed = float(maxSpeed)


    # set the velocity
    try:
        motorControl.setVelocity(leftWheels, leftSpeed);

    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        return(False)

    return(True)


def right_move(request):
    """move the right wheel as requested. This function doesn't use timer, and doesn't setup acceleration. 
       However it is used by the differential drive package to be able to command the robot with twist messages.
    """

    rightSpeed = float(request.data)

    if rightSpeed < minSpeed:
        rightSpeed = float(minSpeed)
    elif rightSpeed > maxSpeed:
        rightSpeed = float(maxSpeed)


    # set the velocity
    try:
        if phidget1065 == True:
            motorControlRight.setVelocity(rightWheels, rightSpeed);
        else:
            motorControl.setVelocity(rightWheels, rightSpeed);

    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        return(False)

    return(True)


def mcAttached(e):
    return

def mcError(e):
    return


def mcVelocityChanged(e):
    return


def leftEncoderUpdated(e):
    # left encoder callback, used with phidget 1065 devices
    global leftPosition, rightPosition, posdataPub, leftEnconderPub
	
    leftPosition -= e.positionChange
    if motorControlRight:
        rightPosition = motorControlRight.getEncoderPosition(rightWheels) # update the right encoder so that we have a correct value of both encoders at a given time.

    sendEncoderPosition()

    return

def rightEncoderUpdated(e):
    # right encoder callback, used with phidget 1065 devices
    global leftPosition, rightPosition, posdataPub, rightEnconderPub

    rightPosition += e.positionChange
    if motorControl:
        leftPosition = motorControl.getEncoderPosition(leftWheels) # update the left encoder so that we have a correct value of both encoders at a given time.

    sendEncoderPosition()
    return

def encoderBoardPositionChange(e):
    # encoder board callback, used only if a phidget 1064 is present
    global leftEncoderPosition, rightEncoderPosition, leftPosition, rightPosition

    if e.index == leftEncoderPosition:
        leftPosition = leftPosition - e.e.positionChange
    elif e.index == rightEncoderPosition:
        rightPosition = rightPosition + e.e.positionChange

    sendEncoderPosition()
    return


def sendEncoderPosition():
   # send message on the position topic

    global posdataPub, rightEncoderPub, leftEncoderPub, leftPosition, rightPosition
    msg = PosMsg()
    msg.px = leftPosition
    msg.py = rightPosition
    msg.header.stamp = rospy.Time.now()
    posdataPub.publish(msg)

    #publish the same data as an Int, more more conveniency
    msg = Int16Msg()
    msg.data = rightPosition
    rightEncoderPub.publish(msg)
    msg.data = leftPosition
    leftEncoderPub.publish(msg)

    return

def initMotorAndEncoderBoards():
    """ Open and Attach the phidget motor control board. Check if an 1064 or 1065 board is used and attach another 1065 board in case a 1065 is detected, or attach an encoder board in case a 1064 is detected
    """

    global motorControl, motorControlRight, rightWheels, phidget1065, encoders 
    

    try:
        motorControl = MotorControl()
    except:
        rospy.logerr("Unable to connect to Phidget HC Motor Control")
     	return

    try:
        motorControl.setOnAttachHandler(mcAttached)
        motorControl.setOnErrorhandler(mcError)
        motorControl.setOnVelocityChangeHandler(mcVelocityChanged)
        motorControl.openPhidget()

        #attach the board
        motorControl.waitForAttach(10000)
        """use the function getMotorCount() to know how many motors the Phidget board can take

        if the result is more than 1, we have a 1064 board and we take care of both motors with one motorControl variable. We need to handle the Phidget encoder board that is 
        separated from the phidget 1064.
        if the result is equal to 1, we have two phidget 1065 boards. The one with serial number that is the lowest will be the left will, the other the right weel. The encoder has to be handled 
        in this file as it is part of the 1065 board. 

        """
        if motorControl.getMotorCount() == 1:
            phidget1065 = True 
            rightWheels = 0
            motorControlRight.setOnAttachHandler(mcAttached)
            motorControlRight.setOnErrorhandler(mcError)
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

        else: # we have a motor controller board that control 2 motors but doesn't get any encoder input, so we need to initialize the encoder board.
            encoders = Encoder()
            encoders.setOnAttachHandler(encoderBoardAttached)
            encoders.setOnPositionChangeHandler(encoderBoardPositionChange)
            encoders.openPhidget()
            encoders.waitForAttach(10000)
            if(motors_inverted or encoders_inverted): # some robots have the left and right encoders switched by mistake
                leftEncoderPosition = 1;
                rightEncoderPosition = 0;
            encoders.setEnabled(leftEncoderPosition, True)
            encoders.setEnabled(rightEncoderPosition, True)
           

    except PhidgetException as e:
        rospy.logerr("Unable to initialize the motors and encoders board: %i: %s", e.code, e.details)
        return
    except:
        rospy.logerr("Unable to register the motors and encoders board")
        return

    if motorControl.isAttached():
        rospy.loginfo("Device: %s, Serial: %d, Version: %d",motorControl.getDeviceName(),motorControl.getSerialNum(),motorControl.getDeviceVersion())
    if phidget1065 == True:
        if motorControlRight.isAttached():
            rospy.loginfo("Device: %s, Serial: %d, Version: %d",motorControlRight.getDeviceName(),motorControlRight.getSerialNum(),motorControlRight.getDeviceVersion())
    else:
        rospy.loginfo("Device: %s, Serial: %d, Version: %d",encoders.getDeviceName(),encoders.getSerialNum(),encoders.getDeviceVersion())

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

    global motorControl, encoders, minAcceleration, maxAcceleration, timer, motors_inverted, phidget1065, leftWheels, posdataPub, leftEnconderPub, rightEnconderPub
    timer = 0
    
    initMotorAndEncoderBoards() # initialize the phidgets boards

    if motorControl != 0 and motorControl.isAttached():
        minAcceleration = motorControl.getAccelerationMin(leftWheels)
        maxAcceleration = motorControl.getAccelerationMax(leftWheels)

        motors_inverted = rospy.get_param('~motors_inverted', False)
        leftEncoderPosition = rospy.get_param('~encoders_inverted', False)

        phidgetMotorTopic = rospy.Subscriber("PhidgetMotor", MotorCommand ,move)
        leftWheelTopic = rospy.Subscriber("lmotor_cmd", Float32 ,left_move) # topic used for differential_drive package to enable twist commands
        rightWheelTopic = rospy.Subscriber("rmotor_cmd", Float32 ,right_move)# topic used for differential_drive package to enable twist commands

        posdataPub = rospy.Publisher("position_data", PosMsg)
        leftEnconderPub = rospy.Publisher("lwheel", Int16)
        rightEncoderPub = rospy.Publisher("rwheel", Int16)

        rospy.spin()

    return

if __name__ == "__main__":
    setupMoveService()

    try:
        motorControl.closePhidget()
        if phidget1065 == True:
            motorControlRight.closePhidget()
        else:
            encoders.closePhidget()
    except:
        pass
