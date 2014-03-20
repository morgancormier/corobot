#!/usr/bin/env python
"""
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
 """


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
from geometry_msgs.msg import Twist
from ctypes import *
from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import EncoderPositionUpdateEventArgs
from Phidgets.Devices.Encoder import Encoder
import numpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

motors_inverted = False # parameter. True if the motors have been inverted, meaning that index 1 is left and 0 is right
encoders_inverted = False # parameter. True if the encoders have been inverted, meaning that index 1 is left and 0 is right
stop_when_obstacle = False # use the motors current and encoders to detect if the robot bumped a wall and stop the motors

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
leftEncoderPub = 0# publish left encoder data if we have phidget 1065 device
rightEncoderPub = 0# publish right encoder data if we have phidget 1065 device
leftPosition = 0 # value of the left encoder, if we have a 1065
rightPosition = 0 # value of the right encoder, if we have a 1065
diagnosticPub = 0 #publish diagnostic messages
motorsError = 0 # help to find out which message we send on the diagnostics topic
encodersError = 0 # help to find out which message we send on the diagnostics topic
timer = 0 # timer for encoderCheck function. Used only if stop_when_obstacle is true.
pastCurrentLeft = 0.0 # used in the stop_when_obstacle procedure
pastCurrentRight = 0.0 # used in the stop_when_obstacle procedure
leftSpeed = 0 # used in the stop_when_obstacle procedure
rightSpeed = 0 # used in the stop_when_obstacle procedure

# diagnostics messages
motorControllerDisconnected = "Phidgets Motor controller disconnected - Please make sure it is connected, try to disconnect and reconnect it, or restart"
motorSpeedError = "Cannot Set Motor Speed - Make sure the Phidget Motor controller board has not been disconnected and the speed is within the range"
encoderBoardDisconnected = "Phidgets Encoder board disconnected - Please make sure it is connected, try to disconnect and reconnect it, or restart"
encoderValueError = "Cannot get encoder value - Please make sure the encoder board and encoders are connected"


# stop the motors
def stop():
    try:
        motorControl.setVelocity(leftWheels,0);

        if phidget1065 == True:
            motorControlRight.setVelocity(rightWheels,0);
        else:
        	motorControl.setVelocity(rightWheels, 0);
    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        motorsError = 2
        return(False)
    motorsError = 0
    return(True)


# move the motors as requested. Request is a corobot_msgs/MotorCommand.msg message 
# It containing an acceleration, speed for both the left and right wheel and a time value
def move(request):
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
        motorsError = 2
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
        motorsError = 2
        return(False)

    # start the timer to stop the motors after the requested duration
    if request.secondsDuration != 0:
        timer = Timer(request.secondsDuration, stop)
        timer.start()
    motorsError = 0
    return(True)


"""move the left wheel as requested. This function doesn't use timer, and doesn't setup acceleration. 
   However it is used by the differential drive package to be able to command the robot with twist messages.
"""
def left_move(request):
    leftSpeed = float(request.data)

    if leftSpeed < minSpeed:
        leftSpeed = float(minSpeed)
    elif leftSpeed > maxSpeed:
        leftSpeed = float(maxSpeed)


    # set the velocity
    try:
        motorControl.setAcceleration(leftWheels, 20);
        motorControl.setVelocity(leftWheels, leftSpeed);

    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        motorsError = 2
        return(False)

    motorsError = 0
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
            motorControlRight.setAcceleration(rightWheels, 20);
            motorControlRight.setVelocity(rightWheels, rightSpeed);
        else:
            motorControl.setAcceleration(rightWheels, 20);
            motorControl.setVelocity(rightWheels, rightSpeed);

    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        motorsError = 2
        return(False)

    motorsError = 0
    return(True)


def mcAttached(e):
    return

def mcError(e):
    return


def mcVelocityChanged(e):
    return


# left encoder callback, used with phidget 1065 devices
def leftEncoderUpdated(e):
    global leftPosition, rightPosition, posdataPub, leftEncoderPub
	
    leftPosition += e.positionChange
    try:
        if motorControlRight:
            # update the right encoder so that we have a correct value of both encoders at a given time.
            rightPosition = motorControlRight.getEncoderPosition(rightWheels) 
    except:
        encodersError = 2
        
    sendEncoderPosition()

    return


# right encoder callback, used with phidget 1065 devices
def rightEncoderUpdated(e):
    global leftPosition, rightPosition, posdataPub, rightEncoderPub

    rightPosition -= e.positionChange
    try:
        if motorControl:
            # update the left encoder so that we have a correct value of both encoders at a given time.
            leftPosition = motorControl.getEncoderPosition(leftWheels) 
    except:
        encodersError = 2
        
    sendEncoderPosition()
    return


# encoder board callback, used only if a phidget 1064 is present
def encoderBoardPositionChange(e):
    global leftEncoderPosition, rightEncoderPosition, leftPosition, rightPosition

    if e.index == leftEncoderPosition:
        leftPosition = leftPosition + e.positionChange
    elif e.index == rightEncoderPosition:
        rightPosition = rightPosition - e.positionChange

    sendEncoderPosition()
    return


# send message on the position topic
def sendEncoderPosition():
    global posdataPub, rightEncoderPub, leftEncoderPub, leftPosition, rightPosition
    msg = PosMsg()
    msg.px = leftPosition
    msg.py = rightPosition
    msg.header.stamp = rospy.Time.now()
    posdataPub.publish(msg)

    #publish the same data as an Int, for more conveniency
    msg = Int16()
    msg.data = numpy.int16(rightPosition)
    rightEncoderPub.publish(msg)
    msg.data = numpy.int16(leftPosition)
    leftEncoderPub.publish(msg)

    return
    
    
""" Check the encoders position and see if the wheels have moved or not.
    If the encoders didn't move but the motors have a speed set, that means that the robot bumped
    in which case we run the backAndRotate() function. 
"""    
def checkEncoders():

    global timer, phidget1065, pastCurrentLeft, pastCurrentRight, leftSpeed, rightSpeed, motorControl, motorControlRight, encoders, leftPosition, rightPosition

    timer.cancel()

    if (phidget1065 == True):
        newLeftPosition = motorControl.getEncoderPosition(leftWheels)
        newRightPosition = motorControlRight.getEncoderPosition(rightWheels)
    else:
        newLeftPosition = encoders.getPosition(leftWheels)
        newRightPosition = encoders.getPosition(rightWheels)        


    #Check if the encoder for left or right wheel didn't move a lot compared to the speed it should be at
    if (((newLeftPosition - leftPosition) < (leftSpeed*10) and (newLeftPosition - leftPosition) > -(leftSpeed*10) ) or ((newRightPosition - rightPosition) < (rightSpeed*10) and (newRightPosition - rightPosition) > -(rightSpeed*10))):
        # Bump detected, we stop the motors
        stop()

    # get the current going to each motors
    currentLeft = motorControl.getCurrent(leftWheels)
    if (phidget1065 == True):
        currentRight = motorControlRight.getCurrent(rightWheels)
    else:
        currentRight = motorControl.getCurrent(rightWheels)

    # We also check if there is a big increase (70% more) in the current in the motor, indicating an obstacle 
    if(pastCurrentRight != 0.0 and pastCurrentLeft != 0.0 and ( currentLeft >= pastCurrentLeft * 1.7 or currentRight >= pastCurrentRight * 1.7 ) and newLeftSpeed == leftSpeed and newRightSpeed == rightSpeed and rightSpeed != 0 and leftSpeed != 0):
        #bump detect, we stop the motors
        stop()

    pastCurrentLeft = currentLeft
    pastCurrentRight = currentRight

    leftPosition = newLeftPosition
    rightPosition = newRightPosition

    # get the velocity of the motors
    leftSpeed = motorControl.getVelocity(leftWheels)
    if (phidget1065 == True):
        rightSpeed = motorControlRight.getVelocity(rightWheels)
    else:
        rightSpeed = motorControl.getVelocity(rightWheels)

    # start the timer again
    timer = Timer(0.5, checkEncoders)
    timer.start()
    return      


""" Open and Attach the phidget motor control board. Check if an 1064 or 1065 board is used and attach another 1065 board in case a 1065 is detected, or attach an encoder board in case a 1064 is detected
"""
def initMotorAndEncoderBoards():

    global motorControl, motorControlRight, rightWheels, phidget1065, encoders, leftEncoderPosition, rightEncoderPosition, motors_inverted, encoders_inverted

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
            motorControlRight = MotorControl()
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
            motorControlRight.openPhidget()
            motorControlRight.waitForAttach(10000)

        # we have a motor controller board that control 2 motors but doesn't get any encoder input, so we need to initialize the encoder board.
        else: 
            encoders = Encoder()
            encoders.setOnPositionChangeHandler(encoderBoardPositionChange)
            encoders.openPhidget()
            encoders.waitForAttach(10000)
            # some robots have the left and right encoders switched by mistake
            if(motors_inverted or encoders_inverted): 
                leftEncoderPosition = 1;
                rightEncoderPosition = 0;
            encoders.setEnabled(leftEncoderPosition, True)
            encoders.setEnabled(rightEncoderPosition, True)
           

    except PhidgetException as e:
        motorsError = 1
        encodersError = 1
        rospy.logerr("Unable to initialize the motors and encoders board: %i: %s", e.code, e.details)
        return
    except:
        motorsError = 1
        encodersError = 1
        rospy.logerr("Unable to register the motors and encoders board")
        return

    if motorControl.isAttached():
        rospy.loginfo("Device: %s, Serial: %d, Version: %d",motorControl.getDeviceName(),motorControl.getSerialNum(),motorControl.getDeviceVersion())
    if phidget1065 == True:
        if motorControlRight.isAttached():
            rospy.loginfo("Device: %s, Serial: %d, Version: %d",motorControlRight.getDeviceName(),motorControlRight.getSerialNum(),motorControlRight.getDeviceVersion())
    else:
        rospy.loginfo("Device: %s, Serial: %d, Version: %d",encoders.getDeviceName(),encoders.getSerialNum(),encoders.getDeviceVersion())


    if stop_when_obstacle:
        timer = Timer(1.0, checkEncoders)
        timer.start()
    return


#Called every second and send diagnostic messages to tell the user if everything is ok or if something is wrong
def diagnosticsCallback (event):

    array = DiagnosticArray()

    # Set the timestamp for diagnostics
    array.header.stamp = rospy.Time.now()
    
    motors_message = DiagnosticStatus(name = 'PhidgetMotors', level = 0,message = 'initialized', hardware_id='Phidget')
    
    if (motorsError == 1):
        motors_message.level = 2
        motors_message.message = "Phidget Motor controller can't be initialized"
        motors_message.values = [ KeyValue(key = 'Recommendation', value = motorControllerDisconnected)]
    if (motorsError == 2):
        motors_message.level = 2
        motors_message.message = "Can't set up motor speed"
        motors_message.values = [ KeyValue(key = 'Recommendation', value = motorSpeedError)]
        
    encoders_message = DiagnosticStatus(name = 'PhidgetEncoders', level = 0,message = 'initialized', hardware_id='Phidget')
    
    if (encodersError == 1):
        encoders_message.level = 2
        encoders_message.message = "Phidget Encoder board can't be initialized"
        encoders_message.values = [ KeyValue(key = 'Recommendation', value = encoderBoardDisconnected)]
    if (encodersError == 2):
        encoders_message.level = 2
        encoders_message.message = "Can't get encoder value"
        encoders_message.values = [ KeyValue(key = 'Recommendation', value = encoderValueError)]
    
    array.status = [ motors_message, encoders_message ]
  
    diagnosticPub.publish(array)


"""Initialize the PhidgetMotor service

   Establish a connection with the Phidget HC Motor Control and
   then with the ROS Master as the service PhidgetMotor

"""
def setupMoveService():

    rospy.init_node(
            'PhidgetMotor',
            log_level = rospy.DEBUG
            )

    global motorControl, encoders, minAcceleration, maxAcceleration, timer, motors_inverted, phidget1065, leftWheels, posdataPub, leftEncoderPub, rightEncoderPub, stop_when_obstacle, diagnosticPub
    timer = 0
    
    motors_inverted = rospy.get_param('~motors_inverted', False)
    encoders_inverted = rospy.get_param('~encoders_inverted', False)
    stop_when_obstacle = rospy.get_param('~stop_when_obstacle', False)

    # initialize the phidgets boards
    initMotorAndEncoderBoards() 

    if motorControl != 0 and motorControl.isAttached():
        minAcceleration = motorControl.getAccelerationMin(leftWheels)
        maxAcceleration = motorControl.getAccelerationMax(leftWheels)


        phidgetMotorTopic = rospy.Subscriber("PhidgetMotor", MotorCommand ,move)

        # topic used for differential_drive package to enable twist commands
        leftWheelTopic = rospy.Subscriber("lmotor_cmd", Float32 ,left_move) 

        # topic used for differential_drive package to enable twist commands
        rightWheelTopic = rospy.Subscriber("rmotor_cmd", Float32 ,right_move)

        posdataPub = rospy.Publisher("position_data", PosMsg)
        leftEncoderPub = rospy.Publisher("lwheel", Int16)
        rightEncoderPub = rospy.Publisher("rwheel", Int16)
        diagnosticPub = rospy.Publisher('/diagnostics', DiagnosticArray)
        
        rospy.Timer(rospy.Duration(1.0), diagnosticsCallback, oneshot=False)

        rospy.spin()

    return



if __name__ == "__main__":
    setupMoveService()

    try:
        motorControl.setVelocity(leftWheels, 0)
        if phidget1065 != True:
            motorControl.setVelocity(rightWheels, 0)
        
        motorControl.closePhidget()
        if phidget1065 == True:
            motorControlRight.setVelocity(rightWheels, 0)
            motorControlRight.closePhidget()
        else:
            encoders.closePhidget()
    except:
        pass
