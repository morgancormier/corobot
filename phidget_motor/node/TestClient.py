#!/usr/bin/env python

import roslib; roslib.load_manifest('PhidgetMotor')

import sys
import rospy
from PhidgetMotor.srv import *

def phidgetMotorClient(
        leftSpeed,
        rightSpeed,
        secondsDuration,
        acceleration
        ):

    print "Waiting for service"
    rospy.wait_for_service('PhidgetMotor')

    try:
        print "Establishing connection to Service"
        phidgetMotor = rospy.ServiceProxy('PhidgetMotor', Move, persistent=True)
        response = phidgetMotor(
                leftSpeed,
                rightSpeed,
                secondsDuration,
                acceleration
                )
        return response.succeeded

    except rospy.ServiceException, e:
        print "Call to service PhidgetMotor failed: %s" % e

    return False

if __name__ == "__main__":
    if len(sys.argv) == 5:
        leftSpeed = int(sys.argv[1])
        rightSpeed = int(sys.argv[2])
        secondsDuration = int(sys.argv[3])
        acceleration = int(sys.argv[4])
    else:
        sys.exit(1)

    print "Requesting: Left %d, Right %d, Duration %d, Acceleration %d" % (leftSpeed, rightSpeed, secondsDuration, acceleration)

    print "Calling the Service"
    if phidgetMotorClient(
            leftSpeed,
            rightSpeed,
            secondsDuration,
            acceleration
            ):
        print "Success"
        sys.exit(0)
    else:
        print "Failure"
        sys.exit(2)

