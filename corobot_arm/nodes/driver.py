#!/usr/bin/env python

"""
  ArbotiX Node: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest('arbotix_python')
import rospy

from arbotix_msgs.msg import *
from arbotix_msgs.srv import *

from arbotix_python.arbotix import ArbotiX
from arbotix_python.diff_controller import DiffController
from arbotix_python.follow_controller import FollowController
from arbotix_python.publishers import *
from arbotix_python.servos import *
from arbotix_python.io import *

###############################################################################
# Main ROS interface
class ArbotixROS(ArbotiX):
    
    def __init__(self):
        pause = False

        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = int(rospy.get_param("~baud", "115200"))

        self.rate = rospy.get_param("~rate", 100.0)
        self.fake = rospy.get_param("~sim", False)

        self.use_sync_read = rospy.get_param("~sync_read",True)      # use sync read?
        self.use_sync_write = rospy.get_param("~sync_write",True)    # use sync write?

        # setup publishers
        self.diagnostics = DiagnosticsPublisher()
        self.joint_state_publisher = JointStatePublisher()

        # start an arbotix driver
        if not self.fake:
            ArbotiX.__init__(self, port, baud)        
            rospy.sleep(1.0)
            rospy.loginfo("Started ArbotiX connection on port " + port + ".")
        else:
            rospy.loginfo("ArbotiX being simulated.")

        # initialize dynamixel & hobby servos
        self.servos = Servos(self)

        # setup controllers
        self.controllers = list()
        controllers = rospy.get_param("~controllers", dict())
        for name, params in controllers.items():
            if params["type"] == "follow_controller":
                self.controllers.append(FollowController(self, name))
                if self.controllers[-1].onboard:
                    pause = True
            elif params["type"] == "diff_controller":
                self.controllers.append(DiffController(self, name))
                pause = True
#           elif params["type"] == "omni_controller":
#               self.controllers.append(OmniController(self, name))
#               pause = True
            else:
                rospy.logerr("Unrecognized controller: " + params["type"])

        # wait for arbotix to start up (especially after reset)
        if not self.fake:
            if rospy.has_param("~digital_servos") or rospy.has_param("~digital_sensors") or rospy.has_param("~analog_sensors"):
                pause = True
            if pause:
                while self.getDigital(1) == -1:
                    rospy.loginfo("Waiting for response...")
                    rospy.sleep(0.25)
            rospy.loginfo("ArbotiX connected.")

        for controller in self.controllers:
            controller.startup()

        # services for io
        rospy.Service('~SetupAnalogIn',SetupChannel, self.analogInCb)
        rospy.Service('~SetupDigitalIn',SetupChannel, self.digitalInCb)
        rospy.Service('~SetupDigitalOut',SetupChannel, self.digitalOutCb)
        # initialize digital/analog IO streams
        self.io = dict()
        if not self.fake:
            for v,t in {"digital_servos":DigitalServo,"digital_sensors":DigitalSensor,"analog_sensors":AnalogSensor}.items():
                temp = rospy.get_param("~"+v,dict())
                for name in temp.keys():
                    pin = rospy.get_param('~'+v+'/'+name+'/pin',1)
                    value = rospy.get_param('~'+v+'/'+name+'/value',0)
                    rate = rospy.get_param('~'+v+'/'+name+'/rate',10)
                    self.io[name] = t(name, pin, value, rate, self)
        
        r = rospy.Rate(self.rate)

        # main loop -- do all the read/write here
        while not rospy.is_shutdown():
    
            # update controllers
            for controller in self.controllers:
                controller.update()

            # update servo positions (via sync_write)
            self.servos.update(self.use_sync_write)

            # update io
            for io in self.io.values():
                io.update()

            # publish
            self.servos.interpolate(self.use_sync_read)
            self.joint_state_publisher.update(self.servos, self.controllers)
            self.diagnostics.update(self.servos, self.controllers)

            r.sleep()

        # do shutdown
        for controller in self.controllers:
            controller.shutdown()


    def analogInCb(self, req):
        # TODO: Add check, only 1 service per pin
        if not self.fake:
            self.io[req.topic_name] = AnalogSensor(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()

    def digitalInCb(self, req):
        if not self.fake:
            self.io[req.topic_name] = DigitalSensor(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()

    def digitalOutCb(self, req):
        if not self.fake:
            self.io[req.topic_name] = DigitalServo(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()


if __name__ == "__main__":
    rospy.init_node('arbotix')
    a = ArbotixROS()

