/*
 * pantilt
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

// Author: Dallas Goecker
// Author: Morgan Cormier // Modifications of the original package

// Controls pan and tilt feature of Logitech Orbit camera.  Will probably work 
// for other USB pand/tilt cameras.
//
// joy.axes[] values are expected to range between -1 and 1.

#include <ros/ros.h>
#include <corobot_msgs/PanTilt.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

extern "C" {
  #include "dynctrl.h"
}

#define BITS_PER_DEGREE (64)
#define MAX_PAN_DEG  (70)
#define MIN_PAN_DEG  (-70)
#define MAX_TILT_DEG (30)
#define MIN_TILT_DEG (-30)
#define MIN_STEP_DEG (2)

int pantilt_reset(int fd);
int pantilt_move(int fd, int pan, int tilt);
void pantilt_check_speed(int fd);

static int cam_fd;
int pantiltError = 0; // use for diagnostics purpose

void pantiltCallback(const corobot_msgs::PanTiltConstPtr& msg)
/**
 * called everytime a corobot_msgs::Pantilt message is received.
 */
{
  if (msg->reset) {
    pantilt_reset(cam_fd);
    ROS_INFO("PanTilt reset");
  } else {
    ROS_INFO("PanTilt moving pan=%d, tilt=%d", msg->pan, msg->tilt);
    pantilt_move(cam_fd,msg->pan, msg->tilt);
  }
}

void pantilt_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
{
	if (!pantiltError)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "initialized");
	else if (pantiltError == 1)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Cannot be initialized");
		stat.addf("Recommendation", "Please make sure the port path is the correct one. Make sure the permissions are correct and that the camera has pan tilt option");
	}
	else if (pantiltError == 2)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Cannot move camera");
		stat.addf("Recommendation", "Please make sure the port path is the correct one. Make sure the permissions are correct and that the camera has pan tilt option");
	}
}

int main(int argc, char** argv)
{
  std::string dev;
  std::string path;
  ros::init(argc, argv, "pantilt");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  
 nh.param<std::string>("device",dev, "/dev/video0");
 nh.param<std::string>("script_path",path, "init_camera.sh");

  //create an updater that will send information on the diagnostics topics
  diagnostic_updater::Updater updater;
  updater.setHardwareIDf("PanTIlt");
  updater.add("PanTilt", pantilt_diagnostic); //function that will be executed with updater.update()

//Execute a script to initialize the camera 

  //system(path.c_str());
  if ((cam_fd = open(dev.c_str(), O_RDWR)) == -1) {
    ROS_ERROR("PanTilt could not open %s interface\n",dev.c_str());
    pantiltError = 1;
  }
  else
    pantiltError = 0;

  ROS_INFO("PanTilt Opened: %s\n",dev.c_str());

// Initialize the pan tilt camera
  initDynCtrls(cam_fd);
  ROS_INFO("PanTilt Initilized: %s\n",dev.c_str());
  ROS_INFO("PanTilt note int(1.4)=%d, int(-1.4)=%d", int(1.4), int(-1.4));

//adversize the topics
  ros::Subscriber pantilt_sub = n.subscribe("/pantilt", 1, pantiltCallback);

  while (ros::ok())
  {
	ros::spinOnce();
	updater.update();
  }

  close(cam_fd);
  return 0;
}


void pantilt_check_speed(int fd) {
  int i, repeat, step, pan;

  sleep(1);
  printf("reseting...");
  pantilt_reset(fd);
  printf("done.\n");
  sleep(1);

  for (step=2;step<=20;step=step+5) {
    printf("Step=%d: ",step);
    for (repeat=0;repeat<3;repeat++) {
      printf("%d,",repeat+1);
      pan = 0;
      for (i=step;i<60;i=i+step) {
        pantilt_move(fd,step,0);
        pan += step;
      }
      pantilt_move(fd,-pan,0);
      sleep(1);
    }
    printf("done.\n");
    pantilt_reset(fd);
    printf("reset.\n");
    sleep(1);
  }
}

int pantilt_reset(int fd) {
  int result;
  result = uvcPanTilt(fd, 0, 0, 3);

  if (result == -1)
	pantiltError = 2;
  else
	pantiltError = 0;

  sleep(2);
  return result;
}

int pantilt_move(int fd, int pan, int tilt){
  int result;
  result = uvcPanTilt(fd, pan*BITS_PER_DEGREE, tilt*BITS_PER_DEGREE, 0);

  if (result == -1)
	pantiltError = 2;
  else
	pantiltError = 0;
  return result;
}








