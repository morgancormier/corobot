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

// Controls pan and tilt feature of Logitech Orbit camera.  Will probably work 
// for other USB pand/tilt cameras.
//
// Subscribes to /joy messages an input control of camera.  This should probably
// subscribe a to pan/tilt like message then have another node/service translate 
// joy messages to pan/tilt messages.  But this is a quick first pass.
//
// joy.axes[] values are expected to range between -1 and 1.

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <corobot_msgs/PanTilt.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <errno.h>

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

void pantiltCallback(const corobot_msgs::PanTiltConstPtr& msg)
{
  if (msg->reset) {
    pantilt_reset(cam_fd);
    ROS_INFO("PanTilt reset");
  } else {
    ROS_INFO("PanTilt moving pan=%d, tilt=%d", msg->pan, msg->tilt);
    pantilt_move(cam_fd,msg->pan, msg->tilt);
  }
}

/**
 * called everytime a joy message is received.
 */
void joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
  // msg.axes->length();
  const float joy_to_pan_gain = 10;
  const float joy_to_tilt_gain = 5;
  const int axis_pan = 0;
  const int axis_tilt = 1;
  const int button_reset = 4;
  const int button_right35 = 7;
  const int button_left35 = 5;
  int pan_inc_deg, tilt_inc_deg;
  int reset_button_previous = 0;

  // The length of the buttons & axes array is only as long as the largest button number pushed.
  // Check length.
  if (msg->buttons.size() > (unsigned int)button_reset && msg->buttons[button_reset] && !reset_button_previous) {
    reset_button_previous = 1;
    pantilt_reset(cam_fd);
    ROS_INFO("PanTilt reset");
  }
  else if (msg->buttons.size() > (unsigned int)button_right35 && msg->buttons[button_right35]) {
    pantilt_move(cam_fd,35,0);
    ROS_INFO("PanTilt moving pan=+35");
  }
  else if (msg->buttons.size() > (unsigned int)button_left35 && msg->buttons[button_left35]) {
    pantilt_move(cam_fd,-35,0);
    ROS_INFO("PanTilt moving pan=-35");
  }
  else {
    reset_button_previous = 0;
    if (msg->axes.size() > (unsigned int)axis_pan) {
      pan_inc_deg = int(msg->axes[axis_pan] * joy_to_pan_gain);
    } else {
      pan_inc_deg = 0;
    }
    if (msg->axes.size() > (unsigned int)axis_tilt) {
      tilt_inc_deg = int(msg->axes[axis_tilt] * joy_to_tilt_gain * -1);
    } else {
      tilt_inc_deg = 0;
    }
    if (pan_inc_deg >= 1 || tilt_inc_deg >= 1 || pan_inc_deg <= -1 || tilt_inc_deg <= -1) {
      pantilt_move(cam_fd,pan_inc_deg, tilt_inc_deg);
      ROS_INFO("PanTilt moving pan=%d, tilt=%d", pan_inc_deg, tilt_inc_deg);
    }
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

//Execute a script to initialize the camera 

  //system(path.c_str());
  if ((cam_fd = open(dev.c_str(), O_RDWR)) == -1) {
    ROS_INFO("ERROR: PanTilt could not open %s interface\n",dev.c_str());
    return -1;
  }
  ROS_INFO("PanTilt Opened: %s\n",dev.c_str());
  initDynCtrls(cam_fd);
  ROS_INFO("PanTilt Initilized: %s\n",dev.c_str());

  ROS_INFO("PanTilt note int(1.4)=%d, int(-1.4)=%d", int(1.4), int(-1.4));
  
  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joyCallback);
  ros::Subscriber pantilt_sub = n.subscribe("/pantilt", 1, pantiltCallback);
  ros::spin();

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
  sleep(2);
  return result;
}

int pantilt_move(int fd, int pan, int tilt){
  int result;
  result = uvcPanTilt(fd, pan*BITS_PER_DEGREE, tilt*BITS_PER_DEGREE, 0);
  //usleep((70+10*abs(pan))*1000);  // moving delay??
  return result;
}








