/*
 * Copyright (c) 2009, Morgan Quigley, Clemens Eppner, Tully Foote
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

// Modified 2013 by Morgan Cormier - change to fit the Corobot's needs
// Modified Apr 6, 2010 by Adam Leeper - changed to use "image_transport"
// Modified Aug 4, 2010 by Stefan Diewald - changed to fit Whiteboard robot's need

#include <cstdio>
#include <ros/ros.h>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <camera_calibration_parsers/parse_ini.h>
#include <dynamic_reconfigure/server.h>
#include <corobot_camera/corobot_cameraConfig.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "corobot_diagnostics/diagnostics.h"

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#if not (ROS_VERSION_MINIMUM(1, 9, 0)) //If the ROS version is bellow Groovy, as cv_bridge is not present in groovy anymore
	#include <cv_bridge/CvBridge.h>
#endif

// get the messages
#include "std_msgs/String.h"
#include "corobot_msgs/videomode.h"
#include "corobot_msgs/state.h"

sensor_msgs::CameraInfo::Ptr camera_info;

// the possible states
enum e_state {PLAYING, STOP, CHANGE_MODE};
char state = STOP;

// variables to change width and height
int new_width, new_height, new_fps;
bool auto_exposure = true;
bool camera_activated = true;
int camera_state = 0; // This is used for the diagnostic function to know what to report to the user
bool isjpeg = false;
// pointer to cam
uvc_cam::Cam * cam_ptr;

/*
 *	change the state of the video sender:
 *
 *		- start: stream images
 *		- stop:	 stop streaming
 */
void stateCallback(const corobot_msgs::state::ConstPtr& msg) {
	if (msg->state == std::string("start")) {
		state = PLAYING;
		ROS_INFO("Camera starting...");
	}
	else if (msg->state == std::string("stop")) {
		state = STOP;
		ROS_INFO("Camera stopping...");
	}
}

/*
 *	change the image resolution (width, height) and fps
 *
 *		As there is no error handling built-in at the moment please be sure that you only set available width, height and fps.
 */
void videomodeCallback(const corobot_msgs::videomode::ConstPtr& msg) {
	new_width = msg->width;
	new_height = msg->height;
	new_fps = msg->fps;
	auto_exposure = msg->auto_exposure;
	
	ROS_INFO("Receiced new video parameters: %dx%d, %d fps", new_width, new_height, new_fps);

	if ((bool)msg->immediately)
		state = CHANGE_MODE;
}


/*
 *	creates the uvc_cam object and gets the frames in a loop
 */
void mainloop(const char* device, int width, int height, int fps, ros::NodeHandle n, image_transport::CameraPublisher pub, diagnostic_updater::Updater& updater)
{
	ROS_INFO("Opening uvc_cam at %s with %dx%d, %d fps, auto-exposure: %s", device, width, height, fps, (auto_exposure)?"true":"false");
	ros::Rate r(fps);
	// instantiate uvc_cam
	uvc_cam::Cam::mode_t mode;
	if(isjpeg)
		mode = uvc_cam::Cam::MODE_MJPG;
	else
		mode = uvc_cam::Cam::MODE_RGB;

	ros::Time t_prev(ros::Time::now());
	uint64_t count = 0, skip_count = 0;

	int buf_idx = 0;
	unsigned char *frame = NULL;
	uint32_t bytes_used;
	 unsigned int pair_id = 0;

	bool camera_activated = false;
	uvc_cam::Cam *cam;
	
	sensor_msgs::ImagePtr image(new sensor_msgs::Image);
	image->height = height;
	image->width = width;
	image->step = 3 * width;
	image->encoding = sensor_msgs::image_encodings::RGB8;
	image->data.resize(image->step * image->height);

	// run as long as ROS is running and videomode should not be changed or camera should not be stopped
	while (n.ok() && state == PLAYING) {
		// only publish camera images if there is at least one subscriber
		if(pub.getNumSubscribers() > 0)
		{
			camera_state = 0;
			if (!camera_activated) // We activate the camera only when there is a subscriber
			{
				try
				{
					cam = new uvc_cam::Cam(device, mode, width, height, fps);
					cam_ptr = cam;
					camera_activated = true;
				}
				catch (std::runtime_error &ex)
				{
					camera_state = 4;
					camera_activated = false;
				}
			}

			if (camera_activated) // we can now grab a frame and publish it
			{
				// get one frame
				buf_idx = cam->grab(&frame,bytes_used);
				if (buf_idx == -1)
					camera_state = 3;
				// do fps counting
				if (count++ % fps == 0) {
					ros::Time t(ros::Time::now());
					ros::Duration d(t - t_prev);
					ROS_DEBUG("%.1f fps skip %lu", (double) fps / d.toSec(), skip_count);
					t_prev = t;
				}
	
				// if we could grab a frame
				if (frame) {

					ros::Time capture_time = ros::Time::now();

					image->header.stamp = capture_time;
 					image->header.seq = pair_id;
 
					std::string frameid = "camera";
					image->header.frame_id = frameid;

					memcpy(&image->data[0], frame, width*height * 3);

					// publish image & camera_info
					pub.publish(image, camera_info);
		
					// the camera sets user settings like exposure time around the 50th frame
					if (count == 50)
						ROS_DEBUG("User settings applied.");

					// release the frame
					cam->release(buf_idx);
				} else // there was no frame
					skip_count++;
				}
				
			}		
		else
		{
			camera_state = 2;
			if (camera_activated) // the camera is activated but no one is subscribed to the topic, so we deactivate the camera
			{
				delete cam;
				cam = NULL;
				cam_ptr = NULL;
				camera_activated = false;
			}
			
		}
		// for the callbacks
		updater.update(); // update the diagnostic status
		ros::spinOnce();
		r.sleep();
	}
}

void dynamic_reconfigureCallback(corobot_camera::corobot_cameraConfig &config, uint32_t level)
/**
 * Dynamic reconfigure callback
 */
{
		camera_activated = config.camera_activated;
}

void webcam_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
{
	if (camera_state == 0)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "The camera is working");
	else if (camera_state == 1)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "The user has stopped the camera. Activate it again to see the camera view");
	}
	else if (camera_state == 2)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "No subscriber to the camera topic - Start corobot_teleop or another image viewer");
	}
	else if (camera_state == 3)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not grab the image");
		stat.addf("Recommendation", CAMERA_DISCONNECTED);
	}
	else if (camera_state == 4)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not initialize the camera");
		stat.addf("Recommendation", ERROR_CAMERA_PARAMETERS);
	}

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "corobot_camera");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	bool immediately;

	dynamic_reconfigure::Server<corobot_camera::corobot_cameraConfig> server;
  	dynamic_reconfigure::Server<corobot_camera::corobot_cameraConfig>::CallbackType f;

	f = boost::bind(&dynamic_reconfigureCallback, _1, _2);
  	server.setCallback(f);

	// image transport with publisher
	image_transport::ImageTransport it(n);
	image_transport::CameraPublisher pub;


	ros::Subscriber videomode_sub = n.subscribe("/camera/set_videomode", 1000, videomodeCallback);
	ros::Subscriber state_sub = n.subscribe("/camera/set_state", 1000, stateCallback);

	// get the parameters
	std::string device;	
	std::string out_topic;
	n_private.param<std::string>("device", device, "/dev/video0");
	n_private.param<std::string>("topic", out_topic, "/camera/image_raw");
	n_private.param("width", new_width, 960);
	n_private.param("height", new_height, 720);
	n_private.param("fps", new_fps, 30); 
	n_private.param("isjpeg", isjpeg, false); 

	std::string cameara_parameter_file;
	n_private.param<std::string> ("camera_parameter_file", cameara_parameter_file, "../camera_parameters.txt");
	
	n_private.param("immediately", immediately, true);

	if (immediately)
		state = CHANGE_MODE;
	// camera_name is read out of the camera_parameter_file, not further used here
	std::string camera_name;
	// create a CameraInfo variable
	camera_info.reset(new sensor_msgs::CameraInfo);
	// read out the "ini" file
	if (camera_calibration_parsers::readCalibrationIni(cameara_parameter_file, camera_name, *camera_info)) {
		ROS_INFO("Successfully read camera calibration.");
	} else {
		ROS_ERROR("No camera_parameters.txt file found.");
	}
	// create an image publisher

	pub = it.advertiseCamera(out_topic.c_str(), 1);
	
	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("Webcam");
	updater.add("Webcam", webcam_diagnostic); //function that will be executed with updater.update()

	/*sleep to overcome the problem of mjpg-streamer (called for 1 seconds in corobot_pantilt node) and this node to change the camera format at the same time. */
	sleep(1);

	while (n.ok()) {
		// set it playing again after the parameter were changed
		if (state == CHANGE_MODE)
			state = PLAYING;

		// start the mainloop which publishes images
		if (state == PLAYING) {
			camera_state = 0;
			mainloop(device.c_str(), new_width, new_height, new_fps, n, pub, updater);
		}
		else
			camera_state = 1;
		updater.update();
		ros::spinOnce();
	}
	return 0;
}

