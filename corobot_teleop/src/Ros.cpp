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

#include "Ros.h"
#include "corobot.h"
#include <QImage>
#include <QGraphicsTextItem>
#include <QPainter>
#include <QGraphicsView>
#include <math.h>
#include <QImageReader>

#include <vector>

using namespace std;

Ros::Ros(){

	speed_value = 100;
        speed_x = 0;
        speed_a = 0;

        arm_px = (double)UPPER_ARM_LENGTH_INCHES * (double)INCHES_TO_METERS;
        arm_py = (double)LOWER_ARM_LENGTH_INCHES * (double)INCHES_TO_METERS;

        pan = 0;
        tilt = 0;
        move_speed_level = 0;
        turning_speed_level = 0;

	turningRight = false;
	turningLeft = false;


        hokuyo_points_ = new Hokuyo_Points [683]; //hokuyo data
        speed_left = 0;
        speed_right = 0;

	initialized = false;

	ir01 = 0;
	ir02 = 0;
	bumper_data[0] = 0;
	bumper_data[1] = 0;
	bumper_data[2] = 0;
	bumper_data[3] = 0;
}



Ros::~Ros(){
    delete [] hokuyo_points_;
    hokuyo_points_ = NULL;
    if(driveControl_pub)
	this->motor_stop();
}

void Ros::subscribe()
//function that initialize every ros variables declared

{
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    nh.param("cameraRear_jpeg_compression", cameraRear_jpeg_compression, false); // true if the jpeg version of the rear camera should be used
    nh.param("cameraFront_jpeg_compression", cameraFront_jpeg_compression, false); // true if the jpeg version of the front camera should be used
    nh.param("Lynxmotion_al5a", arm_al5a, false); // true if there is an al5a arm
    nh.param("PhantomX_Pincher", arm_pincher, false); // true if there is a pincher arm
    nh.param("PhantomX_Reactor", arm_reactor, false); // true if there is a reactor arm
    nh.param("corobot_arm_ssc32", arm_old_ssc32, false); // true if there is an old ssc32 arm
    nh.param("corobot_arm_phidget", arm_old_phidget, false); // true if there is an old phidget arm

    emit arm_model(arm_al5a, arm_pincher, arm_reactor, arm_old_ssc32 || arm_old_phidget);

    //Advertise topics
    driveControl_pub = n.advertise<corobot_msgs::MotorCommand>("PhidgetMotor", 100);
    velocityValue_pub = n.advertise<std_msgs::Int32>("velocityValue", 100);
    moveArm_pub = n.advertise<corobot_msgs::MoveArm>("armPosition", 100);
    pan_tilt_control = n.advertise<corobot_msgs::PanTilt>("pantilt",5);

    //Subscribe to topics
    imu=n.subscribe("imu_data",1000, &Ros::imuCallback,this);
    magnetic=n.subscribe("magnetic_data",1000, &Ros::magCallback,this);
    velocity= n.subscribe("odometry", 1000, &Ros::velocityCallback,this);
    ir= n.subscribe("infrared_data", 1000, &Ros::irCallback,this);
    power= n.subscribe<corobot_msgs::PowerMsg>("power_data", 1000, &Ros::powerCallback,this);
    bumper= n.subscribe("bumper_data", 1000, &Ros::bumperCallback,this);
    gps= n.subscribe("fix", 1000, &Ros::gpsCallback,this);
    scan= n.subscribe("scan", 1000, &Ros::scanCallback,this);
    kinect_rgb = n.subscribe("/camera/rgb/image_color/compressed",100,&Ros::kinectrgbCallback,this);
    kinect_depth = n.subscribe("/camera/depth/image",100,&Ros::kinectdepthCallback,this);
    takepic_sub = n.subscribe("takepicture", 10, &Ros::takepicCallback, this);
    map_image = n.subscribe("/map_image/full_with_position/compressed",10, &Ros::mapCallback,this);

    //Use jpeg stream of raw images for the camera(s)
    if(cameraRear_jpeg_compression)
    	rear_cam = n.subscribe("REAR/image_raw/compressed",10, &Ros::rear_camCallback_compressed,this);
    else
	rear_cam = n.subscribe("REAR/image_raw",10, &Ros::rear_camCallback,this);
    if(cameraFront_jpeg_compression)
    	ptz_cam = n.subscribe("PTZ/image_raw/compressed",10, &Ros::ptz_camCallback_compressed,this);
    else
    	ptz_cam = n.subscribe("PTZ/image_raw",10, &Ros::ptz_camCallback,this);

    kinect_selected = true;
    pan = 0;
    tilt = 0;
    timer = nh.createTimer(ros::Duration(2), &Ros::timerCallback, this); // used to send again the motor command after 2 seconds, in order to stop the robot after a wireless deconnection
    initialized = true;
}

void Ros::init(int argc, char *argv[]){
    ros::init(argc, argv,"GUI");

    this->subscribe();
    this->start();
}

void Ros::run(){
    ros::spin();
}

//subscribe to only the camera topics we are interested in
void Ros::currentCameraTabChanged(int index)
{
    if (initialized == true)
    {
	    ros::NodeHandle n;
	    if(index == 0) // PTZ Camera
	    {
		if(cameraFront_jpeg_compression)
	    	    ptz_cam = n.subscribe("PTZ/image_raw/compressed",10, &Ros::ptz_camCallback_compressed,this);
		else
	    	    ptz_cam = n.subscribe("PTZ/image_raw",10, &Ros::ptz_camCallback,this);
		rear_cam.shutdown();
		kinect_rgb.shutdown();
		kinect_depth.shutdown();
	    }
	    else if(index == 1) // rear Camera
	    {
		if(cameraRear_jpeg_compression)
	    	    rear_cam = n.subscribe("REAR/image_raw/compressed",10, &Ros::rear_camCallback_compressed,this);
		else
		    rear_cam = n.subscribe("REAR/image_raw",10, &Ros::rear_camCallback,this);
		ptz_cam.shutdown();
		kinect_rgb.shutdown();
		kinect_depth.shutdown();
	    }
	    else if (index == 2)
	    {
		rear_cam.shutdown();
		ptz_cam.shutdown();
		kinect_rgb.shutdown();
		kinect_depth.shutdown();
	    }
	    else if(index == 3) // Kinect RGB
	    {
		kinect_rgb = n.subscribe("/camera/rgb/image_color/compressed",100,&Ros::kinectrgbCallback,this);
		rear_cam.shutdown();
		ptz_cam.shutdown();
		kinect_depth.shutdown();
	    }
	    else if(index == 4) // Kinect Depth
	    {
		kinect_depth = n.subscribe("/camera/depth/image",100,&Ros::kinectdepthCallback,this);
		rear_cam.shutdown();
		kinect_rgb.shutdown();
		ptz_cam.shutdown();
	    }
    }
}

// Initialize the ROS node with the ROS_MASTER_URI and ROS_IP given by the user in the interface
void Ros::init(int argc, char *argv[],const std::string & master,const std::string & host){
		std::map<std::string,std::string> remappings;
		remappings["__master"] = master;
		remappings["__ip"] = host;
		ros::init(remappings,"GUI");

		this->subscribe();
		this->start();
}

// Called when the user selects the fast speed movement, which is the fastest possible.
void Ros::setSpeedFast(bool toggled){
	if(toggled)
	{
		speed_value = 100;
		std_msgs::Int32 msg;
		msg.data = speed_value;
		velocityValue_pub.publish(msg);
	}
}

// Called when the user selects the medium speed movement
void Ros::setSpeedModerate(bool toggled){
	if(toggled)
	{
		speed_value = 75;
		std_msgs::Int32 msg;
		msg.data = speed_value;
		velocityValue_pub.publish(msg);
	}
}

// Called when the user selects the slow speed movement
void Ros::setSpeedSlow(bool toggled){
	if(toggled)
	{
		speed_value = 50;
		std_msgs::Int32 msg;
		msg.data = speed_value;
		velocityValue_pub.publish(msg);
	}
}

// Kinect depth image callback
void Ros::kinectdepthCallback(const sensor_msgs::Image::ConstPtr& msg){
    if(kinect_selected){
        unsigned char * copy;
        copy = (unsigned char*)malloc(msg->width*msg->height*3);
        float* tmp;

        for(int j=0;j<(msg->width*msg->height*3);j+=3){
            tmp = (float*)(&msg->data[0]+(j/3)*4);
            copy[j] = (char)((float)64*(float)(*tmp));
            copy[j+1] = (char)((float)64*(float)(*tmp));
            copy[j+2] = (char)((float)64*(float)(*tmp));
        }

    QImage *i = new QImage(copy,msg->width,msg->height,3*msg->width,QImage::Format_RGB888);
    QImage icopy = i->copy(0,0,msg->width,msg->height);
    if(i!=NULL){
                emit update_kinectDepthcam(icopy); //we can't allow a thread to use the scene for the zoom and another one to modify the image as it would crash, so we do everything in one thread
        }
    free(copy);
    delete i;

}
}

// Kinect rgb image callback
void Ros::kinectrgbCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    if(kinect_selected){
    	QImage *i = new QImage();
    	i->loadFromData(&msg->data[0],msg->data.size(),"JPEG");
    	QImage icopy = i->scaled(640,480);
    	if(i!=NULL){
                emit update_kinectRGBcam(icopy); //we can't allow a thread to use the scene for the zoom and another one to modify the image as it would crash, so we do everything in one thread
        }

    	delete i;
    }
}

// Odometry message callback
void Ros::velocityCallback(const nav_msgs::Odometry::ConstPtr& msg){
    double linear, angular;
    linear = sqrt(msg->twist.twist.linear.x *msg->twist.twist.linear.x + msg->twist.twist.linear.y * msg->twist.twist.linear.y);
    angular = msg->twist.twist.angular.z;

    emit velocity_info(linear,angular);
}

// Infrared sensors callback
void Ros::irCallback(const corobot_msgs::SensorMsg::ConstPtr& msg){

    if(msg->type == msg->INFRARED_FRONT)
	ir01 = (double)msg->value;
    else if(msg->type == msg->INFRARED_REAR)
	ir02 = (double)msg->value;

    emit irData(ir01,ir02);
}

// Battery voltage sensor callback
void Ros::powerCallback(const corobot_msgs::PowerMsgConstPtr& msg){
    int percent;
    if(msg->volts>= 12)
        percent = 100;
    else
       percent = ((float)(msg->volts - BATTERY_EMPTY)/(float)(12-BATTERY_EMPTY))*100;
    emit battery_percent(percent);
    emit battery_volts((double)msg->volts);
}

// bumper sensors callback
void Ros::bumperCallback(const corobot_msgs::SensorMsg::ConstPtr& msg){
     if (msg->type == msg->BUMPER)
     {
	bumper_data[msg->index] = msg->value;
	emit bumper_update(bumper_data[0], bumper_data[1], bumper_data[2], bumper_data[3]);
     }
     
}

// non-jpeg Image from the rear camera
void Ros::rear_camCallback(const sensor_msgs::Image::ConstPtr& msg){
    QImage *i = new QImage(&msg->data[0],msg->width,msg->height,msg->step,QImage::Format_RGB888);
    QImage icopy = i->copy(0,0,msg->width,msg->height);

    if(i!=NULL){
	emit update_rearcam(icopy); //we can't allow a thread to use the scene for the zoom and another one to modify the image as it would crash, so we do everything in one thread
        }
    delete i;
}

//jpeg Image from the rear camera
void Ros::rear_camCallback_compressed(const sensor_msgs::CompressedImage::ConstPtr& msg){
    QImage *i = new QImage();
    i->loadFromData(&msg->data[0],msg->data.size(),"JPEG");
    QImage icopy = i->copy(0,0,640,480);
    if(i!=NULL){
	emit update_rearcam(icopy); //we can't allow a thread to use the scene for the zoom and another one to modify the image as it would crash, so we do everything in one thread
        }
    delete i;
}

//non-jpeg Image from the front camera
void Ros::ptz_camCallback(const sensor_msgs::Image::ConstPtr& msg){
    QImage *i = new QImage(&msg->data[0],msg->width,msg->height,msg->step,QImage::Format_RGB888);
    QImage icopy = i->copy(0,0,msg->width,msg->height);

    if(i!=NULL){
	emit update_ptzcam(icopy); //we can't allow a thread to use the scene for the zoom and another one to modify the image as it would crash, so we do everything in one thread
    }

    delete i;
}

//jpeg Image from the front camera
void Ros::ptz_camCallback_compressed(const sensor_msgs::CompressedImage::ConstPtr& msg){
    QImage *i = new QImage();
    i->loadFromData(&msg->data[0],msg->data.size(),"JPEG");
    QImage icopy = i->copy(0,0,640,480);
    if(i!=NULL){
	emit update_ptzcam(icopy); //we can't allow a thread to use the scene for the zoom and another one to modify the image as it would crash, so we do everything in one thread
        }
    delete i;
}

// Jpeg image of the slam map
void Ros::mapCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    QImage *i = new QImage();
    i->loadFromData(&msg->data[0],msg->data.size(),"JPEG");
    QImage icopy = i->copy(0,0,i->height(),i->width());
    if(i!=NULL){   
                emit update_mapimage(icopy); //we can't allow a thread to use the scene for the zoom and another one to modify the image as it would crash, so we do everything in one thread
            }
    delete i;
}

// gps data callback
void Ros::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double lat = msg->latitude;
    double lon = msg->longitude;

        emit gps_lat(lat);
        emit gps_lon(lon);
        emit gps_coord(lat,lon);
}

// Laser Range Finder data callback
void Ros::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
      int index = 0;

      for(int it = 44; it<726; it++)
      {

          hokuyo_points_[it-44].distance = msg->ranges[it];
          hokuyo_points_[it-44].angle_step = msg->angle_increment;
          hokuyo_points_[it-44].index = it - 44;

          double radian = (index - 384 + 44) * msg->angle_increment;

          hokuyo_points_[it-44].x = -(hokuyo_points_[it-44].distance * sin (radian)); 
          hokuyo_points_[it-44].y = hokuyo_points_[it-44].distance * cos (radian); 

          index ++;

      }


      emit hokuyo_update(hokuyo_points_);

}

// Command from the joystick to save an image from the camera
void Ros::takepicCallback(const corobot_msgs::takepic::ConstPtr &msg)
{
    if(msg->take)
    {
        emit save_image(true);
    }
}

// Imu callback
void Ros::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    double acc_x, acc_y, acc_z, ang_x, ang_y, ang_z;

    acc_x = (double)msg->linear_acceleration.x;
    acc_y = (double)msg->linear_acceleration.y;
    acc_z = (double)msg->linear_acceleration.z;
    ang_x = (double)msg->angular_velocity.x;
    ang_y = (double)msg->angular_velocity.y;
    ang_z = (double)msg->angular_velocity.z;


    emit imu_data(acc_x,acc_y,acc_z,ang_x,ang_y,ang_z);
}

void Ros::magCallback(const sensor_msgs::MagneticField::ConstPtr &msg)
{
    double mag_x, mag_y, mag_z;

    mag_x = (double)msg->magnetic_field.x;
    mag_y = (double)msg->magnetic_field.y;
    mag_z = (double)msg->magnetic_field.z;

    emit magnetic_data(mag_x,mag_y,mag_z);
}

// Open of close the gripper. If state is true open the gripper, false close the gripper
void Ros::moveGripper(bool state){
    if(state)
        Ros::openGripper();
    else
        Ros::closeGripper();
}

// turn the wrist
void Ros::turnWrist(float angle){

	if(moveArm_pub)
	{
	    corobot_msgs::MoveArm msg;

	    /* Do some angle conversions. First we add PI/2 because the default position is actually at angle Pi/2. 
	       Then we make sure that the angle is not too high or too low
		Finally we convert to a value in degrees
	    */
	    double temp = -angle + M_PI/2;
	    if  (-angle > M_PI)
		temp =  - 2*M_PI - angle + M_PI/2;
	    if (temp < 0)
		temp = 2*M_PI + temp;
	    if(0 < temp || temp > M_PI)
		temp = (double)((int)(temp * 180/M_PI) % 180); // execute temp % M_PI, the conversion to int in degree is necessary as the modulus works with integers.


		//temporarily move both the flex and rotation see the GUI doesn't do the difference
	    msg.index = msg.WRIST_FLEX;
	    msg.position = temp;

	    moveArm_pub.publish(msg);

	    msg.index = msg.WRIST_ROTATION;
	    msg.position = temp;

	    moveArm_pub.publish(msg);
	}
}

// open the gripper
void Ros::openGripper(){

	if(moveArm_pub)
	{
		corobot_msgs::MoveArm msg;

		msg.index = msg.GRIPPER;
	    	msg.position = 0;

	    	moveArm_pub.publish(msg);
	}
}

// close the gripper
void Ros::closeGripper(){

	if(moveArm_pub)
	{
		corobot_msgs::MoveArm msg;

		msg.index = msg.GRIPPER;
	    	msg.position = 180;

	    	moveArm_pub.publish(msg);
	}
}

// timer callback, send the motor command again to avoid the motors from stopping. This method is used to stop the robot when wireless problem happen
void Ros::timerCallback(const ros::TimerEvent& event)
{
        corobot_msgs::MotorCommand msg;

        msg.leftSpeed = speed_left;
        msg.rightSpeed = speed_right;
        msg.secondsDuration = 3;
        msg.acceleration = 50;

        driveControl_pub.publish(msg);
 
}

//Decrease speed when no robot movement buttons or key is pressed
bool Ros::decrease_speed()
{
    if(driveControl_pub)
    {

        corobot_msgs::MotorCommand msg;

        if(turningLeft)
        {
            speed_left = -speed_value;
            speed_right = speed_value;
        }
        else if(turningRight)
        {
            speed_left = speed_value;
            speed_right = -speed_value;
        }
        else
        {
            speed_left = 0;
            speed_right = 0;
        }

        msg.leftSpeed = speed_left;
        msg.rightSpeed = speed_right;
        msg.secondsDuration = 3;
        msg.acceleration = 50;

        driveControl_pub.publish(msg);

    }
    return true;
}

//Increase speed when the move forward button or key is pressed
bool Ros::increase_speed()
{

    corobot_msgs::MotorCommand msg;
	if(driveControl_pub)
	{
			speed_left += speed_value;
			speed_right += speed_value;

			if(speed_left >speed_value)
				speed_left = speed_value;
			if(speed_right >speed_value)
				speed_right = speed_value;
			msg.leftSpeed = speed_left;
			msg.rightSpeed = speed_right;
			msg.secondsDuration = 3;
			msg.acceleration = 50;


		    driveControl_pub.publish(msg);
		return true;
	}
	else
		return false;
}

//Increase speed when the move backward button or key is pressed
bool Ros::increase_backward_speed()
{
    corobot_msgs::MotorCommand msg;
	if(driveControl_pub)
	{
		speed_left -= speed_value;
		speed_right -= speed_value;

		if(speed_left <-speed_value)
			speed_left = -speed_value;
		if(speed_right <-speed_value)
			speed_right = -speed_value;

		msg.leftSpeed = speed_left;
		msg.rightSpeed = speed_right;
		msg.secondsDuration = 3;
		msg.acceleration = 50;

		driveControl_pub.publish(msg);
		return true;
	}
	else
		return false;
}

//Turn the robot left
bool Ros::turn_left()
{
    corobot_msgs::MotorCommand msg;

	if(driveControl_pub)
	{
		if((speed_left+speed_right)<0)
		{
		    speed_left += speed_value;
		    speed_right -= speed_value;
		}
		else
		{
		    speed_left -= speed_value;
		    speed_right += speed_value;
		}

		if(speed_left >speed_value)
		    speed_left = speed_value;
		if(speed_right >speed_value)
		    speed_right = speed_value;
		if(speed_left <-speed_value)
		    speed_left = -speed_value;
		if(speed_right <-speed_value)
		    speed_right = -speed_value;


		msg.leftSpeed = speed_left;
		msg.rightSpeed = speed_right;
		msg.secondsDuration = 3;
		msg.acceleration = 50;

		driveControl_pub.publish(msg);
		turningLeft = true;
		turningRight = false;
		return true;
	}
	else
		return false;
}

// Turn the robot right
bool Ros::turn_right()
{
    corobot_msgs::MotorCommand msg;

	if(driveControl_pub)
	{
		if((speed_left+speed_right)<0)
		{
		    speed_left -= speed_value;
		    speed_right += speed_value;
		}
		else
		{
		    speed_left += speed_value;
		    speed_right -= speed_value;
		}
		if(speed_left >speed_value)
		    speed_left = speed_value;
		if(speed_right >speed_value)
		    speed_right = speed_value;
		if(speed_left <-speed_value)
		    speed_left = -speed_value;
		if(speed_right <-speed_value)
		    speed_right = -speed_value;

		msg.leftSpeed = speed_left;
		msg.rightSpeed = speed_right;
		msg.secondsDuration = 3;
		msg.acceleration = 50;

		driveControl_pub.publish(msg);
		turningRight = true;
		turningLeft = false;
		return true;
	}
	else
		return false;
}

// stop the robot from turning
bool Ros::stop_turn()
{
    corobot_msgs::MotorCommand msg;

	if(driveControl_pub)
	{	
		if((speed_left+speed_right)<0)
		{
			speed_left = -speed_value;
			speed_right = -speed_value;
		}
		else if((speed_left+speed_right)>0)
		{
			speed_left = +speed_value;
			speed_right = +speed_value;
		}
		else
		{
			speed_left = 0;
			speed_right = 0;
		}
	
		msg.leftSpeed = speed_left;
		msg.rightSpeed = speed_right;
		msg.secondsDuration = 3;
		msg.acceleration = 50;

		driveControl_pub.publish(msg);
		turningLeft = false;
		turningRight = false;
		return true;
	}
	else
		return false;
}

// stop the motors
bool Ros::motor_stop()
{
    corobot_msgs::MotorCommand msg;

	if(driveControl_pub)
	{
		msg.leftSpeed = 0;
		msg.rightSpeed = 0;
		msg.secondsDuration = 0;
		msg.acceleration = 50;
		driveControl_pub.publish(msg);
		return true;
	}
	else
		return false;
}

// move the shoulder of the arm
void Ros::moveShoulderArm(double shoulder)
{
	if(moveArm_pub)
	{
		corobot_msgs::MoveArm msg;

		msg.index = msg.SHOULDER;
	    	msg.position = shoulder *180/M_PI; //convertion from radian to degree

	    	moveArm_pub.publish(msg);
	}
}

// move the elbow arm
void Ros::moveElbowArm(double elbow)
{
	if(moveArm_pub)
	{
		corobot_msgs::MoveArm msg;

		msg.index = msg.ELBOW;
	    	msg.position = elbow *180/M_PI; //convertion from radian to degree

	    	moveArm_pub.publish(msg);
	}
}

// rotate the arm
void Ros::rotateArm(double angle)
{
	if(moveArm_pub)
	{
		corobot_msgs::MoveArm msg;

		msg.index = msg.BASE_ROTATION;
	    	msg.position = angle *181/M_PI; //convertion from radian to degree

	    	moveArm_pub.publish(msg);
	}
}

// reset the arm position
void Ros::ResetArm()
{
	if(moveArm_pub)
	{
		corobot_msgs::MoveArm msg;

		msg.index = msg.SHOULDER;
	    	msg.position = 0;
	    	moveArm_pub.publish(msg);

		msg.index = msg.ELBOW;
	    	msg.position = 0; 
	    	moveArm_pub.publish(msg);

		msg.index = msg.WRIST_FLEX;
	    	msg.position = 90; 
	    	moveArm_pub.publish(msg);

		msg.index = msg.WRIST_ROTATION;
	    	msg.position = 90; 
	    	moveArm_pub.publish(msg);

		msg.index = msg.GRIPPER;
	    	msg.position = 0; 
	    	moveArm_pub.publish(msg);

		msg.index = msg.BASE_ROTATION;
	    	msg.position = 90; 
	    	moveArm_pub.publish(msg);
	}
}

void Ros::Pan_control(int value) // get the user interaction to control the pan camera and send it to the ros node
{
    pan = value;
    corobot_msgs::PanTilt msg;
    msg.pan = pan;
    msg.tilt = tilt;
    msg.reset = 0;
    pan_tilt_control.publish(msg);
}

void Ros::Tilt_control(int value) // get the user interaction to control the tilt camera and send it to the ros node
{
    tilt = value;
    corobot_msgs::PanTilt msg;
    msg.pan = pan;
    msg.tilt = tilt;
    msg.reset = 0;
    pan_tilt_control.publish(msg);
}

