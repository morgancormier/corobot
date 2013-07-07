#ifndef ROS_H
#define ROS_H

#include <QThread>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "corobot_msgs/SensorMsg.h"
#include "corobot_msgs/PowerMsg.h"
#include "corobot_msgs/PanTilt.h"
#include "corobot_msgs/takepic.h"
#include "corobot_msgs/MoveArm.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"

#include "corobot_msgs/MotorCommand.h"


#include "sensor_msgs/NavSatFix.h"

#include <QGraphicsScene>


#include "Point.h"
#include "Image.h"
#include "Hokuyo.h"

const float BATTERY_EMPTY= 12* 7/10;

class Ros : public QThread {

    Q_OBJECT

private:
    //ros variables to call services and to get messages from a topic
    ros::ServiceClient setOdom_client;


    ros::Timer timer;

    ros::Subscriber velocity,ir,power,bumper,ptz,arm,cameraInfo,cameraImage,imu,magnetic,map_image;
    ros::Subscriber gps,scan,kinect_rgb,kinect_depth,kinect_skel;
    ros::Subscriber rear_cam,ptz_cam; // subsriber for rear_cam and PTZ_cam


    ros::Publisher driveControl_pub, velocityValue_pub, moveArm_pub;
    //*************************************************************************
    //Hardware component "ctf" subscriber
    ros::Subscriber ssc32_info_sub,phidget_info_sub;

    //*************************************************************************
    //Receive message from Joy stick
    ros::Subscriber takepic_sub;



    bool cameraRear_jpeg_compression, cameraFront_jpeg_compression; //True if the jpeg compression is activated
    bool arm_al5a, arm_pincher, arm_reactor, arm_old_ssc32, arm_old_phidget; //is there a lynxmotion al5a arm? a PhantomX pincher? a Phantom X reactor? Or maybe an old corobot_arm? 

    float speed_x,speed_a; //linear and angular speed
    int speed_value; //Maximum speed of the wheels. Between 0 and 100.
    bool initialized;
    float arm_px,arm_py; //arm position
    float speed_left, speed_right; //Speed of the left wheel and right wheel. Between -100 and 100
    bool turningLeft, turningRight; //Is the robot turning left, is the robot turning right
    double ir01, ir02; // range value for the small front and rear infrared sensors
    int bumper_data[4]; // save the information on if the bumper is hit or not


    bool kinect_selected; //is kinect selected?


    bool Corobot; //is Corobot

       void timerCallback(const ros::TimerEvent& event);


signals :
       void posArmReal(double x, double y); //position of the robot arm
       void angleWristReal (double angle);//angle of the robot wrist
       void irData(double ir01, double ir02);
       void battery_percent(int value); //percentage of battery left
     //  void ptzData();
       void gps_lat(double lat); //robot lattitude
       void gps_lon(double lon); //robot longitude
       void gps_coord(double lat, double lon); //gps coordinate
       void arm_model(bool arm_al5a,bool arm_pincher,bool arm_reactor,bool arm_old_corobot); //give the model of the arm connected to the robot

        void update_mapimage(QImage image);
	void update_rearcam(QImage image);
	void update_ptzcam(QImage image);
	void update_kinectDepthcam(QImage image);
	void update_kinectRGBcam(QImage image);
       void battery_volts(double volts);
       void bumper_update(int bumper1, int bumper2, int bumper3, int bumper4);
       void velocity_info(double linear, double angular);
       void imu_data(double acc_x,double acc_y, double acc_z, double ang_x, double ang_y, double ang_z);
       void magnetic_data(double mag_x, double mag_y, double mag_z);

       void hokuyo_update(Hokuyo_Points* hokuyo_points);
	void save_image(bool save); //order to save the front camera current displayed image

   public slots:
       void turnWrist(float angle); //turn the robor wrist
       void moveGripper(bool state);//change gripper state
       bool turn_left();
       bool turn_right();
       bool motor_stop();
       void moveShoulderArm(double shoulder);
       void moveElbowArm(double elbow);
       void rotateArm(double angle);
       void ResetArm();
       bool decrease_speed();
       bool increase_speed();
       bool increase_backward_speed();
       bool stop_turn();
	void setSpeedFast(bool toggled);
	void setSpeedModerate(bool toggled);
	void setSpeedSlow(bool toggled);
	void currentCameraTabChanged(int index); //subscribe to only the camera topics we are interested in
	void Pan_control(int value); // get the user interaction to control the pan camera and send it to the ros node
	void Tilt_control(int value); // get the user interaction to control the tilt camera and send it to the ros node

public:
        ~Ros();
        Ros();
	void run();
        void init(int argc, char *argv[]);
        void init(int argc, char *argv[], const std::string & master, const std::string & host);

        ros::Publisher pan_tilt_control;
        void subscribe(); //function that initialize every ros variables declared above.


        void velocityCallback(const nav_msgs::Odometry::ConstPtr& msg);//just receive position data from the robot
        void irCallback(const corobot_msgs::SensorMsg::ConstPtr& msg); //just received ir data from the robot
        void powerCallback(const corobot_msgs::PowerMsgConstPtr& msg);//just received power data from the robot
        void bumperCallback(const corobot_msgs::SensorMsg::ConstPtr& msg);//just received bumper data from the robot
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);//just receive gps information from the robot
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);//just receive hokuyo information from the robot
        void kinectdepthCallback(const sensor_msgs::Image::ConstPtr& msg);//just received kinect depth image from the robot
        void kinectrgbCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);//just received kinect rgb image from the robot
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
        void takepicCallback(const corobot_msgs::takepic::ConstPtr& msg);


        //*****************************************************************
        //Modified to compress image for the sake of wifi transmitting
        void mapCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
        void rear_camCallback_compressed(const sensor_msgs::CompressedImage::ConstPtr& msg);
        void ptz_camCallback_compressed(const sensor_msgs::CompressedImage::ConstPtr& msg);
        void rear_camCallback(const sensor_msgs::Image::ConstPtr& msg);
        void ptz_camCallback(const sensor_msgs::Image::ConstPtr& msg);

       bool setOdom(float x, float y);//change the encoders
       bool resetOdom();//reset the encoders
       void resetArm();//reset arm position

       void openGripper();//open the gripepr
       void closeGripper();//close the gripper
       int pan,tilt; //pan/tilt

       int move_speed_level;
       int turning_speed_level;

       Hokuyo_Points* hokuyo_points_;

   };


#endif // ROS_H
