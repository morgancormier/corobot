#ifndef ROS_H
#define ROS_H

#include <QThread>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "corobot_msgs/IrMsg.h"
#include "corobot_msgs/PowerMsg.h"
#include "corobot_msgs/BumperMsg.h"
#include "corobot_msgs/GripperMsg.h"
#include "corobot_msgs/PanTilt.h"
#include "corobot_msgs/spatial.h"
#include "corobot_msgs/ssc32_info.h"
#include "corobot_msgs/phidget_info.h"
#include "corobot_msgs/takepic.h"
#include "corobot_msgs/MoveArm.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/LaserScan.h"
#include "corobot_msgs/velocityValue.h"

#include "corobot_msgs/MotorCommand.h"


//#include "body_msgs/Skeletons.h"

#include "sensor_msgs/NavSatFix.h"

#include <QGraphicsScene>

//#include "dynamic_uvc_cam/control.h"
//#include "dynamic_uvc_cam/setcontrol.h"
//#include "dynamic_uvc_cam/state.h"
//#include "dynamic_uvc_cam/videomode.h"

#include "Point.h"
#include "Image.h"
#include "ImageBig.h"
#include "Hokuyo.h"

const float BATTERY_EMPTY= 12* 7/10;

class Ros : public QThread {

    Q_OBJECT

private:
    //ros variables to call services and to get messages from a topic
    ros::ServiceClient setOdom_client;


    ros::Timer timer;

    ros::Subscriber velocity,ir,power,bumper,ptz,gripper,arm,cameraInfo,cameraImage,spatial,map_image;
    ros::Subscriber gps,scan,kinect_rgb,kinect_depth,kinect_skel;
    ros::Subscriber rear_cam,ptz_cam; // subsriber for rear_cam and PTZ_cam


    ros::Publisher driveControl_pub, velocityValue_pub, moveArm_pub;
    //*************************************************************************
    //Hardware component "ctf" subscriber
    ros::Subscriber ssc32_info_sub,phidget_info_sub;

    //*************************************************************************
    //Receive message from Joy stick
    ros::Subscriber takepic_sub;

    //Motion speed options
    //Forward speed Backward speed
    //Turning speed


    bool cameraRear_jpeg_compression, cameraFront_jpeg_compression; //True if the jpeg compression is activated
    bool arm_al5a, arm_pincher, arm_reactor, arm_old_ssc32, arm_old_phidget; //is there a lynxmotion al5a arm? a PhantomX pincher? a Phantom X reactor? Or maybe an old corobot_arm? 

    float speed_x,speed_a; //linear and angular speed
    int speed_value; //Maximum speed of the wheels. Between 0 and 100.
    bool initialized;
    float arm_px,arm_py; //arm position
    float speed_left, speed_right; //Speed of the left wheel and right wheel. Between -100 and 100
    bool turningLeft, turningRight; //Is the robot turning left, is the robot turning right

   // float last_x_pos,last_y_pos; for kinect hand gesture detection

    bool kinect_selected; //is kinect selected?


    QGraphicsScene * scenes_map_image;//map image scene
    QGraphicsScene * scenes_front_image;//front camera view in the easy view tab
    QGraphicsScene * scenes_rear_cam;//rear camera scene
    QGraphicsScene * scenes_ptz_cam;//ptz camera scene
    QGraphicsScene * scenes;//hd camera scene
    QGraphicsScene * scenes_kinect_rgb; //kinect rgb scene
    QGraphicsScene * scenes_kinect_depth; //kinect depth scene


    bool Corobot; //is Corobot

    Image image_kinect_depth;// allcamera scene : bottom right image
    Image image_kinect_rgb;// allcamera scene : bottom right image
    Image image_camera;// allcamera scene : bottom right image
    Image image_rear_cam;
    Image image_ptz_cam;
    Image image_front_cam;
    Image image_map_image;

       void timerCallback(const ros::TimerEvent& event);


signals :
       void posArmReal(double x, double y); //position of the robot arm
       void angleWristReal (double angle);//angle of the robot wrist
       void irData(double ir01, double ir02);
       void battery_percent(int value); //percentage of battery left
     //  void ptzData();
       void griperState(int state); //gripper state
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

       void spatial_data(double acc_x,double acc_y, double acc_z, double ang_x, double ang_y, double ang_z, double mag_x, double mag_y, double mag_z);

       void hokuyo_update(Hokuyo_Points* hokuyo_points);

   public slots:

       void turnWrist(float angle); //turn the robor wrist
       void moveGripper(bool state);//change gripper state
       void corobot(bool value);//is robot a corobot or explorer
       void select_kinect(bool value);//is kinect selected
    //   bool move_forward();
   //    bool move_backward();
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

public:
        void run();
        ~Ros();
        Ros();
        void init();
        void init(const std::string & master, const std::string & host);

        ros::Publisher pan_tilt_control;
        void subscribe(); //function that initialize every ros variables declared above.


        void velocityCallback(const nav_msgs::Odometry::ConstPtr& msg);//just receive position data from the robot
        void irCallback(const corobot_msgs::IrMsg::ConstPtr& msg); //just received ir data from the robot
        void powerCallback(const corobot_msgs::PowerMsgConstPtr& msg);//just received power data from the robot
        void bumperCallback(const corobot_msgs::BumperMsg::ConstPtr& msg);//just received bumper data from the robot
        void gripperCallback(const corobot_msgs::GripperMsg::ConstPtr& msg);//just received gripper data from the robot
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);//just reiceved camera information from the robot
        void cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg);//just received hd camera image from the robot
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);//just receive gps information from the robot
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);//just receive hokuyo information from the robot
        void kinectdepthCallback(const sensor_msgs::Image::ConstPtr& msg);//just received kinect depth image from the robot
        void kinectrgbCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);//just received kinect rgb image from the robot
        void spatialCallback(const corobot_msgs::spatial::ConstPtr& msg);

        void takepicCallback(const corobot_msgs::takepic::ConstPtr& msg);


        //*****************************************************************
        //Modified to compress image for the sake of wifi transmitting
        //void rear_camCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
        //void ptz_camCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
        void mapCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
        void rear_camCallback_compressed(const sensor_msgs::CompressedImage::ConstPtr& msg);
        void ptz_camCallback_compressed(const sensor_msgs::CompressedImage::ConstPtr& msg);
        void rear_camCallback(const sensor_msgs::Image::ConstPtr& msg);
        void ptz_camCallback(const sensor_msgs::Image::ConstPtr& msg);
       // void kinectskelCallback(const body_msgs::Skeletons::ConstPtr& msg);


        //Hardware info callback
        void ssc32infoCallback(const corobot_msgs::ssc32_info::ConstPtr& msg);
        void phidgetinfoCallback(const corobot_msgs::phidget_info::ConstPtr& msg);

       void add_map_image_scene(QGraphicsScene * scene);
       void add_front_image_scene(QGraphicsScene * scene);
       void add_camera_info_scene(QGraphicsScene * scene);
       void add_rear_cam_scene(QGraphicsScene * scene);
       void add_ptz_cam_scene(QGraphicsScene * scene);

       void add_kinect_rgb_scene(QGraphicsScene * scene);
       void add_kinect_depth_scene(QGraphicsScene * scene);
       void add_allcam_scene(QGraphicsScene * scene);
       bool setOdom(float x, float y);//change the encoders
       bool resetOdom();//reset the encoders
       void resetArm();//reset arm position

       void openGripper();//open the gripepr
       void closeGripper();//close the gripper
       bool setPtzcommand(int pan,int tilt);//change pan, tilt
       bool setPtzmode(int mode);
       bool setCameraControl(int id, int value);
       bool setCameraState(bool state);//true = start, falsedriveControl_client = stop
       bool setCameraMode(int width, int weight, bool immediately, int fps, bool auto_exposure);

       //void setRangeWidgetParent(QWidget* parent);
       //void setRangeWidget2Parent(QWidget* parent);

       int pan,tilt; //pan/tilt
       int forwardspeed[3];
       int backwardspeed[3];
       int forwardspeed_chosen;
       int turnleftspeed;
       int turnrightspeed;

       int left_motor_value;//SSC32 control value for two main DC motors, range from 500 - 2500
       int right_motor_value;
       int move_speed_level;
       int turning_speed_level;

       Hokuyo_Points* hokuyo_points_;

   };


#endif // ROS_H
