#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "ArmWidget.h"
#include "ArmRotationWidget.h"
#include "ArmCircleWidget.h"
#include "GripperWidget.h"
#include "WristWidget.h"
#include "GPS.h"
#include "joystick.h"
#include "Ros.h"
#include "sensor_msgs/Image.h"
#include "corobot_teleop/PanTilt.h"
#include "Hokuyo.h"
#include "MyQGraphicsView.h"



//***************************************************
//Speed control values, need to be determined on tests
#define FORWARD_SPEED_a 800
#define FORWARD_SPEED_b 700
#define FORWARD_SPEED_c 600
#define FORWARD_SPEED_d 500

#define BACKWARD_SPEED_a 1400
#define BACKWARD_SPEED_b 1700
#define BACKWARD_SPEED_c 2000
#define BACKWARD_SPEED_d 2300

#define TURN_LEFT_SPEED_a_l 1100
#define TURN_LEFT_SPEED_a_r 700

#define TURN_LEFT_SPEED_b_l 1100
#define TURN_LEFT_SPEED_b_r 500

#define TURN_RIGHT_SPEED_a_l 700
#define TURN_RIGHT_SPEED_a_r 1100

#define TURN_RIGHT_SPEED_b_l 500
#define TURN_RIGHT_SPEED_b_r 1100
//**********************************************


namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void resizeEvent(QResizeEvent *); //execute this function when the window size changes

//#ifdef Q_WS_WIN
//    bool winEvent(MSG*,long int *); // event windows used to receive joystick messages, only compiled for the windows version
//#endif

protected:
    void keyPressEvent(QKeyEvent *event);//executed each time a keyboard key is pushed
    void keyReleaseEvent(QKeyEvent *event);//executed each time a keyboard key is release
    signals :
          void size_changed(); //emited when the size of the window changes

private:
    Ui::MainWindow *ui;
    ArmWidget arm;
    ArmRotationWidget arm_rotation;
    ArmCircleWidget armCircle;
    GripperWidget gripper;
    WristWidget wrist;
    Gps gps;
    //Joystick j;
    Ros r;
    Hokuyo hokuyo;
    bool next_gripper_state; //false open, true close
    bool LRF_lines_show;
    bool IR_data_show;
    QMessageBox msgBox;
    QElapsedTimer gpsUrlTImer;



    public slots:
            void change_url(QUrl url); //set url to the web viewer
            void connect_clicked(); // executed when the connect button is pushed
            void update_ptz(QImage image); //order the ptz camera scene to update
            void update_map(QImage image); //order the map image scene to update
            void update_rear(QImage image); //order the rear camera scene to update
            void update_kinectRGB(QImage image); //order the kinect RGB camera scene to update
            void update_kinectDepth(QImage image); //order the kinect Depth camera scene to update
            void bumper_update_slot(int bumper1, int bumper2, int bumper3, int bumper4);
            void Pan_control(int value);
            void Tilt_control(int value);
            void Pan_Tilt_reset();
            void encoder_info_update(double linearVelocity,double angularVelocity);
            void move_speed_select(int x);
            void turning_speed_select(int x);
            void motor_control_toggle(int value);
            void irdata_update_slot(double ir01,double ir02);
            void spatial_update_slot(double acc_x,double acc_y, double acc_z, double ang_x, double ang_y, double ang_z, double mag_x, double mag_y, double mag_z);
            void showLRFlines();
            void showIRdata(int);
            void showLRFdata(int);
};

#endif // MAINWINDOW_H
