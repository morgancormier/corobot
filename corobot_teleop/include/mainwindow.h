#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "ArmWidget.h"
#include "ArmRotationWidget.h"
#include "GripperWidget.h"
#include "WristWidget.h"
#include "GPS.h"
#include "Ros.h"
#include "Image.h"
#include "sensor_msgs/Image.h"
#include "corobot_msgs/PanTilt.h"
#include "Hokuyo.h"

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
    void setArguments(int argc_, char *argv_[]);

protected:
    void keyPressEvent(QKeyEvent *event);//executed each time a keyboard key is pushed
    void keyReleaseEvent(QKeyEvent *event);//executed each time a keyboard key is release
    signals :
          void size_changed(); //emited when the size of the window changes

private:
    Ui::MainWindow *ui;
    ArmWidget arm;
    ArmRotationWidget arm_rotation;
    GripperWidget gripper;
    WristWidget wrist;
    Gps gps;
    Ros r;
    Hokuyo hokuyo;
    bool next_gripper_state; //false open, true close
    bool LRF_lines_show;
    bool IR_data_show;
    QMessageBox msgBox;
    QElapsedTimer gpsUrlTImer;
    int argc;
    char **argv;

    // Object Image representing the camera steam that is displayed on the graphicsview
    Image image_kinect_depth;// allcamera scene : bottom right image
    Image image_kinect_rgb;// allcamera scene : bottom right image
    Image image_rear_cam;
    Image image_ptz_cam;
    Image image_front_cam;
    Image image_map_image;



    public slots:
    void change_url(QUrl url); //set url to the web viewer
    void connect_clicked(); // executed when the connect button is pushed
    void update_ptz(QImage image); //order the ptz camera scene to update
    void update_map(QImage image); //order the map image scene to update
    void update_rear(QImage image); //order the rear camera scene to update
    void update_kinectRGB(QImage image); //order the kinect RGB camera scene to update
    void update_kinectDepth(QImage image); //order the kinect Depth camera scene to update
    void bumper_update_slot(int bumper1, int bumper2, int bumper3, int bumper4); // update bumper data
    void Pan_Tilt_reset(); // ask the ros node to reset the pan and tilt position of the camera
    void encoder_info_update(double linearVelocity,double angularVelocity); // update the encoder info on the interface
    void motor_control_toggle(int value); // interface the toggle button "Motor Control Disable"
    void irdata_update_slot(double ir01,double ir02); //update the onscreen value of the infrared sensor
    void imu_update_slot(double acc_x,double acc_y, double acc_z, double ang_x, double ang_y, double ang_z); //update the imu values on the interface
    void magnetic_update_slot(double mag_x, double mag_y, double mag_z); //update the imu values on the interface
    void showLRFlines(); // Interface the show line button in the rangefinding interface
    void showIRdata(int); // Interface the show ir data button in the rangefinder interface
    void showLRFdata(int); // Interface the show LRF button in the rangefinder interface
};

#endif // MAINWINDOW_H
