#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QThread>
#include <QWidget>
#include "ArmWidget.h"
#include "Ros.h"
#include "WristWidget.h"

class Joystick : public QThread {

private:
    double normalizeAxis(int value);//normalize the joystick axis
    QWidget *p;
    bool driveEnabled;
    ArmWidget *arm;
    Ros *ros;
    WristWidget *wrist;

    public:
        void run();
        Joystick(QWidget *parent = 0);
        void JoystickAxisChange(double x, double y);//computes the new speed of the robot
        void DriveEnableChange(const int value);//if value is true, the robot can moves thanks to the joystick
        void setArmWidget(ArmWidget * arm_);//gives pointer to the arm widget
        void setRos(Ros * ros_);//gives pointer to the ros thread
        void setWristWidget(WristWidget *wrist_);//gives pointer to the wrist widget

    };

#endif // JOYSTICK_H
