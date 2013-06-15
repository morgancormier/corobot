#include "joystick.h"

#ifdef Q_WS_WIN
#include "windows.h"
#include "mmsystem.h"
#else
#include "linux/joystick.h"
#include <unistd.h>
#include <fcntl.h>
#endif

/** the joystick device **/
const char* JOY_DEV = "/dev/input/js0";
/** 50 milliseconds **/
const int MS_50 = 50000;

/** specifies which joystick axis is X **/
const int DRIVE_X_AXIS = 0;
/** specifies which joystick axis is Y **/
const int DRIVE_Y_AXIS = 1;

const int ARM_X_AXIS = 4;
const int ARM_Y_AXIS = 5;
const int WRIST_AXIS = 2;
const int GRIPPER_AXIS = 3;

/** tells whether or not the drive is enabled **/
const int DRIVE_ENABLE = 0;

Joystick::Joystick(QWidget *parent){

    p = parent;
}

double Joystick::normalizeAxis(int value) //normalize the joystick axis
{
    if (value > 0)
    {
        if (value < 100)
        {
            return 0.0;
        }
        else
        {
            return -(value - 100) / 32668.0;
        }
    }
    else
    {
        if (value > -100)
        {
            return 0.0;
        }
        else
        {
            return -(value + 100) / 32668.0;
        }
    }
}


void Joystick::run(){

double *axis=NULL;
int num_of_axis=0;

#ifndef Q_WS_WIN
    int joy_fd;


    int x;
    struct js_event js;

    // keep looping until we open the joystick
    while ((joy_fd = open( JOY_DEV , O_RDONLY)) == -1)
    {
        usleep(MS_50);
    }

    // determine what the capabilities are supported
    ioctl( joy_fd, JSIOCGAXES, &num_of_axis );

    // allocate memory to support what we have
    axis = (double *) calloc( num_of_axis, sizeof( double ) );

    fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */



#else

    UINT wNumDevs;
    JOYINFO joyinfo;
    JOYCAPS joycaps;
    if((wNumDevs = joyGetNumDevs()) != 0)
    {
        // keep looping until we open the joystick
        while (joyGetPos(JOYSTICKID1,&joyinfo) != JOYERR_UNPLUGGED)
        {
            usleep(MS_50);
        }
        joyGetDevCaps(JOYSTICKID1,&joycaps,sizeof(joycaps));


        joySetCapture(p->winId(),JOYSTICKID1, 0, TRUE);
        joySetCapture(p->winId(),JOYSTICKID2, 0, TRUE);
    }

#endif
#ifndef Q_WS_WIN

    // clear the read queue
    while (read(joy_fd, &js, sizeof(struct js_event)) > 0)
    {
    }

    // hang here forever reading from the joystick
    while (1)
    {
        // Process the current joystick state and read the next one
        while (read(joy_fd, &js, sizeof(struct js_event)) <= 0)
        {
            if (axis[ARM_X_AXIS] > 0.5)
            {
                arm->moveArmLeft();
            }
            else if (axis[ARM_X_AXIS] < -0.5)
            {
                arm->moveArmRight();
            }
            if (axis[ARM_Y_AXIS] > 0.5)
            {
                arm->moveArmUp();
            }
            else if (axis[ARM_Y_AXIS] < -0.5)
            {
                 arm->moveArmDown();
            }
            if (axis[GRIPPER_AXIS] < -0.5)
            {
                 ros->openGripper();
            }
            else if (axis[GRIPPER_AXIS] > 0.5)
            {
                ros->closeGripper();
            }
            if (axis[WRIST_AXIS] < -0.5)
            {
                wrist->turnCounterClockwise();
            }
            else if (axis[WRIST_AXIS] > 0.5)
            {
                wrist->turnClockwise();
            }
            usleep(MS_50);
        }

        // see what to do with the event
        switch (js.type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_AXIS:
            axis[js.number] = normalizeAxis(js.value);
            // is this a drive axis change? If so inform the drive system
            if ((js.number == DRIVE_X_AXIS) || (js.number == DRIVE_Y_AXIS))
            {
                JoystickAxisChange(axis[DRIVE_X_AXIS], axis[DRIVE_Y_AXIS]);
            }
            break;
        case JS_EVENT_BUTTON:
            if (js.number == DRIVE_ENABLE)
            {
                // inform the drive service of a button status change
                DriveEnableChange(js.value);
            }
            break;
        }
    }
    close( joy_fd );	// too bad we never get here

#endif
}

void Joystick::JoystickAxisChange(double x, double y)
//computes the new speed of the robot
{
        double r=sqrt(x*x+y*y);
        if (r>1.0)
                r=1.0;
        double radTheta=atan2(-y,x)-M_PI/2.0;

        float translation=-r*cos(radTheta);
        float rotation=-r*sin(radTheta);

        if (x==0)
                {
                        if (y==0)
                                {
                                        translation=0.0;
                                        rotation=0.0;
                                }
                        else
                                {
                                        rotation=0.0;
                                }
                }
        else
                {
                        if (y==0)
                                {
                                        translation=0;
                                        rotation=x;
                                }
                        else
                                {
                                        // neither is zero, do nothing
                                }
                }
/*
        if (driveEnabled)
    {
            ros->setSpeed(translation,rotation);
    }
    else
    {
        ros->setSpeed(0.0, 0.0);
        }*/
}

void Joystick::DriveEnableChange(const int value)
//if value is true, the robot can moves thanks to the joystick
{
        if (value != 0)
        {
                driveEnabled = true;
        }
        else
        {
                driveEnabled = false;
        }
}

void Joystick::setArmWidget(ArmWidget * arm_)
//gives pointer to the arm widget
{
    arm = arm_;
}

void Joystick::setRos(Ros * ros_)
//gives pointer to the ros thread
{
    ros = ros_;
}

void Joystick::setWristWidget(WristWidget * wrist_)
//gives pointer to the wrist widget
{
    wrist = wrist_;
}
