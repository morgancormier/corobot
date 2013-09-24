#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

//Definition of the messages that will be written on the lcd display


//Motion
#define MOTOR_DISCONNECTED ""
#define MOTOR_CONTROLLER_DISCONNECTED "Phidgets Motor controller disconnected - Please make sure it is connected, try to disconnect and reconnect it, or restart"
#define motorSpeedError "Cannot Set Motor Speed - Make sure the Phidget Motor controller board has not been disconnected and the speed is within the range"
#define ENCODER_BOARD_DISCONNECTED "Phidgets Encoder board disconnected - Please make sure it is connected, try to disconnect and reconnect it, or restart"
#define encoderValueError "Cannot get encoder value - Please make sure the encoder board and encoders are connected"
#define SSC32_ERROR_CONNECTION "Can't connected to the ssc32 controller - Please make sure it is connected, that the port is correct and that you have the permissions"
#define ARBOTIX_ERROR_CONNECTION ""
#define PHIDGET_SERVO_ERROR_CONNECTION "Can't connect to the Phidget Servo controller - Please make sure it is connected and the Phidgets library is installed"
#define PHIDGET_STEPPER_ERROR_CONNECTION "Can't connect to the Phidget Stepper controller - Please make sure it is connected and that the serial number given is correct"
#define ERROR_MOVING_ARM "Can't move the arm - Please make sure the servo motors are connected to the board"
#define ARM_OUT_OF_LIMIT ""
#define OBSTACLE_HIT ""

//GPS
#define GPS_INIT_ERROR "GPS can't initialize - Please verify the gps connection and configurations"
#define GPS_WRONG_LIB "Wrong lib gpsd version - Please install or update gpsd"

//Camera
#define CAMERA_DISCONNECTED "Camera disconnected - Please make sure it is connected and verify the port"
#define ERROR_CAMERA_PARAMETERS "Camera can't be initialized - Please verify the parameters"
#define ERROR_MOVING_CAMERA "Camera can't move - Please make sure it is a pan and tilt camera"

// Other Sensors 
#define PhidgetIK_INIT_ERROR "Phidget 8/8/8 can't be initialized - Please make sure it is connected, try to disconnect and reconnect, or restart"
#define PhidgetIMU_INIT_ERROR "IMU can't be initialized - Please make sure it is connected, try to disconnect and reconnect, or restart"

//Battery
#define LOW_BATTERY ""

//Connections
#define KEYBOARD_DISCONNECTED ""
#define MOUSE_DISCONNECTED ""
#define VGA_DISCONNECTED ""

//Memory
#define LOW_RAM ""
#define LOW_HDD_SPACE ""

//
#endif //DIAGNOSTICS_H
