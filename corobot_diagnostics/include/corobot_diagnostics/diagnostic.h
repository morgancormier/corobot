#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H

#include <vector>
#include <string>
#include "lcd.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

class Diagnostic
/**
 * This class receive error messages from a diagnostic agregator node and display the messages every 5s on the lcd display until it
 * receives a message from the agregator to remove the error message (ie the error doesn't exist anymore).
 * It also uses tturn on the buzzer everytime a new error message arrives.
 * It turns on the led red if an error message exist or green if everything is good.
 */
{
    private:
    Lcd lcd; // our lcd variable
    std::vector<std::string> errorList; // Save the list of messages that should be displayed on the LCD
    int errorIndex; // Error message that is being displayed on the lcd
    
    public:
    Diagnostic();
    ~Diagnostic();
    void newError(const std_msgs::String &message); // receive a new error message
    void removeError(const std_msgs::String &message); // remove an error message
    void timerCallback(const ros::TimerEvent&); // timer callback that takes care of displaying the next message on the lcd

};



#endif //DIAGNOSTIC_H
