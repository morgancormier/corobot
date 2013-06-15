#include "ros/ros.h"
#include "ssc32.hpp"

namespace Servo
{
    //Constructor
    SSC32::SSC32(){
	nrOfPorts = 32;
	}
    
    //Destructor
	SSC32::~SSC32() {}

    void SSC32::initSerial()
    {
        // Set baud rate
        cfsetispeed(&port_settings, BAUDRATE);
        cfsetospeed(&port_settings, BAUDRATE);
        
        // Initialise the timeout structure
        timeout.tv_sec = TIMEOUT_SEC;

        // Process the configuration file
        /*FILE *cFile;
        cFile = fopen("config.txt","r");
        int m,n,o,p,q,r;
        nrOfPorts = 0;
        int res = fscanf (cFile, "%d %d %d %d %d %d", &m, &n, &o, &p, &q, &r);
        while(res == 6)
        {
            servoLimits[nrOfPorts][0] = m;
            servoLimits[nrOfPorts][1] = n;
            servoLimits[nrOfPorts][2] = o;
            servoLimits[nrOfPorts][3] = p;
            servoLimits[nrOfPorts][4] = q;
            servoLimits[nrOfPorts][5] = r;
            res = fscanf (cFile, "%d %d %d %d %d %d", &m, &n, &o, &p, &q, &r);
            nrOfPorts ++;
        }
        std::cout << "number of ports: " << nrOfPorts << std::endl;*/

    }
    
    bool SSC32::startSerial(std::string COM_PORT_)
    {
	const char *COM_PORT;
	COM_PORT = COM_PORT_.c_str();
	int flags;
	printf("COM_PORT =  %s \n",COM_PORT);
        // Try to open the serial port
        fd = open(COM_PORT, O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR);
	//fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if(fd == -1)  //Unable to open the serial port
        {
            std::cout << "(SSC-32) Unable to open " << COM_PORT<< "\n" << std::endl;
            return false;
        }

	if (tcflush(fd, TCIFLUSH) < 0)
    	{
		close(fd);
		fd = -1;
		return false;
    	}
    	if (tcgetattr(fd, &port_settings) < 0)
    	{
		close(fd);
		fd = -1;
		return false;
    	}

    	cfmakeraw(&port_settings);
    	cfsetispeed(&port_settings, BAUDRATE);
    	cfsetospeed(&port_settings, BAUDRATE);

    	if (tcsetattr(fd, TCSAFLUSH, &port_settings) < 0)
    	{
		perror("corobot_open():tcsetattr():");
		close(fd);
		fd = -1;
		return false;
    	}

	    /* We know the robot is there; switch to blocking */
	    /* ok, we got data, so now set NONBLOCK, and continue */
    	if ((flags = fcntl(fd, F_GETFL)) < 0)
    	{
		perror("corobot_comm_open():fcntl():");
		close(fd);
		fd = -1;
		return false;
    	}
	if (fcntl(fd, F_SETFL, flags ^ O_NONBLOCK) < 0)
    	{
		perror("corobot_comm_open():fcntl()");
		close(fd);
		fd = -1;
		return false;
    	}
        else
        {
            std::cout << "(SSC-32) Opened " << COM_PORT << "\n" << std::endl;
            return true;
        }

    }
    
    void SSC32::stopSerial()
    {   
        // Close the serial port
	SendMessage(0,0,0);
	SendMessage(1,0,0);
	
        close(fd);
    }

    bool SSC32::SendMessage(int port, int position, int speed)
    {
        // Check port, position and speed limits
        if(port >= nrOfPorts || port < 0)
        {
            std::cout << "(SSC-32) Port number not valid: " << port << std::endl;
            return false;
        }
        
        /*if(position < servoLimits[port][1]) position = servoLimits[port][1];
        else if(position > servoLimits[port][2]) position = servoLimits[port][2];

        if(speed > servoLimits[port][3]) speed = servoLimits[port][3];
        else if(speed < 0) speed = 0;
        
        // Scale position and speed 
        position = (position - servoLimits[port][1]) / (servoLimits[port][2] - servoLimits[port][1]) * (servoLimits[port][5] - servoLimits[port][4]) + servoLimits[port][4];

        speed = (speed) / (servoLimits[port][3]) * (servoLimits[port][5] - servoLimits[port][4]) + servoLimits[port][4];*/
        
        // Convert port, position and speed to string
        std::ostringstream temp, temp2, temp3; 
        temp << port; 
        Port = temp.str();

        temp2 << position; 
        Position = temp2.str();

        temp3 << speed; 
        Speed = temp3.str(); 
        
        // Compose actual command
	if(speed == -1)
        	command= "#" + Port + " P" + Position + " \r";  //Using T, you can use T = 0 to stop the servo output.
	else
		command= "#" + Port + " P" + Position + " T" + Speed + " \r";  //Using T, you can use T = 0 to stop the servo output.
        Output = command.c_str();
        
        // Write command to port
        int err = write(fd,Output,strlen(Output));
        std::cout << "(SSC32) " << command << "\n" << std::endl;

	if(err == -1)
		ROS_ERROR("Error sending command to SSC32 controller");
	else if(err == 0)
		ROS_WARN("The command was not send to the SSC32 controller");
	else if(err != ((int) strlen(Output)))
		ROS_WARN("Only a part of the command was sent to the SSC32 controller, %d bits out of %d", err, (int) strlen(Output));
	else
		ROS_DEBUG("Command sent successfully to the SSC32 Controller");
        return true;
    }

    bool SSC32::CheckIfFinished()
    {	
	    char someData; 
	    int err = write(fd,"Q \r",strlen("Q \r"));
	    err = read(fd,&someData,1);
	    if (char(someData) == '.') 
        {
            std::cout << "(SSC32) Servos have finished rotating " << "\n" << std::endl;
            return true;
        }
	    return false;
    }
}//namespace

