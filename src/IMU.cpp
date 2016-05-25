/*
 * Main.cpp
 *
 *  Created on: 9-Jan-2009
 *      Author: root
 */


/////////////////////////////////////////////////
// Serial port interface program               //
/////////////////////////////////////////////////

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include "ros/ros.h"//#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <SerialStream.h>
#include <sstream>

geometry_msgs::PoseStamped attitude;
geometry_msgs::PoseStamped rpy;

void send_cmd(int, int);
int Read_PR();

int fd;
int thr_value=0;
int yaw_value=63;
int pitch_value=63;
int roll_value=63;
int mode_value=127;

unsigned char state=0; 
float imupitch,imuroll,imuyaw;
/*
class IMUData
{
public:
  IMUData()
  {
ros::NodeHandle n_; 


/*
char c;
		

	//while(!kbhit()); // wait for input
	do
	{	
		// c=getch(); 

		if(Read_PR() == 1)
		{
			printf("pitch = %f\n",imupitch);
			printf("roll = %f\n",imuroll);
			printf("yaw = %f\n",imuyaw);
std::stringstream ss;
ss << imupitch;
ss << ", ";
ss << imuroll;
ss << ", ";
ss << imuyaw;
ss << ", ";


		}
						

	}while(1);
}*/

int Read_PR()
{
	unsigned char C_in;
	unsigned char length=0;
	unsigned char src_sys, cmp_sys,seq;
	unsigned char* buff = new unsigned char [20];
	unsigned char index=0;
	bool readloop = 1;
	int res;

	while(readloop)
	{
		res = read(fd, &C_in, 1);
		if(res < 0) {printf("read error\n"); return 0;};

		switch(state)
		{
			case 0 : if(C_in == 0xFE) state++; break;
			case 1 : length = C_in; state++; break;
			case 2 : seq = C_in; state++; break;
			case 3 : src_sys = C_in; state++; break;
			case 4 : cmp_sys = C_in; state++; break;
			case 5 : if(C_in == 30){index = 0; state++;} else state=0; break;
			case 6 : if(index > 15){state++;}
				else
				{
					buff[index] = C_in;
					index++;
				}
				break;
			
			case 7 : imuroll = ((float*)buff)[1]; imupitch = ((float*)buff)[2]; imuyaw = ((float*)buff)[3]; state++; break;
			case 8 : readloop = 0; state = 0; break;

			default : state = 0;
		}

	}
	delete(buff);

	return 1;	

}


char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}



int open_port(void)
{
	int fd; // file description for the serial port
	
	fd = open("/dev/ttyTHS1", O_RDWR| O_NONBLOCK | O_NDELAY);
	
	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		printf("open_port: Unable to open /dev/ttyTHS1. \n");
		//return 0;
	}

	if ( fd < 0 )
    	{
        	//cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) 			<< endl;
        	perror("USB ");
    	}

	else
	{
		fcntl(fd, F_SETFL, 0);
		printf("port is open.\n");
	}
	
	return(fd);
} //open_port

int configure_port(int fd)      // configure the port
{
	struct termios port_settings;      // structure to store the port settings in

	cfsetispeed(&port_settings, B57600);    // set baud rates
	cfsetospeed(&port_settings, B57600);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
	
	port_settings.c_cc[VMIN]=0;
        port_settings.c_cc[VTIME]=0;

	
	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return(fd);

} //configure_port

void send_cmd()
{
	unsigned char data1[]={0xFE, 0x09, 0x00, 0xFF, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x08, 0x00, 0x00, 0x03, 0x28, 0x42};
	write(fd, data1, 17);

	unsigned char data2[]={0xFE, 0x02, 0x01, 0xFF, 0xBE, 0x15, 0x01, 0x01, 0xAC, 0xA8};
	write(fd, data2, 10);
	
	unsigned char data3[]={0xFE, 0x06, 0x02, 0xFF, 0xBE, 0x42, 0x02, 0x00, 0x01, 0x01, 0x02, 0x01, 0xC0, 0x34};   
	write(fd, data3, 14);

	unsigned char data4[]={0xFE, 0x06, 0x03, 0xFF, 0xBE, 0x42, 0x03, 0x00, 0x01, 0x01, 0x06, 0x01, 0xE5, 0x9D};   
	write(fd, data4, 14);
	
	unsigned char data5[]={0xFE, 0x06, 0x04, 0xFF, 0xBE, 0x42, 0x0A, 0x00, 0x01, 0x01, 0x0A, 0x01, 0x99, 0xD9};   
	write(fd, data5, 14);

//printf("Heartbeat Sent \n");

}

int main(int argc, char **argv)
{ 

ros::init(argc, argv, "IMUData");
ros::NodeHandle n_;
ros::Publisher pub_attitude;
pub_attitude = n_.advertise<geometry_msgs::PoseStamped>("/attitude",1);


	fd = open_port();
	configure_port(fd);
	send_cmd();
	

if(Read_PR() == 1)
{

rpy.pose.position.x = imuroll;
rpy.pose.position.y = imupitch;
rpy.pose.position.z = imuyaw;

pub_attitude.publish(rpy);


}		


	ros::spin();

	  
	return(0);
	
} //main



