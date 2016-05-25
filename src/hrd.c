#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <iostream>


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
	
    fd = open("/dev/ttyUSB1", O_RDWR| O_NONBLOCK | O_NDELAY);
	
	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		printf("open_port: Unable to open /dev/ttyUSB0. \n");
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
	
	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return(fd);

} //configure_port

int state=0;
int main(void)
{ 
	int fd = open_port();
	configure_port(fd);

	//timer for 50ms rate
	clock_t start,stop;	
	unsigned char buffer[4];
	int dist,p;
	unsigned char byte1,byte2,byte3;
	
	start = clock();
	while(1)
	{
		stop = clock();
		float time = (float)(stop - start)*1000.0/CLOCKS_PER_SEC; 
        if(time > 20.0)
		{
			if(read(fd, buffer, 4) > 0)
			{
				for(int k=0;k<4;k++)
				{
					//printf("%d\n\n",k);
					if(buffer[k]==0x81)
					{
						if(k+1 >= 4) p=k-3; else p=k+1;
						byte1 = buffer[p];
						if(k+2 >= 4) p=k-2; else p=k+2;
						byte2 = buffer[p];
						if(k+3 >= 4) p=k-1; else p=k+3;
						byte3 = buffer[p];
						if(byte3 == (0x81 ^ byte1 ^ byte2))
						{
							dist = byte1*128+byte2;
							printf("%d\n",dist);
							break;
						}	
					}
				}
			}
				//printf("%x %x %x %x\n",buffer[0],buffer[1],buffer[2],buffer[3]);
				//std::cout << "done\n";
				start = clock();			
		}
	}
		

	//---------------------------
/*
	float fb = 2.3657;
	unsigned char* aa = (unsigned char*)&fb;
	
	printf("%u\t%u\t%u\t%u\n",aa[0],aa[1],aa[2],aa[3]);
	
	//unsigned char aa[]={161,103,23,64};
	
	float* fb2 = (float*)aa;
	
	printf("fb2=%f\n",*fb2);
	
*/	
	//===========================
	return(0);
	
} //main


