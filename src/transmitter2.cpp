#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
//int Arr[];
bool t=false;
float x,y,pitch,roll,yaw,lclX,lclY,lclYaw,nxtX,nxtY,nxtYaw,goX,goY,goYaw,laseryaw,tarX,tarY;
bool slam,IMU,obs_av_way,obj_tr_way;
geometry_msgs::PoseStamped height;
//geometry_msgs::PoseStamped next_way;
//geometry_msgs::PoseStamped local_goal_position;
void send_cmd(int, int);
//void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);


//int fd;
int thr_value=0;
int yaw_value=63;
int pitch_value=63;
int roll_value=63;
int mode_value=127;

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

int open_port(char* str)
{
	int fd; // file description for the serial port
	
    fd = open(str/*"/dev/ttyUSB0"*/, O_RDWR| O_NONBLOCK | O_NDELAY);
	
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


void Send_Data(int fd,float X_laser, float Y_laser, float X_waypoint, 
	float Y_waypoint, float yaw_waypoint, float imu_pitch,
	float imu_roll, float imu_yaw, float imu_p1, float imu_p2, float imu_p3,
	int height)
{
	unsigned char* X_laser_byte = (unsigned char*)&X_laser;
	unsigned char* Y_laser_byte = (unsigned char*)&Y_laser;

	unsigned char* X_waypoint_byte = (unsigned char*)&X_waypoint;
	unsigned char* Y_waypoint_byte = (unsigned char*)&Y_waypoint;
	unsigned char* yaw_waypoint_byte = (unsigned char*)&yaw_waypoint;

	unsigned char* imu_pitch_byte = (unsigned char*)&imu_pitch;
	unsigned char* imu_roll_byte = (unsigned char*)&imu_roll;
	unsigned char* imu_yaw_byte = (unsigned char*)&imu_yaw;
	unsigned char* imu_p1_byte = (unsigned char*)&imu_p1;
	unsigned char* imu_p2_byte = (unsigned char*)&imu_p2;
	unsigned char* imu_p3_byte = (unsigned char*)&imu_p3;

	unsigned char* height_byte = (unsigned char*)&height;

	unsigned char send_bytes[]={0xFA,0xFB,0xFC,0xFD,X_laser_byte[0],X_laser_byte[1],
			X_laser_byte[2],X_laser_byte[3],Y_laser_byte[0],Y_laser_byte[1],
			Y_laser_byte[2],Y_laser_byte[3],

			X_waypoint_byte[0],X_waypoint_byte[1],
			X_waypoint_byte[2],X_waypoint_byte[3],
			Y_waypoint_byte[0],Y_waypoint_byte[1],
			Y_waypoint_byte[2],Y_waypoint_byte[3],
			yaw_waypoint_byte[0],yaw_waypoint_byte[1],
			yaw_waypoint_byte[2],yaw_waypoint_byte[3],
			
			imu_pitch_byte[0],imu_pitch_byte[1],
			imu_pitch_byte[2],imu_pitch_byte[3],
			imu_roll_byte[0],imu_roll_byte[1],
            imu_roll_byte[2],imu_roll_byte[3],
			imu_yaw_byte[0],imu_yaw_byte[1],
			imu_yaw_byte[2],imu_yaw_byte[3],
			

			imu_p1_byte[0],imu_p1_byte[1],
			imu_p1_byte[2],imu_p1_byte[3],
			imu_p2_byte[0],imu_p2_byte[1],
			imu_p2_byte[2],imu_p2_byte[3],
			imu_p3_byte[0],imu_p3_byte[1],
			imu_p3_byte[2],imu_p3_byte[3],

			height_byte[0],height_byte[1],
			height_byte[2],height_byte[3],

			
			0xFA^0xFB^0xFC^0xFD^X_laser_byte[0]^X_laser_byte[1]^
			X_laser_byte[2]^X_laser_byte[3]^Y_laser_byte[0]^Y_laser_byte[1]^
			Y_laser_byte[2]^Y_laser_byte[3]^

			X_waypoint_byte[0]^X_waypoint_byte[1]^
			X_waypoint_byte[2]^X_waypoint_byte[3]^
			Y_waypoint_byte[0]^Y_waypoint_byte[1]^
			Y_waypoint_byte[2]^Y_waypoint_byte[3]^
			
			imu_pitch_byte[0]^imu_pitch_byte[1]^
			imu_pitch_byte[2]^imu_pitch_byte[3]^
			imu_roll_byte[0]^imu_roll_byte[1]^
			imu_roll_byte[2]^imu_roll_byte[3]^
			imu_yaw_byte[0]^imu_yaw_byte[1]^
			imu_yaw_byte[2]^imu_yaw_byte[3]^
			
			
			imu_p1_byte[0]^imu_p1_byte[1]^
			imu_p1_byte[2]^imu_p1_byte[3]^
			imu_p2_byte[0]^imu_p2_byte[1]^
			imu_p2_byte[2]^imu_p2_byte[3]^
			imu_p3_byte[0]^imu_p3_byte[1]^
			imu_p3_byte[2]^imu_p3_byte[3]^

			height_byte[0]^height_byte[1]^
			height_byte[2]^height_byte[3]};
	
	//unsigned char send_bytes[]={0xFA,0xFB,0xFC,0xFD};	
	write(fd,send_bytes,53);
	
}

void IMU_pose_callback(const geometry_msgs::PoseStamped& input)
  {

      //pitch roll yaw are the current attitude of UAV from IMU.cpp
      pitch=input.pose.orientation.x;
      roll=input.pose.orientation.y;
      yaw=input.pose.orientation.z;
       IMU=true;
   }

void slam_out_pose_callback(const geometry_msgs::PoseStamped& input)
  {

      //pox poy yaw are the current x,y position of UAV from hector_slam
      x=input.pose.position.x;
      y=input.pose.position.y;
	laseryaw = atan2(2.0*input.pose.orientation.z*input.pose.orientation.w, 1.0-2.0*pow(input.pose.orientation.z, 2));
      slam=true;

   }

void local_goal_position(const geometry_msgs::PoseStamped& input)
  {
	//waypoints coming from obstacle avoidance algorithm
      lclX=input.pose.position.x;
      lclY=input.pose.position.y;
      lclYaw=input.pose.position.z;
      obs_av_way=true;

   }
void next_way(const geometry_msgs::PoseStamped& input)
  {
	//waypoints coming from object triangulation
      nxtX=input.pose.position.x;
      nxtY=input.pose.position.y;
      nxtYaw=input.pose.position.z;
      obj_tr_way=true;
   }

void target_estimate(const geometry_msgs::PoseStamped& input)
  {
	//target_estimate coming from object triangulation
      tarX=input.pose.position.x;
      tarY=input.pose.position.y;
      //tarZ=input.pose.position.z;
      
   }
/*void pub_img(const std_msgs::Bool& input)
  {
	
      t=input.data;
   }*/


FILE* fp;

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "transmitter2");
    ros::NodeHandle n_;
    ros::Publisher pub_height;
ros::Subscriber sub_local_goal_position;
  ros::Subscriber sub_next_way;
ros::Subscriber sub_target_estimate;
    ros::Subscriber IMU_pose;
     ros::Subscriber sub_slam_out_pose;
   ros::Rate loop_rate(25);
ros::Subscriber sub_img; 
   int fd = open_port("/dev/ttyUSB0");
	configure_port(fd);

    int fd1 = open_port("/dev/ttyUSB1");
    configure_port(fd1);
	//timer for 40ms rate
	clock_t start,stop;	
	
    unsigned char buffer[4];
    int dist,p;
    unsigned char byte1,byte2,byte3;

    fp = fopen("/home/toledo/test1.txt","w+");
    if(fp==NULL) return 0;		

   pub_height = n_.advertise<geometry_msgs::PoseStamped>("/height",1000);
	start = clock();
     while (ros::ok())
	{
	stop = clock();
	float time = (float)(stop - start)*1000.0/CLOCKS_PER_SEC;
	if(time > 50.0)
	{	
         if(read(fd1, buffer, 4) > 0)
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
		start = clock();
	}
         }
        IMU_pose= n_.subscribe("/current_attitude", 1, IMU_pose_callback);
        sub_slam_out_pose= n_.subscribe("/slam_out_pose", 1, slam_out_pose_callback);
	sub_local_goal_position = n_.subscribe("/local_goal_position", 1, local_goal_position);
  	sub_next_way = n_.subscribe("/next_way", 1, next_way);
	sub_target_estimate = n_.subscribe("/target_estimate", 1, target_estimate);
	//sub_img = n_.subscribe("/pub_img", 100, pub_img);
//Algorithm Switching

//we need to use waypoints from obstacle avoidance if our x is more than 9.5. After we reach target room, we want to use waypoints from object triangulation. 

	if (x>9.5)
	{
		goX = nxtX;
		goY = nxtY;
		goYaw = nxtYaw;
	}
	else
	{
		goX = lclX;
		goY = lclY;
		goYaw = lclYaw;
	}

//--------------------------------------------
       // if ((slam==true) && (IMU==true)){
        //if (IMU==true){
            Send_Data(fd, x, y, goX,
            goY, goYaw, tarX, tarY,
	    laseryaw, 4256, 6.21111, 2.0,
            dist);
            //slam=false;IMU=false;
         //}

	    //fprintf(fp,"%f\n",x);	

            printf("X is %f \n",x);
            printf("Y is %f \n",y);

            printf("pitch is %f \n",pitch);
            printf("roll is %f \n",roll);
            printf("yaw is %f \n",yaw);
			std::cout << "done\n";
	height.pose.position.x=dist;
	pub_height.publish(height);
        loop_rate.sleep();
        ros::spinOnce();
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
	
	fclose(fp);

	return(0);
	
} //main


// cmd=0-> throttle ...
/*//void send_cmd(int cmd, int value)
{
	unsigned char send_bytes[4];
	// header byte
	send_bytes[0]=0x81;
	// throttle=0x82, yaw=0x83, pitch=0x84, roll=0x85, mode=0x86;
	send_bytes[1]=0x82 + cmd;
	// value bayte	
	send_bytes[2]=value;
	//CRC byte
	send_bytes[3]=send_bytes[0] ^ send_bytes[1] ^ send_bytes[2]; 
	write(fd, send_bytes,4);
	printf("done\n");
}*/
