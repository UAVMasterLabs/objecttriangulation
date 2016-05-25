#include "ros/ros.h"
#include <SerialStream.h>
#include <string.h>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

geometry_msgs::PoseStamped current_attitude;

using namespace LibSerial;
using namespace std;

class Serial {
 LibSerial::SerialStream sstream;
public:
 Serial();
 virtual ~Serial();
 int sendmsg(char* msg, int buffersize);
 void request_IMUCalc();
 void read(char* data);
 void requestandread();
 void motors_on();
 void thrust_on(int com);
 void closeports();
};

Serial::Serial() {
 // TODO Auto-generated constructor stub
 const char* const SERIAL_PORT_DEVICE = "/dev/ttyUSB0" ;
     sstream.Open( SERIAL_PORT_DEVICE ) ;
     if ( ! sstream.good() )
     {
         std::cout << "Error: Could not open serial port "
                   << SERIAL_PORT_DEVICE
                   << std::endl ;
         exit(1) ;
     }
     //
     // Set the baud rate of the serial port.
     //
     sstream.SetBaudRate( SerialStreamBuf::BAUD_57600 ) ;
     if ( ! sstream.good() )
     {
         std::cout << "Error: Could not set the baud rate." << std::endl ;
         exit(1) ;
     }
     //
     // Set the number of data bits.
     //
     sstream.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
     if ( ! sstream.good() )
     {
         std::cout << "Error: Could not set the character size." << std::endl ;
         exit(1) ;
     }
     //
     // Disable parity.
     //
     sstream.SetParity( SerialStreamBuf::PARITY_NONE ) ;
     if ( ! sstream.good() )
     {
         std::cout << "Error: Could not disable the parity." << std::endl ;
         exit(1) ;
     }
     //
     // Set the number of stop bits.
     //
     sstream.SetNumOfStopBits( 1 ) ;
     if ( ! sstream.good() )
     {
         std::cout << "Error: Could not set the number of stop bits."
                   << std::endl ;
         exit(1) ;
     }
     //
     // Turn on hardware flow control.
     //
     sstream.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_HARD ) ;
     if ( ! sstream.good() )
     {
         std::cout << "Error: Could not use hardware flow control."
                   << std::endl ;
         exit(1) ;
     }
}

Serial::~Serial() {
 // TODO Auto-generated destructor stub
}

void Serial::closeports(){
 sstream.Close();
}

int Serial::sendmsg(char* msg, int buffersize){
 puts("Sending message...");
  this->sstream.write(msg , buffersize);
  puts("Message sent");
  return 1;
}



void Serial::read(char* data){
 std::cout << "Reading..." << std::endl ;
 int i = 0;
 //usleep(1000*80); //wait for the message to arrive
     while( sstream.rdbuf()->in_avail() > 0  )
         {

             char next_byte;
             sstream.get(next_byte);
             //std::cout << hex << int(next_byte) << " ";
             data[i] = next_byte;
             ++i;
         }

     std::cout << std::endl ;
     std::cout << "Done reading" << std::endl ;
}




SerialStream* openport(){

 SerialStream sstream;
 sstream.SetBaudRate( SerialStreamBuf::BAUD_57600 );
 sstream.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE );
 sstream.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 );
 sstream.SetNumOfStopBits(1);
 sstream.SetParity( SerialStreamBuf::PARITY_NONE );
 sstream.Open( "/dev/ttyUSB0" );

 if ( ! sstream.good() )
     {
         std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                   << "Error: Could not open serial port."
                   << std::endl ;
     }

 SerialStream* pointer = &sstream;

 return pointer;
}  
  





//sends the structure
int send_msg(SerialStream* ssp, char* msg, int buffersize)
{
 puts("enviando msje");
 (*ssp).write(msg , buffersize);
 puts("mensaje enviado");
 return 1;
}

int close_ports(SerialStream* ssp)
{
 ssp->Close();
 return 1;
}

float convertint(char t1,char t2,char t3,char t4,char t5,char t6,char t7,char t8)
{
   char temp[8];
   temp[0]=t1;temp[1]=t2;temp[2]=t3;temp[3]=t4;temp[4]=t5;temp[5]=t6;temp[6]=t7;temp[7]=t8;
   int float_array[32];
   int j;
   float expo,frac,val;
   expo=0;frac=0;
   //convert hex to binary
   for (int i=0;i<8;i++)
   {
     switch ( temp[i] )
      {
         case 'a':
            float_array[4*i]=1;float_array[4*i+1]=0;float_array[4*i+2]=1;float_array[4*i+3]=0;
            break;
         case 'b':
            float_array[4*i]=1;float_array[4*i+1]=0;float_array[4*i+2]=1;float_array[4*i+3]=1;
            break;
         case 'c':
            float_array[4*i]=1;float_array[4*i+1]=1;float_array[4*i+2]=0;float_array[4*i+3]=0;
            break;
         case 'd':
            float_array[4*i]=1;float_array[4*i+1]=1;float_array[4*i+2]=0;float_array[4*i+3]=1;
         break;
         case 'e':
            float_array[4*i]=1;float_array[4*i+1]=1;float_array[4*i+2]=1;float_array[4*i+3]=0;
         break;
         case 'f':
            float_array[4*i]=1;float_array[4*i+1]=1;float_array[4*i+2]=1;float_array[4*i+3]=1;
         break;
         case '0':
            float_array[4*i]=0;float_array[4*i+1]=0;float_array[4*i+2]=0;float_array[4*i+3]=0;
            break;
         case '1':
            float_array[4*i]=0;float_array[4*i+1]=0;float_array[4*i+2]=0;float_array[4*i+3]=1;
            break;
         case '2':
            float_array[4*i]=0;float_array[4*i+1]=0;float_array[4*i+2]=1;float_array[4*i+3]=0;
            break;
         case '3':
            float_array[4*i]=0;float_array[4*i+1]=0;float_array[4*i+2]=1;float_array[4*i+3]=1;
            break;
         case '4':
            float_array[4*i]=0;float_array[4*i+1]=1;float_array[4*i+2]=0;float_array[4*i+3]=0;
         break;
         case '5':
            float_array[4*i]=0;float_array[4*i+1]=1;float_array[4*i+2]=0;float_array[4*i+3]=1;
         break;
         case '6':
            float_array[4*i]=0;float_array[4*i+1]=1;float_array[4*i+2]=1;float_array[4*i+3]=0;
         break;
         case '7':
            float_array[4*i]=0;float_array[4*i+1]=1;float_array[4*i+2]=1;float_array[4*i+3]=1;
         break;
         case '8':
            float_array[4*i]=1;float_array[4*i+1]=0;float_array[4*i+2]=0;float_array[4*i+3]=0;
         break;
         case '9':
            float_array[4*i]=1;float_array[4*i+1]=0;float_array[4*i+2]=0;float_array[4*i+3]=1;
         break;
         
      }
   } 
   
    //calculate flow value
    for (int i=1;i<9;i++)
    {
     expo=expo+float_array[i]*pow(2,(8-i));
    }
    for (int i=9;i<32;i++)
    {
     frac=frac+float_array[i]*pow(2,(8-i));
    }
    val=pow(-1,float_array[0])*(1+frac)*(pow(2,expo-127));
    return val;
 }







int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "IMU");
  ros::NodeHandle n_; 
  ros::Publisher pub_current_attitude_;
  //Create an object of class GoalPlanner that will take care of everything
   int j;
 int t=200;
 int temp;
 int temp1,temp2;
 float z[t],v_z[t],des_az[t],err_z[t];
 int cmd_thrust[t];
char* mid[3];

 pub_current_attitude_ = n_.advertise<geometry_msgs::PoseStamped>("/current_attitude",1000);

 cmd_thrust[0]=400;

  Serial* serialport = new Serial();

  
	  //serialport->request_IMUCalc();
	  int size = 200; //size of data buffer
	  char databuff[size];
          char data[399];
   ros::Rate loop_rate(10);
   int count = 0;
   while (ros::ok())
   {
	  serialport->read(databuff);
	  //IMU_CALCDATA imudata = serialport->processIMU(databuff, size);
	  //serialport->displayData(imudata);
          j=0;

          for(int ii = 0; ii< 200; ++ii)
          {
               sprintf(data+(2*ii), "%02x", (unsigned char) databuff[ii]); 
     
          }
             
          while (j<399)
          {
             
                    
          if ((data[j]=='f') && (data[j+1]=='e') && (data[j+10]=='1') && (data[j+11]=='e')) 
           {
                  
                             
                 
                
                 float roll=convertint(data[j+26],data[j+27],data[j+24],data[j+25],data[j+22],data[j+23],data[j+20],data[j+21]);
                 float pitch=convertint(data[j+34],data[j+35],data[j+32],data[j+33],data[j+30],data[j+31],data[j+28],data[j+29]);
                 float yaw=convertint(data[j+42],data[j+43],data[j+40],data[j+41],data[j+38],data[j+39],data[j+36],data[j+37]);

                 float v_roll=convertint(data[j+50],data[j+51],data[j+48],data[j+49],data[j+46],data[j+47],data[j+44],data[j+45]);
                 float v_pitch=convertint(data[j+58],data[j+59],data[j+56],data[j+57],data[j+54],data[j+55],data[j+52],data[j+53]);
                 float v_yaw=convertint(data[j+66],data[j+67],data[j+64],data[j+65],data[j+62],data[j+63],data[j+60],data[j+61]);
                  //std::cout << "roll is " << roll << " ";
                  //std::cout << "pitch is " << pitch << " ";
                  //std::cout << "yaw is " << yaw << " ";

                 current_attitude.pose.position.x=v_roll;
                 current_attitude.pose.position.y=v_pitch;
                 current_attitude.pose.position.z=v_yaw;

                 current_attitude.pose.orientation.x=roll;
                 current_attitude.pose.orientation.y=pitch;
                 current_attitude.pose.orientation.z=yaw;
                    
            }
           j++;
           }
           //std::cout << std::endl ;
         pub_current_attitude_.publish(current_attitude);
        ros::spinOnce();
        //loop_rate.sleep();
        ++count;
      }

  //Close the motor

  

  serialport->closeports();
     
     


  return 0;
}
