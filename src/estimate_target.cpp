#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h> 
#include <stdio.h> 
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
geometry_msgs::PoseStamped next_way;
geometry_msgs::PoseStamped target_estimate;
bool pub_img;
bool status = false;
float tr = 13.16;		// x position of target room (meters)
float delyaw = 0;
float posx = 0;
float posy = 0;
float yaw = 0;
float yaw_2 = 0;
float yaw_3 = 0;
float yaw_4 = 0;
float yawt = 0;
float yawt2 = 0;
float yawt3 = 0;
float xx2 = 0;
float yy2 = 0;
float x = 0;		//actual target position x
float y = 0;		//actual target position y
float px1 = 0;
float py1 = 0;
float px2 = 0;
float py2 = 0;
float yaw1 = 0;
float yaw2 = 0;
float randx = 15.0;	// target position estimate x
float randy = 6.0;	// target position estimate y
float search_yaw = 1.221;	// search angle for target (radians)
float first_yaw = 0;
float mid_yaw = 0;
double threshold_dis = 0.4; //minimum distance the UAV must move to triangulate object.
float dis = 0; //distance the UAV has moved.
double threshold_yaw = 0.7; //minimum angle the UAV must move to triangulate object.
int control = 0; //used to control step in triangulation sequence
bool yaw_flag=false;
bool yaw_flag2=false;
bool yaw_flag3=false;
bool another_flag = false;
bool triangulated = false;
/*
void object_pic_callback(const std_msgs::Bool& input){
      pub_img = input.data;
}*/

void slam_out_pose_callback(const geometry_msgs::PoseStamped& input){
      posx = input.pose.position.x;
      posy = input.pose.position.y;
      yaw = atan2(2.0*input.pose.orientation.z*input.pose.orientation.w, 1.0-2.0*pow(input.pose.orientation.z, 2));
}
       
void slam_object_callback(const std_msgs::Bool& msg){
      status = msg.data;

      //std::cout << "Status = " << status << std::endl;
      //std::cout << "control = " << control << std::endl;

		
if (posx >= tr)
{

if (1)
{
		 next_way.pose.position.x=posx;
                 next_way.pose.position.y=posy;
		 next_way.pose.orientation.x=0;
                 next_way.pose.orientation.y=0;
                 next_way.pose.orientation.z=0;
 
	if ((yaw < search_yaw) && (another_flag == false))
	{
                next_way.pose.position.z=yaw + 0.1;
	}
	if (yaw >= search_yaw)
	{
	another_flag = true;
	next_way.pose.position.z=yaw - 0.1;
	}
            ros::NodeHandle n_;
ros::Publisher pub_next_way;
pub_next_way = n_.advertise<geometry_msgs::PoseStamped>("/next_way",1000);
pub_next_way.publish(next_way);
}

if (1)
{
x = randx;
y = randy;
std::ofstream out("/home/ubuntu/Desktop/target_position.txt");
out << "Target position x (m) = " << x << "\n";
out << "Target position y (m) = " << y << "\n";
out << "Target position z (m) = 2.0 \n";
out << "Uncertainty in target position (radius of sphere in meters) = 1.75";
out.close();
}

/*

if (!triangulated)
{
x = randx;
y = randy;
std::ofstream out("/home/ubuntu/Desktop/target_position.txt");
out << "Target position x (m) = " << x << "\n";
out << "Target position y (m) = " << y << "\n";
out << "Target position z (m) = 2.0 \n";
out << "Uncertainty in target position (radius of sphere in meters) = 1.75";
out.close();
}
*/
      if(control == 0){
              //first step in triangulation sequence.  Whenever callback from ObjectTracker reads "true", save position/yaw and progress to next step.
              if (!status){
                  std::cout << "change yaw" << std::endl;
		 if(!yaw_flag)
		{
			 yawt = yaw;
			yaw_flag=true;
		}
		 first_yaw = yaw+0.1;

		 next_way.pose.position.x=posx;
                 next_way.pose.position.y=posy;
                 next_way.pose.position.z=first_yaw;

                 next_way.pose.orientation.x=0;
                 next_way.pose.orientation.y=0;
                 next_way.pose.orientation.z=0;

		
                 /*yaw_2 = yawt+1.222;
		 next_way.pose.position.x=posx;
                 next_way.pose.position.y=posy;
                 next_way.pose.position.z=yaw_2;

                 next_way.pose.orientation.x=0;
                 next_way.pose.orientation.y=0;
                 next_way.pose.orientation.z=0;
		*/


              }else{
                  px1 = posx;
                  py1 = posy;
                  yaw1 = yaw;
		delyaw = std::abs(yawt-yaw1);
                  control = 1; //move on to next step in triangulation sequence.
                  std::cout << "Position 1 saved..." << std::endl;
		
		target_estimate.pose.position.x=posx+(3*cos(delyaw))+0.6096;
                target_estimate.pose.position.y=posy+(3*sin(delyaw))+6.0960;
                target_estimate.pose.position.z=0;

                target_estimate.pose.orientation.x=0;
                target_estimate.pose.orientation.y=0;
                target_estimate.pose.orientation.z=0;

              }
      }

      if(control == 1){
              //second step in triangulation sequence.
              dis = sqrt(pow(posx - px1,2) + pow(posy - py1,2));
              std::cout << "Yaw: " << yaw << " | Yaw_delta: " << (yaw1-yaw) << " | Yaw1: " << yaw1 << " | dis: " << dis << std::endl;
              if (dis < threshold_dis){
                if (std::abs(yaw1 - yaw) < threshold_yaw){
                    std::cout << "change yaw away from the target" << std::endl;
		if(!yaw_flag2)
		{
			 yawt2 = yaw;
			yaw_flag2=true;
		}
	         yaw_3 = yawt2-1.222;
		 next_way.pose.position.x=px1;
                 next_way.pose.position.y=py1;
                 next_way.pose.position.z=yaw_3;

                 next_way.pose.orientation.x=0;
                 next_way.pose.orientation.y=0;
                 next_way.pose.orientation.z=0;

                }else if (std::abs(yaw1 - yaw) >= threshold_yaw){
                    mid_yaw = yaw;
                    std::cout << "Yaw angle is enough.  Move away from Position 1." << std::endl;
			xx2 = px1+0.3;
			yy2 = py1+0.4;
		 next_way.pose.position.x=xx2;
                 next_way.pose.position.y=yy2;
                 next_way.pose.position.z=yaw_3;

                 next_way.pose.orientation.x=0;
                 next_way.pose.orientation.y=0;
                 next_way.pose.orientation.z=0;

                }
              }

              if (dis >= threshold_dis){
                  if (!status){
                      std::cout << "Yaw and distance are enough.  Yaw towards target to save Position 2" << std::endl;
		if(!yaw_flag3)
		{
			 yawt3 = yaw;
			yaw_flag3=true;
		}
		 yaw_4 = yawt3 + 1.222;
		 next_way.pose.position.x=xx2;
                 next_way.pose.position.y=yy2;
                 next_way.pose.position.z=yaw_4;

                 next_way.pose.orientation.x=0;
                 next_way.pose.orientation.y=0;
                 next_way.pose.orientation.z=0;

                  }else{
                      px2 = posx;
                      py2 = posy;
                      yaw2 = yaw;
                      control = 2;
                      std::cout << "Position 2 is saved..." << std::endl;
                  }
              }
      }

      if(control == 2){
              //third step in triangulation sequence.
              //let initial raw be rotate 60 degree to the left from central between UAV and target
              float length1 = dis*(sin(3.14159 - yaw2 + mid_yaw)/sin(yaw2 - yaw1));
              x = sin(yaw1)*length1 + px1;
              y = py1 + cos(yaw1)*length1;
		std::cout << "===================TRIANGULATED!!===================" << std::endl;
		triangulated = true;

std::ofstream out("/home/ubuntu/Desktop/target_position.txt");
out << "Target position x (m) = " << x << "\n";
out << "Target position y (m) = " << y << "\n";
out << "Target position z (m) = 2.0 \n";
out << "Uncertainty in target position (radius of sphere in meters) = 1.75";
out.close();

              std::cout << "x: " << x << std::endl;
              std::cout << "y: " << y << std::endl;
             std::cout << "z: 2.0" << std::endl;
      }
}


ros::NodeHandle n_;
ros::Publisher pub_next_way;
pub_next_way = n_.advertise<geometry_msgs::PoseStamped>("/next_way",1000);
pub_next_way.publish(next_way);
ros::Publisher pub_target_estimate;
pub_target_estimate = n_.advertise<geometry_msgs::PoseStamped>("/target_estimate",1000);
pub_target_estimate.publish(target_estimate);
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "object_triangulation");
  ros::NodeHandle n_;
ros::Publisher pub_next_way;
ros::Publisher pub_target_estimate;
  ros::Subscriber sub_slam_out_pose;
  ros::Subscriber sub_object_status;
 // ros::Subscriber sub_img;
  //ros::Rate loop_rate(100);

  sub_slam_out_pose = n_.subscribe("/slam_out_pose", 1000, slam_out_pose_callback);
  sub_object_status = n_.subscribe("/object_tracking", 1000, slam_object_callback);
// sub_img = n_.subscribe("/pub_img", 1000, object_pic_callback);
pub_next_way = n_.advertise<geometry_msgs::PoseStamped>("/next_way",1000);
pub_target_estimate = n_.advertise<geometry_msgs::PoseStamped>("/target_estimate",1000);
//pub_next_way.publish(next_way);
  ros::spin();
  //loop_rate.sleep();

  return 0;
}
