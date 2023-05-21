//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "jetson/zed2i_msg.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
#include "jetson/task_info.h"				// message that comes from mission control
#include "jetson/AMS_state.h"
#include "jetson/Acoustics_msg.h"
#include "jetson/Detect_Dock_Fling.h"
#include "jetson/NED_objects.h"  // message type that has an array of pointstamped
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Task5_Publisher");
  ros::NodeHandle n;
  
   //declare variables
jetson::zed2i_msg str_pub;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare publisher "squarenum" is the name of the node
  //1 is the number of values to keep stored until they are overwritten
  ros::Publisher obj_rec_pub = n.advertise<jetson::zed2i_msg>("obj_rec", 1);
  ros::Publisher task_info_pub = n.advertise<jetson::task_info>("task_info", 1);
  ros::Publisher AMS_status_pub = n.advertise<jetson::AMS_state>("AMS_status", 1);
  ros::Publisher CC_animals_ned_pub = n.advertise<jetson::NED_objects>("CC_animals_ned", 1);
  ros::Publisher Acoustics_gate_pub = n.advertise<jetson::Acoustics_msg>("Acoustics_gate", 1);
   ros::Publisher Detect_Dock_pub = n.advertise<jetson::Detect_Dock_Fling>("Det_Dock", 1);
  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	
   
    /* 
	str_pub.objects[0].header.frame_id = "Red Light";
	Task5_pub.publish(str_pub);
	*/
    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}




