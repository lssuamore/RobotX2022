//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/AMS_state.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
//Global Variables
std::string Item_state_str;
int AMS_state = 0; 
std::string UAV_state_str;
int UAV_state = 0; 
char UAV_Task8[2];


void AMS_status (const jetson::AMS_state::ConstPtr& msg)
{
	Item_state_str = msg->Item;
	UAV_state_str = msg->UAV;

	if (UAV_state_str == "Stowed")
		UAV_Task8[0] = 1 + '0';
	else if (UAV_state_str == "Deployed")
		UAV_Task8[0] = 2 + '0';
	else if (UAV_state_str == "Faulted")
		UAV_Task8[0] = 3 + '0';
	
		if (Item_state_str == "Not Picked Up")
		UAV_Task8[1] = 1 + '0';
	else if (Item_state_str == "Picked Up")
		UAV_Task8[1] = 2 + '0';
	else if (Item_state_str == "Delivered")
		UAV_Task8[1] = 3 + '0';
} // END OF AMS_status()

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_Tutorial_Listener");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(1);

  //declare subsrcriber "squarenum" is the name of the node
  //1 is how many to save in the buffer
  //uint_function is the function called when a value is recieved
	ros::Subscriber AMS_status_sub = n.subscribe("AMS_state", 1, AMS_status);						//Obtains what is the current AMS state 

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	ROS_INFO("UAV  State is: %c",UAV_Task8[0]);
	ROS_INFO("Item Status is %c",UAV_Task8[1]);
    //looks for data
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }

  return 0;
}



