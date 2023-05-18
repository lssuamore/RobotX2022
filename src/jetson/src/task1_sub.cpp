//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/AMS_state.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
//Global Variables
std::string AMS_state_str;
int AMS_state = 0; 
std::string UAV_state_str;
int UAV_state = 0; 

void AMS_status (const jetson::AMS_state::ConstPtr& msg)
{
	AMS_state_str = msg->AMS;
	UAV_state_str = msg->UAV;
	if (AMS_state_str =="Remote Operated")
		AMS_state = 1;
	else if (AMS_state_str  == "Autonomous")
		AMS_state = 2;
	else if (AMS_state_str =="Killed")
		AMS_state = 3;
	if (UAV_state_str == "Stowed")
		UAV_state = 1;
	else if (UAV_state_str == "Deployed")
		UAV_state = 2;
	else if (UAV_state_str == "Faulted")
		UAV_state = 3;
} // END OF object_rec()

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
	ROS_INFO("AMS State is: %d",AMS_state);
	ROS_INFO("UAV  State is: %d",UAV_state);
    //looks for data
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }

  return 0;
}



