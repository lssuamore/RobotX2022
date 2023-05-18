//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/Detect_Dock_Fling.h"
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
//Global Variables
std::string Color_Fling;  
std::string Status_Fling; 
char Task7_str[2];

void Dock_Fling(const jetson::Detect_Dock_Fling::ConstPtr& msg)
{
	
	Color_Fling= msg->Color;
	Status_Fling= msg->Status;

	if (Color_Fling == "red")
		Task7_str[0] = 'R';
	else if (Color_Fling== "blue")
		Task7_str[0] = 'B';
	else if (Color_Fling== "green")
		Task7_str[0] = 'G';
	if (Status_Fling == "Scanning")
		Task7_str[1] = '1';
	else if (Status_Fling== "Flinging")
		Task7_str[1] = '2';
	
} // END OF Dock_Fling()

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
	ros::Subscriber Detect_Dock_Fling_sub = n.subscribe("Det_Dock", 1, Dock_Fling);						//Obtains what object the camera recognise, pose of the object and order of recognition

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  
	 ROS_INFO("Color: %c", Task7_str[0]);
	  ROS_INFO("AMS Status: %c", Task7_str[1]);
	  
	     //looks for data
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }
	
 
  

  return 0;
}



