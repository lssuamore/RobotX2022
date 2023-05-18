//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/Acoustics_msg.h"
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
//Global Variables
char Task2_str[0];
int Entry_gate= 0; 
char Exit_gate_str[0];
int Exit_gate = 0; 

void Acoustics_get_gate (const jetson::Acoustics_msg::ConstPtr& msg)
{
	Entry_gate = msg->Entrance.data;
	Task2_str[0] = Entry_gate + '0';
	Exit_gate = msg->Exit.data;
	Task2_str[1] = Exit_gate + '0';
} // END OFAcoustics_get_gate()

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
	ros::Subscriber Acoustics_get_sub = n.subscribe("Acoustics_gate", 1, Acoustics_get_gate);						//Obtains what is the current AMS state 

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	ROS_INFO("Entrance gate: %c",Task2_str[0] );
	ROS_INFO("Exit gate: %c",Task2_str[1] );
    //looks for data
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }

  return 0;
}



