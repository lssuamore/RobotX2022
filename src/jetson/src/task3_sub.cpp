//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/task_info.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
//Global Variables
std::string State;  // string array to hold the state of the task
int State_str = 0;

void task_state (const jetson::task_info::ConstPtr& msg)
{
	State = msg->state;
	
	if (State=="In Progress")
		State_str = 1;
	else if (State =="Completed")
		State_str = 2;
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
	ros::Subscriber Task_info = n.subscribe("task_info", 1, task_state);						//Obtains what task and state we are at

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	ROS_INFO("%d",State_str);
    //looks for data
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }

  return 0;
}



