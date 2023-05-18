//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/AMS_state.h"				// message that says the AMS state
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
#include "jetson/task_info.h"				// message that comes from mission control
//Global Variables
int Task = 0;
//Task1
std::string AMS_state_str;
int AMS_state = 0; 
std::string UAV_state_str;
int UAV_state = 0; 
char Task1_str[2];


// THIS FUNCTION: Subscribes to the task topic
// ACCEPTS: jetson/task_info.msg
// RETURNS: (VOID)
// =============================================================================
void Task_Control (const jetson::task_info::ConstPtr& msg)									//Task name subscriber
{
	Task = msg->name.data;
}//END OF //Task_Control()

//======================TASK 1==================================================
// THIS FUNCTION: Gets the AMS Status and writes char variable 
// ACCEPTS: jetson/AMS_state.msg
// RETURNS: (VOID)
// =============================================================================
void AMS_status (const jetson::AMS_state::ConstPtr& msg)
{
	AMS_state_str = msg->AMS;
	UAV_state_str = msg->UAV;
	
	if (AMS_state_str =="Remote Operated")
		Task1_str[0] = 1;
	else if (AMS_state_str  == "Autonomous")
		Task1_str[0] = 2;
	else if (AMS_state_str =="Killed")
		Task1_str[0] = 3;
	if (UAV_state_str == "Stowed")
		Task1_str[1] = 1;
	else if (UAV_state_str == "Deployed")
		Task1_str[1] = 2;
	else if (UAV_state_str == "Faulted")
		Task1_str[1] = 3;
	
	ROS_INFO("%c",Task1_str[0]);
} // END OF AMS_status()

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Heartbeat_Control");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(1);

	ros::Subscriber AMS_status_sub = n.subscribe("AMS_status", 1, AMS_status);					//Obtains what is the current AMS state 
	ros::Subscriber Task_info_sub = n.subscribe("task_info", 1,Task_Control);							//Obtains current AMS task 
	
	
  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	if (Task == 1)
	{
		ROS_INFO("%c",Task1_str[0]);
	}
	
	else if (Task == 2)
	{
		ROS_INFO("Task 2");
	}
	
	else if (Task == 3)
	{
		ROS_INFO("Task 3");
	}
	
	else if (Task == 4)
	{
		ROS_INFO("Task 4");
	}
	
	else if (Task == 5)
	{
		ROS_INFO("Task 5");
	}
	
	else if (Task == 6)
	{
		ROS_INFO("Task 6");
	}
	
	else if (Task == 7)
	{
		ROS_INFO("Task 7");
	}
	
	else if (Task == 8)
	{
		ROS_INFO("Task 8");
	}
	
	else if (Task == 9)
	{
		ROS_INFO("Task 9");
	}
	
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }

  return 0;
}



