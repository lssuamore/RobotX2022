//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
//Tasks messages 
#include "jetson/AMS_state.h"				// message that says the AMS state
#include "std_msgs/UInt64.h"
#include "jetson/task_info.h"				// message that comes from mission control
#include "jetson/Acoustics_msg.h"		//Meassage published by the computer from Acoustics
#include "jetson/NED_objects.h"  // message type that has an array of pointstamped
#include "jetson/zed2i_msg.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
#include "jetson/Detect_Dock_Fling.h"		//Message for task 6 and task 7

//Global Variables
int Task = 0;
//Task1
std::string AMS_state_str;
int AMS_state = 0; 
std::string UAV_state_str;
int UAV_state = 0; 
char Task1_str[2];

// Task 2
char Task2_str[0];
int Entry_gate= 0; 
char Exit_gate_str[0];
int Exit_gate = 0; 

//Task 3
std::string State;  // string array to hold the state of the task
char Task3_str[1];

//Task 4
std::string Animal[3];  // string array to hold the names of the animals
char Animal_str[4]; // String to send animals for heartbeat
int animal_qty = 0;

//Task 5 
std::string Object[9];  // string array to hold the names of the objects from the zed2i
float x_object_NED[9], y_object_NED[9];  // arrays to hold animal locations
char Cod_Seq[100];
int cnt = 0;
int black = 0;

//Task 6
std::string Color_Dock;  
std::string Status_Dock; 
char Task6_str[2];

//Task 7
std::string Color_Fling;  
std::string Status_Fling; 
char Task7_str[2];

//Task 8
std::string Item_state_str;
std::string UAV_state2_str;
char UAV_Task8[2];

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
        Task1_str[0] = 1 + '0';
    else if (AMS_state_str  == "Autonomous")
        Task1_str[0] = 2 + '0';
    else if (AMS_state_str =="Killed")
        Task1_str[0] = 3 + '0';
    if (UAV_state_str == "Stowed")
        Task1_str[1] = 1 + '0';
    else if (UAV_state_str == "Deployed")
        Task1_str[1] = 2+ '0';
    else if (UAV_state_str == "Faulted")
        Task1_str[1] = 3  + '0';
	
    ROS_INFO("%c",Task1_str[0]);
} // END OF AMS_status()
//======================TASK 2==================================================
// THIS FUNCTION: Gets the Acoustics gate and writes char variable 
// ACCEPTS: jetson/Acoustics_msg.msg
// RETURNS: (VOID)
// =============================================================================
void Acoustics_get_gate (const jetson::Acoustics_msg::ConstPtr& msg)
{
    Entry_gate = msg->Entrance.data;
    Task2_str[0] = Entry_gate + '0';
    Exit_gate = msg->Exit.data;
    Task2_str[1] = Exit_gate + '0';
} // END OFAcoustics_get_gate()
//======================TASK 3==================================================
// THIS FUNCTION: Gets Task 3 if it's completed or in progress 
// ACCEPTS: jetson/task_info.msg
// RETURNS: (VOID)
// =============================================================================
void task_state (const jetson::task_info::ConstPtr& msg)
{
    State = msg->state;
	
    if (State=="In Progress")
        Task3_str[0] = 1 + '0';
    else if (State =="Completed")
        Task3_str[0] = 2 + '0';
} // END OF task_state()

//======================TASK 4==================================================
// THIS FUNCTION: Gets what animals detected and the order, also get the locations in respect the USV
// ACCEPTS: jetson/NED_objects.msg
// RETURNS: (VOID)
// =============================================================================
void CC_animals_ned_update(const jetson::NED_objects::ConstPtr& object)
{
        int i = 0;
        animal_qty = object->quantity;
		
        Animal_str[0] = animal_qty + '0';
        for (int i = 0; i <object->quantity; i++)
        {
           Animal[i] = object->objects[i].header.frame_id;  // Getting array of animal names
           if (Animal[i] == "crocodile")
           {
           Animal_str[i]= 'C';
           }
           else if (Animal[i] == "turtle")
           {
               Animal_str[i] = 'T';
           }
           else if (Animal[i] == "platypus")
           {
              Animal_str[i] = 'P';
           }
        }
}  // END OF CC_animals_ned_update()

//======================TASK 5==================================================
// THIS FUNCTION: Gets the color of the light bouy
// ACCEPTS: jetson/zed2i_msg.msg
// RETURNS: (VOID)
// =============================================================================
void object_rec(const jetson::zed2i_msg::ConstPtr& msg)
{
    int i = 0;
           Object[i] = msg->objects[i].header.frame_id;  // Getting array of animal name. Based on on array location, the object is closer or farther from USV
           x_object_NED[i] = msg->objects[i].point.x;  // Getting x position of animals
           y_object_NED[i] = msg->objects[i].point.y;  // Getting y position of animals
    //std::cout<<Object[0];
	
    if (Object[0] == "Blue Light")
    {
        Cod_Seq[cnt] = 'B';
        black = 0;
        cnt++;
    }
    else if (Object[0] == "Red Light")
    {
        Cod_Seq[cnt] = 'R';
        black = 0;
        cnt++;
    }
    else if (Object[i] == "Green Light")
    {
        Cod_Seq[cnt] = 'G';
        black = 0;
        cnt++;
    }
    else if (Object[i] == "Black Light")
    {
        black++;
    }
} // END OF object_rec()
//======================TASK 6==================================================
// THIS FUNCTION: Gets what dock we are docking and the status
// ACCEPTS: jetson/Detect_Dock_Fling.msg
// RETURNS: (VOID)
// =============================================================================
void Dock_detect(const jetson::Detect_Dock_Fling::ConstPtr& msg)
{
	
    Color_Dock= msg->Color;
    Status_Dock= msg->Status;

    if (Color_Dock == "red")
    {
        Task6_str[0] = 'R';
    }
    else if (Color_Dock== "blue")
        Task6_str[0] = 'B';
    else if (Color_Dock == "green")
        Task6_str[0] = 'G';
    if (Status_Dock == "Docking")
        Task6_str[1] = '1';
    else if (Status_Dock== "Complete")
        Task6_str[1] = '2';
	
} // END OF Dock()

//======================TASK 7==================================================
// THIS FUNCTION: Gets what dock we are flinging and the status
// ACCEPTS: jetson/Detect_Dock_Fling.msg
// RETURNS: (VOID)
// =============================================================================
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
//======================TASK 8==================================================
// THIS FUNCTION: Gets the UAV status and if it picked up the can or not
// ACCEPTS: jetson/AMS_state.msg
// RETURNS: (VOID)
// =============================================================================
void AMS2_status (const jetson::AMS_state::ConstPtr& msg)
{
    Item_state_str = msg->Item;
    UAV_state2_str = msg->UAV;

    if (UAV_state2_str == "Stowed")
        UAV_Task8[0] = 1 + '0';
    else if (UAV_state2_str == "Deployed")
        UAV_Task8[0] = 2 + '0';
    else if (UAV_state2_str == "Faulted")
        UAV_Task8[0] = 3 + '0';
	
        if (Item_state_str == "Not Picked Up")
        UAV_Task8[1] = 1 + '0';
    else if (Item_state_str == "Picked Up")
        UAV_Task8[1] = 2 + '0';
    else if (Item_state_str == "Delivered")
        UAV_Task8[1] = 3 + '0';
} // END OF AMS_status()

//======================MAIN=====================================================
int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Heartbeat_Control");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(1);
//Subscribers
    ros::Subscriber AMS_status_sub = n.subscribe("AMS_status", 1, AMS_status);					//Obtains what is the current AMS state
	ros::Subscriber Task_info_sub = n.subscribe("task_info", 1,Task_Control);							//Obtains current AMS task
	ros::Subscriber Acoustics_get_sub = n.subscribe("Acoustics_gate", 1, Acoustics_get_gate);						//Obtains what is the current AMS state 
	ros::Subscriber Task3_info = n.subscribe("task_info", 1, task_state);						//Obtains if task 3 is in progress or completed
	ros::Subscriber CC_animals_ned_sub = n.subscribe("CC_animals_ned", 1,CC_animals_ned_update );						//Obtains what is the current AMS state 
	ros::Subscriber obj_rec_sub = n.subscribe("obj_rec", 1, object_rec);						//Obtains what object the camera recognise, pose of the object and order of recognition
	ros::Subscriber Detect_Dock_Fling_sub = n.subscribe("Det_Dock", 1, Dock_detect);						//Obtains what object the camera recognise, pose of the object and order of recognition
	ros::Subscriber Detect_Fling_sub = n.subscribe("Det_Dock", 1, Dock_Fling);						//Obtains what object the camera recognise, pose of the object and order of recognition
    ros::Subscriber AMS2_status_sub = n.subscribe("AMS_status", 1, AMS2_status);					//Obtains what is the current AMS state
	
  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	if (Task == 1)
	{
        ROS_INFO("%c,%c",Task1_str[0],Task1_str[1]);
	}
	
	else if (Task == 2)
	{
        ROS_INFO("%c,%c",Task2_str[0],Task2_str[1]);
	}
	
	else if (Task == 3)
	{
        ROS_INFO("%c",Task3_str[0]);
	}
	
	else if (Task == 4)
	{
        ROS_INFO("%c,%c,%c,%c",Animal_str[0], Animal_str[1],Animal_str[2], Animal_str[3]);
	}
	
	else if (Task == 5)
	{
		ROS_INFO("Task 5");
		if (black >= 3)
		{
			for (int i = 0; i<cnt; i++)
			{
				std::cout<<Cod_Seq[i];
			}
			black = 0;
		}
	}
	
	else if (Task == 6)
	{
        ROS_INFO("%c,%c",Task6_str[0],Task6_str[1]);
	}
	
	else if (Task == 7)
	{
        ROS_INFO("%c,%c",Task7_str[0],Task7_str[1]);
	}
	
	else if (Task == 8)
	{
        ROS_INFO("%c,%c",UAV_Task8[0],UAV_Task8[1]);
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



