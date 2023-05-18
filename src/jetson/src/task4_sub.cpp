//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/NED_objects.h"  // message type that has an array of pointstamped
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
//Global Variables
//Task 4 Variables
std::string Animal[3];  // string array to hold the names of the animals
char Animal_str[3] = {};; // String to send animals for heartbeat
int animal_qty = 0;

void CC_animals_ned_update(const jetson::NED_objects::ConstPtr& object)
{
	
		int i = 0;
		animal_qty = object->quantity;
		
		Animal_str[0] = animal_qty + '0';
		for (int i = 1; i <object->quantity+1; i++)
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
	ros::Subscriber CC_animals_ned_sub = n.subscribe("CC_animals_ned", 1,CC_animals_ned_update );						//Obtains what is the current AMS state 

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  ROS_INFO("Animal Quantity %c",Animal_str[0]);
	 
	 //std::cout<<animal_qty;
	 for (int i = 0; i < 4;i++)
	{
	 std::cout <<Animal_str[i];
	}
    //looks for data
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }

  return 0;
}



