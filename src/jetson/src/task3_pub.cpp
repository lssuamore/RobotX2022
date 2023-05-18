//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "jetson/task_info.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Task3_Publisher");
  ros::NodeHandle n;
  
   //declare variables


  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare publisher "squarenum" is the name of the node
  //1 is the number of values to keep stored until they are overwritten
  ros::Publisher Task5_pub = n.advertise<jetson::task_info>("task_info", 1);

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




