//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "jetson/zed2i_msg.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"
//Global Variables
std::string Object[9];  // string array to hold the names of the objects from the zed2i
float x_object_NED[9], y_object_NED[9];  // arrays to hold animal locations
char Cod_Seq[100];
int cnt = 0;
int black = 0;
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
	ros::Subscriber obj_rec_sub = n.subscribe("obj_rec", 1, object_rec);						//Obtains what object the camera recognise, pose of the object and order of recognition

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	if (black >= 3)
	{
		for (int i = 0; i<cnt; i++)
		{
			std::cout<<Cod_Seq[i];
			//ROS_INFO("%s",Cod_Seq[i].c_str());
		}
		black = 0;
		//cnt = 0;
	}
	
    //looks for data
    ros::spinOnce();					// update subscribers
	loop_rate.sleep();				// sleep for set loop_rate
  }

  return 0;
}



