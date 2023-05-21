#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"					// message type used for receiving NED USV state from navigation_array
#include "jetson/state_msg.h"					// message type used to recieve state of operation from mission_control
#include "std_msgs/Bool.h"					// message used to communicate publish state to propulsion_system
#include "jetson/usv_pose_msg.h"				// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "jetson/zed2i_msg.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
#include "sensor_msgs/NavSatFix.h"	

int main(int argc, char **argv)
{
//names the program for visual purposes
	ros::init(argc, argv, "Publishers");
ros::NodeHandle n;
//sets the frequency for which the program sleeps at. 10=1/10 second
ros::Rate loop_rate(10);
//declare variables
ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("Gps_update",1);

//rosk::ok() will stop when the user inputs Ctrl+C
while(ros::ok())
{
//clear the input buffer
std::fflush; //the two “f&#39;s” are correct, not a typo

ros::spinOnce();
loop_rate.sleep();
}
return 0;
}
