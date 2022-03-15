//  Filename:						        High_Level_trajectory_processor.cpp
//  Creation Date:						1/31/2022
//  Last Revision Date:                
//  Author(s) [email]:					Brad Hacker [bhacker@lssu.edu]
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................About Task1_SK_Planner.cpp.......................
//  This code subscribes to the trajectory publisher and feeds the path to the low level controllers. This trajectory processor 
//   uses mid level control such as acceptance radius to feed the trajectory to the low level controller. 
//
//  Inputs and Outputs of the Task1_SK_Planner.cpp file
//				Inputs [subscribers]: goal pose in lat and long from VRX Task 1
//				Outputs [publishers]: goal pose for SK controller to subscribe to

// Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "time.h"
#include "vrx_gazebo/Task.h"

// include necessary message libraries
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

// DEFINING GLOBAL VARIABLES
#define PI 3.14159265

// Global Variables ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int loop_count=0;            // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
float x_goal;
float y_goal;
float psi_goal;
// for getting vehicle position and heading
float x_usv_NED;           // x-position in NED
float y_usv_NED;           // y-position in NED
float psi_NED;                // WAM-V z-orientation in NED

float ON_OFF = 0.0;         // if this variable is a 0.0, the controller is OFF, if it is a 1.0, the controller is ON

float e_x;                        // error between the desired x-pos. and the actual x-pos.
float e_y;                        // error between the desired y-pos. and the actual y-pos.
float e_xy;                      // magnitude of position error

bool goal_recieved = false;  // if goal_recieved = false, goal position has not been acquired from "mpp_goal" yet
int loop_goal_recieved;          // this is kept in order to ensure planner doesn't start until sytem is through initial startup
bool point_reached = false;   // if point_reached is false this means the current point has not been reached
bool E_reached = false;        // if E_reached is false this means the last point has not been reached
bool function = false;                           // if convert = false, this means according to the current task status, conversion shouldn't be done

void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	if ((!goal_recieved) && (loop_count>20))
	{
		// Collect coordinates in NED
		x_goal = odom->pose.pose.position.x;
		y_goal = odom->pose.pose.position.y;
		psi_goal = odom->pose.pose.orientation.z;
	
		// adjust current heading back within -PI and PI
		if (psi_goal < -PI)
		{
			psi_goal = psi_goal + 2.0*PI;
		}
		if (psi_goal > PI)
		{
			psi_goal = psi_goal - 2.0*PI;
		}
		goal_recieved = true;
		loop_goal_recieved = loop_count;
	}
	else if (goal_recieved)
	{
		// Collect coordinates in NED
		x_usv_NED = odom->pose.pose.position.x;
		y_usv_NED = odom->pose.pose.position.y;
		psi_NED = odom->pose.pose.orientation.z;
	
		// adjust current heading back within -PI and PI
		if (psi_NED < -PI)
		{
			psi_NED = psi_NED + 2.0*PI;
		}
		if (psi_NED > PI)
		{
			psi_NED = psi_NED - 2.0*PI;
		}
	}
}

void update_task(const vrx_gazebo::Task::ConstPtr& msg)
{
	if (((msg->name == "station_keeping") && (msg->state == "ready")) || ((msg->name == "station_keeping") && (msg->state == "running")))
	{
		function = true;
	}
	else
	{
		function = false;
	}
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "SK Path Planner");
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  
  // set up NodeHandles
  ros::NodeHandle nh2;  // subscriber to usv_ned which provides pose in NED
  ros::NodeHandle nh5;  // publisher to SK_Planner_status
  ros::NodeHandle nh6;  // publisher to mpp_goal
  
  // start publishers and subscribers
  ros::Subscriber ned_sub = nh2.subscribe("usv_ned", 1, pose_update);                                                               // subscriber for current position converted to NED
  
  ros::Publisher SK_Planner_status_pub = nh5.advertise<std_msgs::Bool>("SK_Planner_status", 1);                // SK_Planner status publisher
  ros::Publisher mpp_goal_pub = nh6.advertise<nav_msgs::Odometry>("mpp_goal", 1);                                     // publisher for current goal for low level controller
  
  // local variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  nav_msgs::Odometry nav_odom; // mpp_goal position
  std_msgs::Bool publish_status;     // SK_Planner_status
  ros::Time current_time, last_time;  // creates time variables
  current_time = ros::Time::now();   // sets current time to the time it is now
  last_time = ros::Time::now();        // sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(4);
  ros::spinOnce();                             // initialize position
  loop_rate.sleep();  

  // ros::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  current_time = ros::Time::now(); // update time
	  
	  // publish whether or not the goal pose has been acquired
	  if (!goal_recieved)
	  {
		  publish_status.data = false;
	  }
	  else
	  {
		  publish_status.data = true;
	  }
	  SK_Planner_status_pub.publish(publish_status);
	  
	  if ((goal_recieved) && (loop_count > loop_goal_recieved+10) && (function))
	  {
		  // determine error in x and y (position)
		  e_x = x_goal - x_usv_NED;                                                // calculate error in x position
		  e_y = y_goal - y_usv_NED;                                                // calculate error in y position
		  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                           // calculate magnitude of positional error
		  if ((e_xy < 0.02) && (!E_reached))
		  {
			  E_reached = true;
			  ROS_INFO("e_xy: %.2f", e_xy);
			  ROS_INFO("End point has been reached.\n");
		  }
	
		  if (E_reached)
		  {
			  // once exit point is reached, the current position is fed
			  x_goal = x_usv_NED;
			  y_goal = y_usv_NED;
			  ON_OFF = 0.0; // controller is OFF once exit point is reached
		  }
		  else
		  {
			  ON_OFF = 1.0; // controller is ON until exit point is reached
		  }
	
		  // fill in message with values that are unnecesary
		  nav_odom.header.seq +=1;                                          // sequence number
		  nav_odom.header.stamp = current_time;                    // sets stamp to current time
		  nav_odom.header.frame_id = "odom";                         // header frame
		  nav_odom.child_frame_id = "base_link";                      // child frame
		  nav_odom.pose.pose.position.x = x_goal;                    // sets x-location
		  nav_odom.pose.pose.position.y = y_goal;                    // sets y-location
		  nav_odom.pose.pose.position.z = ON_OFF; // use this to shut off low-level controller by sending a 0.0
		  nav_odom.pose.pose.orientation.x = 0.0;
		  nav_odom.pose.pose.orientation.y = 0.0;
		  nav_odom.pose.pose.orientation.z = psi_goal;
		  nav_odom.pose.pose.orientation.w = 0.0;
		  nav_odom.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		  nav_odom.twist.twist.linear.x = 0.0; //sets velocity values all to zero
		  nav_odom.twist.twist.linear.y = 0.0;
		  nav_odom.twist.twist.linear.z = 0.0;
		  nav_odom.twist.twist.angular.x = 0.0;
		  nav_odom.twist.twist.angular.y = 0.0;
		  nav_odom.twist.twist.angular.z = 0.0;
		  nav_odom.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		  mpp_goal_pub.publish(nav_odom); // publishes desired pose to mpp_goal node
	  } // if (goal_recieved)
		  
	  ROS_INFO("ON_OFF: %.1f", ON_OFF);
	  if (publish_status.data)
	  {
		  ROS_INFO("goal recieved!");
	  }
	  else
	  {
		  ROS_INFO("goal NOT recieved!");
	  }
	  
	  last_time = current_time; 
  	  ros::spinOnce();
	  loop_rate.sleep();
	  loop_count = loop_count + 1;
  } // while(ros::ok() && !goal)

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}
