//  Filename:						        High_Level_trajectory_processor.cpp
//  Creation Date:						2/2/2022
//  Last Revision Date:                2/18/2022
//  Author(s) [email]:					Brad Hacker [bhacker@lssu.edu]
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................About Task2_WF_Planner.cpp.......................
//  This code subscribes to the waypoints printed by task 2 for VRX. This trajectory processor 
//   uses mid level control such as acceptance radius to feed the trajectory to the low level controller. 
//
//  Inputs and Outputs of the Task2_WF_Planner.cpp file
//				Inputs [subscribers]: waypoints in lat and long from VRX Task 1
//				Outputs [publishers]: goal pose for SK controller to subscribe to

// Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geographic_msgs/GeoPath.h"
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "time.h"

#include "geometry_msgs/Point.h"
#include "amore/NED_waypoints.h"

// include necessary message libraries
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

// DEFINING GLOBAL VARIABLES
#define PI 3.14159265

// Global Variables ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int loop_count=0;            // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
int point=0;                      // used to keep track of the number of points on trajectory reached 
int goal_poses;               // total number of points to reach 

float x_goal[100], y_goal[100], psi_goal[100];  // array to hold the NED goal poses

// for holding vehicle position and heading in NED
float x_usv_NED, y_usv_NED, psi_NED;

float ON_OFF = 0.0;      // if 0.0, the controller is OFF, if it is a 1.0, the controller is ON

// error variables used to calculate magnitude of pose error
float e_x, e_y, e_xy;

bool goal_recieved = false;   // if goal_recieved = false, goal position has not been acquired from "mpp_goal" yet
int loop_goal_recieved;          // this is kept in order to ensure planner doesn't start until sytem is through initial startup
bool point_reached = false;   // if point_reached is false this means the current point has not been reached
int point_reached_loop = -1; // keeps track of which loop the point is reached
bool E_reached = false;        // if E_reached is false this means the last point has not been reached
bool waypoints_published = false;        // if waypoints_published is false this means the NED poses have not yet been calculated and published 

//..................................................................Functions.................................................................
// this function subscribes to the WF_Converter_status node to see when goal waypoints have been converted
void goal_publish_check(const std_msgs::Bool published) 
{
	if (published.data)
	{
		waypoints_published = true;
	}
	else
	{
		waypoints_published= false;
	}
}

void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	if (goal_recieved)
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

void goal_update(const amore::NED_waypoints::ConstPtr& goal) 
{
	goal_poses = goal->quantity;
	if ((loop_count > 25) &&  (!goal_recieved) && (waypoints_published))
	{
		for (int i = 0; i < goal_poses; i++)
		{
			x_goal[i] = goal->points[i].x;
			y_goal[i] = goal->points[i].y;
			psi_goal[i] = goal->points[i].z;
		}
		goal_recieved = true;
		
		// UPDATES STATUSES TO USER ///////////////////////////////////////////////
		ROS_INFO("GOAL POSITIONS HAVE BEEN ACQUIRED BY THE WF PLANNER.");
		ROS_INFO("Size of x_goal array: %i", goal_poses);
	}
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "SK Path Planner");
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  
  // set up NodeHandles
  ros::NodeHandle nh2;  // subscriber to usv_ned which provides pose in NED
  ros::NodeHandle nh3;  // subscriber to waypoints_NED which provides goal waypoints in NED
  ros::NodeHandle nh4;
  ros::NodeHandle nh6;  // publisher to mpp_goal
  
  // start publishers and subscribers
  ros::Subscriber ned_sub = nh2.subscribe("usv_ned", 1, pose_update);                                                               // subscriber for current position converted to NED
  ros::Subscriber waypoints_ned_sub = nh3.subscribe("waypoints_NED", 1, goal_update);                                 // subscriber for goal waypoints converted to NED
  ros::Subscriber WF_Converter_sub = nh4.subscribe("WF_Converter_status", 1, goal_publish_check);           // subscriber for whether or not goal waypoints have been converted yet
  ros::Publisher mpp_goal_pub = nh6.advertise<nav_msgs::Odometry>("mpp_goal", 1);                                    // publisher for current goal for low level controller
  
  // local variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  nav_msgs::Odometry nav_odom; // mpp_goal position
  std_msgs::Bool publish_status;     // SK_Planner_status
  ros::Time current_time, last_time;  // creates time variables
  current_time = ros::Time::now();   // sets current time to the time it is now
  last_time = ros::Time::now();        // sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(100);
  ros::spinOnce();                             // initialize position
  loop_rate.sleep();  

  // ros::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  current_time = ros::Time::now(); // update time
	  
	  if ((goal_recieved) && (loop_count > loop_goal_recieved+1000))
	  {
		  // determine error in x and y (position)
		  e_x = x_goal[point] - x_usv_NED;                                       // calculate error in x position
		  e_y = y_goal[point] - y_usv_NED;                                       // calculate error in y position
		  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
		  // ADD E_PSI
		  if ((e_xy < 0.5) && (!E_reached))
		  {
			  point = point + 1;
			  ROS_INFO("Point %i reached.", point);
			  if (point==goal_poses)
			  {
				  E_reached = true;
				  ROS_INFO("End point has been reached.\n");
			  }
		  }
	
		  if (E_reached)
		  {
			  // once exit point is reached, the current position is fed
			  x_goal[point] = x_usv_NED;
			  y_goal[point] = y_usv_NED;
			  psi_goal[point] = psi_NED;
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
		  nav_odom.pose.pose.position.x = x_goal[point];                    // sets x-location
		  nav_odom.pose.pose.position.y = y_goal[point];                    // sets y-location
		  nav_odom.pose.pose.position.z = ON_OFF; // use this to shut off low-level controller by sending a 0.0
		  nav_odom.pose.pose.orientation.x = 0.0;
		  nav_odom.pose.pose.orientation.y = 0.0;
		  nav_odom.pose.pose.orientation.z = psi_goal[point];
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
	  } // if ((goal_recieved) && (loop_count > loop_goal_recieved+10))
	  
      // DEBUG INFO
	  /* ROS_INFO("ON_OFF: %.1f", ON_OFF);
	  if (goal_recieved)
	  {
		  ROS_INFO("goal recieved!");
	  }
	  else
	  {
		  ROS_INFO("goal NOT recieved!");
	  } */
	  
	  last_time = current_time; 
  	  ros::spinOnce();
	  loop_rate.sleep();
	  loop_count = loop_count + 1;
  } // while(ros::ok() && !goal)

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}