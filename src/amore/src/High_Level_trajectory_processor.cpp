//  Filename:						        High_Level_trajectory_processor.cpp
//  Creation Date:						1/12/2022
//  Last Revision Date:                
//  Author(s) [email]:					Brad Hacker [bhacker@lssu.edu]
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................About High_Level_trajectory_processor.cpp.......................
//  This code subscribes to the trajectory publisher and feeds the path to the low level controllers. This trajectory processor 
//   uses mid level control such as acceptance radius to feed the trajectory to the low level controller. 
//
//  Inputs and Outputs of the High_Level_trajectory_processor.cpp file
//				Inputs [subscribers]: trajectory from traj_pub
//				Outputs [publishers]: Positions for low level controller to subscribe to

// Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "time.h"

// include necessary message libraries
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

// DEFINING GLOBAL VARIABLES
#define PI 3.14159265

// Global Variables ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int loop_count=0;            // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
int point=0;                      // used to keep track of the number of points on trajectory reached 
float x_goal;
float y_goal;
float traj_XY[500][2];
// for getting vehicle position and heading
float x_usv_ENU;           // x-position in ENU in real time by transfering lat and long
float y_usv_ENU;           // y-position in ENU in real time by transfering lat and long
float x_usv_NED;           // x-position in NED
float y_usv_NED;           // y-position in NED
// q0 - q3 are current orientation in quaternion form
float q1;
float q2;
float q3;
float q0;
float psi_ENU;                // WAM-V z-orientation in ENU in real time through the compass
float psi_NED;                // WAM-V z-orientation in NED

float ON_OFF = 0.0;         // if this variable is a 0.0, the controller is OFF, if it is a 1.0, the controller is ON

float e_x;                        // error between the desired x-pos. and the actual x-pos.
float e_y;                        // error between the desired y-pos. and the actual y-pos.
float e_xy;                      // magnitude of position error

bool traj_got = false; // if traj_got is true, this means the trajectory has been aquired
bool point_reached = false; // if point_reached is false this means the current point has not been reached
int point_reached_loop = -1; // keeps track of which loop the point is reached
bool E_reached = false; // if E_reached is false this means the last point has not been reached

void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	x_usv_ENU = odom->pose.pose.position.x;
	y_usv_ENU = odom->pose.pose.position.y;
	q1 = odom->pose.pose.orientation.x;
	q2 = odom->pose.pose.orientation.y;
	q3 = odom->pose.pose.orientation.z;
	q0 = odom->pose.pose.orientation.w;
	
	// convert current heading from quaternion into radians
	psi_ENU = atan2((2.0*(q3*q0 + q1*q2)) , (1 - 2*(pow(q2,2.0) + pow(q3,2.0)))); // orientation off x-axis
	
	// convert from ENU to NED
    x_usv_NED = y_usv_ENU;
	y_usv_NED = x_usv_ENU;
	psi_NED = PI/2.0 - psi_ENU;
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

/* void get_traj(const nav_msgs::Odometry::ConstPtr& traj)
{
	// only get trajectory from traj_pub once
	if (!traj_got)
	{
		traj_XY[traj->header.seq - 1][0] = traj->pose.pose.position.x;
		traj_XY[traj->header.seq - 1][1] = traj->pose.pose.position.y;
		traj_got = true;
	}
} */

void gen_traj()
{
	// Variables
	int i;					// for indexing
	float x = 0.0;		// holds desired x-value
	float y = 0.0;		// holds desired y-value
	
	if (!traj_got)
	{
		for(i=0; i<100; i++)
		{
			// .....Straight lines.....
			// y = 20..........................................
			x = 20.0;
			y = y + (float)i;
			/* // y = x..........................................
			x = (float)i;
			y = x; */
			/* // y = (-3/13)x.................................
			x = (float)i;
			y = -(3.0/13.0)*x; */
			
			// .....Sinusoids.....
			/* // y = sin(x/2)..................................
			x = (float)i;
			y = sin(x/2.0); */
			
			// .....Power Functions.....
			/* // y = 2^x - 1...................................
			x = (float)i;
			y = pow(2.0,x) - 1.0; */
			/* // y = 2^x.......................................
			x = (float)i;
			y = pow(2.0,x); */
			
			// .....Other Curves.....
			/* // y = x^(-x) + (0.7)x.......................
			x = (float)i;
			y = pow(x,-x) + (0.7)*x; */
			
			traj_XY[i][0] = x;
			traj_XY[i][1] = y;
		} // end of for-loop
	}
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "Path Planner");
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  // set up NodeHandles
  ros::NodeHandle nh1;  // subscriber to geonav_odom which provides current position in x, y
  //ros::NodeHandle nh2;  // subscriber to goal_pos
  ros::NodeHandle nh6;  // publisher to mpp_goal
  
  // start publishers and subscribers
  ros::Subscriber nav_sub = nh1.subscribe("geonav_odom", 1, pose_update);  // subscribes to geonav_odom to get USV pose in global frame
  //ros::Subscriber traj_sub = nh2.subscribe("goal_pos", 1000, get_traj);  // subscribes to goal_pos to get goal position of USV over time
  ros::Publisher mpp_goal_pub = nh6.advertise<nav_msgs::Odometry>("mpp_goal", 10); //publishes current goal for low level controller
  
  // local variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  nav_msgs::Odometry nav_odom; // mpp_goal position
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
	if (!traj_got)
	{
		gen_traj();
		traj_got = true;
	}
	
	current_time = ros::Time::now(); // update time
	
	if ((loop_count > 20)&&(traj_got))
	{
	  x_goal = traj_XY[point][0];
	  y_goal = traj_XY[point][1];
	  // determine error in x and y (position)
	  e_x = x_goal - x_usv_NED;                                                // calculate error in x position
	  e_y = y_goal - y_usv_NED;                                                // calculate error in y position
	  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                           // calculate magnitude of positional error
	  
	  if (e_xy < 5.0)
	  {
		point += 1;
		ROS_DEBUG("Point %i reached.\n", point );
	  }
	  if ((!E_reached) && (point==100))
	  {
		E_reached = true;
		ROS_DEBUG("End point has been reached.\n");
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
	  nav_odom.pose.pose.orientation.z = 0.0;
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
	} // if (loop_count>20)
	
	last_time = current_time; 
	ros::spinOnce();
	loop_rate.sleep();
	loop_count = loop_count + 1;
  } // while(ros::ok() && !goal)

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}
