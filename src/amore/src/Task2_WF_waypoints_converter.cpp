// Filename:                         Task2_WF_waypoints_converter.cpp
// Creation Date:                 02/02/2022
// Last Revision Date:         03/25/2022
// Author(s) [email]:			    Brad Hacker [bhacker@lssu.edu]
// Revisor(s) {Date}:        	
// Organization/Institution:	Lake Superior State University

//...................................................About Task2_WF_waypoints_converter.cpp.....................................................................
// This file is used to subscribe to "/vrx/wayfinding/waypoints" to collect the array of poses to reach for VRX Task 2. 
// The poses are printed to "/vrx/wayfinding/waypoints" in longitude, latitude, and quarternion. These poses are then converted
// to be in NED convention relative to the user placed NED global frame. This code then publishes the poses to "waypoints_NED"
// in xNED, yNED, psiNED.

// Inputs and Outputs of the waypoints_converter.cpp file
//				Inputs: /vrx/wayfinding/waypoints
//				Outputs: waypoints_NED


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"

#include "geographic_msgs/GeoPath.h"								// message type published by VRX Task 2
#include "amore/NED_waypoints.h"										// message created to hold an array of the converted WF goal waypoints w/ headings and the number of goal poses in the array
#include "geometry_msgs/Point.h"										// message type used to hold the goal waypoints w/headings

#include "vrx_gazebo/Task.h"												// message published by VRX detailing current task and state
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants..................................................................
#define PI 3.14159265
#define CONFIRM 15 // count of loops to hold the same data point for before sending next pose to convert
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
int loop_count = 0;                                    			// loop counter, first 10 loops used to intitialize subscribers

double latitude, longitude, altitude;									// ECEF position variables in spherical coordinates
float qx, qy, qz, qw;														// 
float omega_x, omega_y, omega_z;								// angular velocities
float vx, vy, vz;																// linear velocities

double xNED, yNED, zNED;			        						// NED position
float q1NED, q2NED, q3NED, q0NED;
float phiNED, thetaNED, psiNED;
float omega_xNED, omega_yNED, omega_zNED;		// angular velocities
float vxNED, vyNED, vzNED;										// linear velocities

int WF_loops_sent = 0;                                  																// count of loops to hold the same data point for before sending next
int WF_point_num = 0;                                               														// used to keep track of the point being converted
int WF_goal_poses_quantity = -1;                                   												// used to keep track of the number of goal poses to convert

float WF_qx_goal[100], WF_qy_goal[100], WF_qz_goal[100], WF_qw_goal[100];
float WF_goal_lat[100], WF_goal_long[100];
float WF_x_NED[100], WF_y_NED[100], WF_psi_NED[100];

bool WF_point_sent = false;              																			// if WF_point_sent = false, the current point has not been published to nav_odom for conversion
bool WF_goal_lat_long_acquired = false;                  													// if WF_goal_lat_long_acquired = false, goal waypoint poses in lat and long coordinates have not been acquired
bool WF_waypoints_converted = false;     																	// if WF_waypoints_converted = false, goal waypoints in NED have not been converted
bool WF_waypoints_published = false;     																	// if WF_waypoints_published = false, goal waypoints in NED have not been published
bool WF_conv = true; 																									// WF_conv = true means the WF waypoint converter is being used, ######## should start false
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// this function subscribes to the goal position in lat and long published by the station_keeping task
void WF_goal_pose_sub(const geographic_msgs::GeoPath::ConstPtr& goal)
{
	ROS_INFO("CHECK IF.");
	if ((!WF_goal_lat_long_acquired) && (WF_conv))
	{
		ROS_INFO("ENTERING IF.");
		for (int i = 0; i < (int)sizeof(goal->poses)/8; i++)
		{
			WF_goal_lat[i] = goal->poses[i].pose.position.latitude;
			WF_goal_long[i] = goal->poses[i].pose.position.longitude;
			WF_qx_goal[i] = goal->poses[i].pose.orientation.x;
			WF_qy_goal[i] = goal->poses[i].pose.orientation.y;
			WF_qz_goal[i] = goal->poses[i].pose.orientation.z;
			WF_qw_goal[i] = goal->poses[i].pose.orientation.w;
		}
		WF_goal_lat_long_acquired = true;
		// UPDATES STATUSES TO USER ///////////////////////////////////////////////
		ROS_INFO("GOAL POSITIONS HAVE BEEN ACQUIRED FROM THE VRX TASK 2 WAYPOINTS NODE.");
		ROS_INFO("Size of array: %i", (int)sizeof(goal->poses)/8);
		WF_goal_poses_quantity = (int)sizeof(goal->poses)/8;
	}
	else
	{
		ROS_INFO("NOT ENTERING IF.");
	}
} // end of WF_goal_pose_sub()

void WF_GPS_Position()
{
	latitude = WF_goal_lat[WF_point_num];
	longitude = WF_goal_long[WF_point_num];
	altitude = 0.0; 														//sets altitude to 0
} // end of WF_GPS_Position()

void WF_GPS_Velocity() 
{
	// set velocity to zero since it isn't pertinent
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
} // end of WF_GPS_Velocity()

 void WF_IMU_processor()
{
	// set orientation quaternion
	qx = WF_qx_goal[WF_point_num];
	qy = WF_qy_goal[WF_point_num];
	qz = WF_qz_goal[WF_point_num];
	qw = WF_qw_goal[WF_point_num];
	
	// set body-fixed angular velocity to zero since it isn't pertinent
	omega_x = 0.0;
	omega_y = 0.0;
	omega_z = 0.0;
} // end of WF_IMU_processor()

void NED_Func(const nav_msgs::Odometry::ConstPtr& enu_state)
{
	// Variables
	float q1, q2, q3, q0, phi, theta, psi;
	
	// Convert the position to NED from ENU
	xNED = enu_state->pose.pose.position.y;
	yNED = enu_state->pose.pose.position.x;
	zNED = -(enu_state->pose.pose.position.z);
	// Convert from quaternion into radians
	q1 = enu_state->pose.pose.orientation.x;
	q2 = enu_state->pose.pose.orientation.y;
	q3 = enu_state->pose.pose.orientation.z;
	q0 = enu_state->pose.pose.orientation.w;
	phi = atan2((2.0*(q1*q0 + q3*q2)) , (1.0 - 2.0*(pow(q1,2.0) + pow(q2,2.0)))); 
	theta = asin(2.0*(q0*q2 - q1*q3));
	psi = atan2((2.0*(q3*q0 + q1*q2)) , (1.0 - 2.0*(pow(q2,2.0) + pow(q3,2.0)))); // orientation off x-axis
	// Convert the orientation to NED from ENU
	phiNED = theta;
	thetaNED = phi;
	psiNED = PI/2.0 - psi;
	// Adjust psiNED back within -PI and PI
	if (psiNED < -PI)
	{
		psiNED = psiNED + 2.0*PI;
	}
	if (psiNED > PI)
	{
		psiNED = psiNED - 2.0*PI;
	}
	// Convert the linear velocity to NED from NWU
	vxNED = vx;
	vyNED = -vy;
	vzNED = -vz;
	// Convert the angular velocity to NED
	
} // end of NED_Func()

void update_task(const vrx_gazebo::Task::ConstPtr& msg)
{
	// if the task is wayfinding and the state is ready or running and the waypoints haven't been published, converter will run
	if ((msg->name == "wayfinding") && (!WF_waypoints_published) && ((msg->state == "ready") || (msg->state == "running")))
	{
		WF_conv = true;
		ROS_INFO("WF POINT CONVERTER ON");
	}
	else
	{
		WF_conv = false;
		//ROS_INFO("WF POINT CONVERTER OFF");
	}	
}
//............................................................End of Functions............................................................

//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "T2_WAYPOINT_CONVERTER");
	
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	
	// NodeHandles
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
	ros::NodeHandle nh3;
	ros::NodeHandle nh4;
	ros::NodeHandle nh5;
	ros::NodeHandle nh6;
	ros::NodeHandle nh7;
	ros::NodeHandle nh8;
	ros::NodeHandle nh9;
	ros::NodeHandle nh10;
  
	// Subscribers
	ros::Subscriber task_status = nh1.subscribe("/vrx/task/info", 1, update_task);
	ros::Subscriber WF_waypoints_sub = nh2.subscribe("/vrx/wayfinding/waypoints", 100, WF_goal_pose_sub);                         			// subscriber for goal waypoints given by Task2_WF
	ros::Subscriber ned_sub = nh3.subscribe("geonav_odom", 1000, NED_Func);
	
	// Publishers
	ros::Publisher usvstate_pub = nh7.advertise<nav_msgs::Odometry>("nav_odom", 1000);   										// USV state publisher
	ros::Publisher WF_waypoints_pub = nh8.advertise<amore::NED_waypoints>("waypoints_NED", 100); 							// WF goal poses converted to NED publisher
	ros::Publisher WF_Converter_status_pub = nh9.advertise<std_msgs::Bool>("WF_Converter_status", 1);                 // WF_Converter_status publisher
	ros::Publisher WF_geonav_transform_status_pub = nh10.advertise<std_msgs::Bool>("WF_geonav_transform_status", 1);                 // WF_geonav_transform_status publisher
	
	// Local variables
	nav_msgs::Odometry nav_odom;
	std_msgs::Bool WF_publish_status, WF_geonav_transform_status;
	amore::NED_waypoints WF_NED_waypoints;
	
	// Initialize simulation time
	ros::Time::init();
	ros::Time current_time, last_time;  // creates time variables
	current_time = ros::Time::now();   // sets current time to the time it is now
	last_time = ros::Time::now();        // sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(100); // {Hz} THIS IS TOUCHY BUSINESS


	while(ros::ok())
	{
		current_time = ros::Time::now();
		
		// publish whether or not the WF goal waypoints converter has published the waypoints for the WF planner
		if (!WF_waypoints_published)
		{
		  WF_publish_status.data = false;
		}
		else
		{
		  WF_publish_status.data = true;
		}
		WF_Converter_status_pub.publish(WF_publish_status);
		
		// publish whether or not this code is using geonav transform package
		if (!WF_conv)
		{
		  WF_geonav_transform_status.data = false;
		}
		else
		{
		  WF_geonav_transform_status.data = true;
		}
		WF_geonav_transform_status_pub.publish(WF_geonav_transform_status);
		
		if ((!WF_waypoints_converted) && (WF_goal_lat_long_acquired) && (WF_conv))
		{
			if (!WF_point_sent)
			{
				// UPDATE NAV_ODOM MSG
				WF_GPS_Position();
				WF_GPS_Velocity();
				WF_IMU_processor();
				// Fill the odometry header for nav_odom, the USV state in ENU and NWU
				nav_odom.header.seq +=1;								// sequence number
				nav_odom.header.stamp = current_time;				// sets stamp to current time
				nav_odom.header.frame_id = "odom";					// header frame
				nav_odom.child_frame_id = "base_link";				// child frame
			
				// Fill the USV pose
				nav_odom.pose.pose.position.x = longitude; //sets long
				nav_odom.pose.pose.position.y = latitude; //sets lat
				nav_odom.pose.pose.position.z = altitude; //sets altitude
				nav_odom.pose.pose.orientation.x = qx;
				nav_odom.pose.pose.orientation.y = qy;
				nav_odom.pose.pose.orientation.z = qz;
				nav_odom.pose.pose.orientation.w = qw;
				nav_odom.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
				// Fill the USV velocities
				nav_odom.twist.twist.linear.x = vx;
				nav_odom.twist.twist.linear.y = vy;
				nav_odom.twist.twist.linear.z = vz;
				nav_odom.twist.twist.angular.x = omega_x;
				nav_odom.twist.twist.angular.y = omega_y;
				nav_odom.twist.twist.angular.z = omega_z;
				nav_odom.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
				usvstate_pub.publish(nav_odom);
				WF_point_sent = true;
			}
			
			// Update waypoint array in NED units
			if (WF_point_num < WF_goal_poses_quantity)
			{
				WF_x_NED[WF_point_num] = xNED;
				WF_y_NED[WF_point_num] = yNED;
				WF_psi_NED[WF_point_num] = psiNED;
				
				WF_loops_sent += 1;
				if (WF_loops_sent == CONFIRM)
				{
					WF_loops_sent = 0;								// reset WF_loops_sent counter
					WF_point_num += 1;
					WF_point_sent = false;
				}
			}
			if (WF_point_num == WF_goal_poses_quantity) 							// means all coordinates have been converted
			{
				WF_waypoints_converted = true;
				ROS_INFO("WAYPOINTS HAVE BEEN CONVERTED");
			}
		} // if ((!WF_waypoints_converted) && (WF_goal_lat_long_acquired) && (WF_conv))
		
	    if ((WF_waypoints_converted) && (!WF_waypoints_published))
		{
			WF_NED_waypoints.points.clear();
			
			WF_NED_waypoints.quantity = WF_goal_poses_quantity;        // publish quantity of poses so the high level control knows
			
			for (int i = 0; i < WF_goal_poses_quantity; i++)
			{
				geometry_msgs::Point point;
				point.x = WF_x_NED[i];
				point.y = WF_y_NED[i];
				point.z = WF_psi_NED[i];
				WF_NED_waypoints.points.push_back(point);
				ROS_INFO("point: %i", i);
				ROS_INFO("x: %f", WF_x_NED[i]);
				ROS_INFO("y: %f", WF_y_NED[i]);
				ROS_INFO("psi: %f", WF_psi_NED[i]);
			}
			WF_waypoints_pub.publish(WF_NED_waypoints);
			WF_waypoints_published = true;
			ROS_INFO("WAYPOINTS HAVE BEEN PUBLISHED");
		} // if ((WF_waypoints_converted) && (!WF_waypoints_published))
		
		if (WF_waypoints_published)
		{
			WF_waypoints_pub.publish(WF_NED_waypoints);
		}
		
		last_time = current_time;
		ros::spinOnce();
		loop_rate.sleep();
		loop_count = loop_count + 1;
	}
	return 0;
} // end of main()
