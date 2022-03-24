// Filename:                         Task2_WF_waypoints_converter.cpp
// Creation Date:                 02/02/2022
// Last Revision Date:         02/02/2022
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
#include <vector>
#include "math.h"
#include "stdio.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geographic_msgs/GeoPath.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "tf/transform_broadcaster.h"

#include "geometry_msgs/Point.h"
#include "amore/NED_waypoints.h"
#include "vrx_gazebo/Task.h"
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants..................................................................
#define PI 3.14159265
#define CONFIRM 15 // count of loops to hold the same data point for before sending next
//............................................................End of Constants.............................................................

//.................................................................Structures..................................................................
struct Point {
	float x;
	float y;
};
//............................................................Structures.............................................................

//..............................................................Global Variables............................................................
int loop_count=0;                                     // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
int loop_confirm=0;                                  // count of loops to hold the same data point for before sending next
int point_num=0;                                               // used to keep track of the point being converted
int goal_poses=-1;                                   // used to keep track of the number of goal poses to convert
double latitude, longitude, altitude;			 // The ECEF position variables in spherical coordinates
float qx, qy, qz, qw;
float omega_x, omega_y, omega_z;		 // angular velocities
float vx, vy, vz;										 // linear velocities
double xNED, yNED, zNED;				     // The NED position
float x_NED[100], y_NED[100], psi_NED[100];			 // The NED poses
float q1NED, q2NED, q3NED, q0NED;
float phiNED, thetaNED, psiNED;
float omega_xNED, omega_yNED, omega_zNED;		// angular velocities
float vxNED, vyNED, vzNED;										// linear velocities
float qx_goal[100], qy_goal[100], qz_goal[100], qw_goal[100];
float goal_lat[100], goal_long[100];
float NED_POSES[100];                    // used to hold NED poses: x,y,psi
bool point_published = false;              // if point_published = false, the current point has not been published
bool goal_lat_long = false;                  // if goal_lat_long = false, goal waypoint poses in lat and long coordinates have not been acquired
bool waypoints_converted = false;     // if waypoints_converted = false, goal waypoints in NED have not been converted
bool waypoints_published = false;     // if waypoints_published = false, goal waypoints in NED have not been published
bool convert = true;                           // if convert = false, this means according to the current task status, conversion shouldn't be done ################################################################ should start false
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// this function subscribes to the goal position in lat and long published by the station_keeping task
void goal_pose_sub(const geographic_msgs::GeoPath::ConstPtr& goal)
{
	ROS_INFO("CHECK IF.");
	if ((!goal_lat_long) && (convert)) //(loop_count>5) && 
	{
		ROS_INFO("ENTERING IF.");
		for (int i = 0; i < (int)sizeof(goal->poses)/8; i++)
		{
			goal_lat[i] = goal->poses[i].pose.position.latitude;
			goal_long[i] = goal->poses[i].pose.position.longitude;
			qx_goal[i] = goal->poses[i].pose.orientation.x;
			qy_goal[i] = goal->poses[i].pose.orientation.y;
			qz_goal[i] = goal->poses[i].pose.orientation.z;
			qw_goal[i] = goal->poses[i].pose.orientation.w;
		}
		goal_lat_long = true;
		// UPDATES STATUSES TO USER ///////////////////////////////////////////////
		ROS_INFO("GOAL POSITIONS HAVE BEEN ACQUIRED FROM THE VRX TASK 2 WAYPOINTS NODE.");
		ROS_INFO("Size of array: %i", (int)sizeof(goal->poses)/8);
		goal_poses = (int)sizeof(goal->poses)/8;
	}
	else
	{
		ROS_INFO("NOT ENTERING IF.");
	}
}

void GPS_Position()
{
	latitude = goal_lat[point_num]; //sets latitude from gps
	longitude = goal_long[point_num]; //sets longitude from gps
	altitude = 0.0; //sets altitude to 0
} // end of GPS_Position()

void GPS_Velocity() 
{
	// gather the velocity, which is in NWU, wrt the GPS sensor (which I think is almost the USV origin)
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
} // end of GPS_Velocity()

 void IMU_processor()
{
	// gather orientation quaternion
	qx = qx_goal[point_num];
	qy = qy_goal[point_num];
	qz = qz_goal[point_num];
	qw = qw_goal[point_num];
	
	// gather body-fixed angular velocity
	omega_x = 0.0;
	omega_y = 0.0;
	omega_z = 0.0;
} // end of IMU_processor()

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
	if ((msg->name == "wayfinding") && (!waypoints_published) && ((msg->state == "ready") || (msg->state == "running")))
	{
		convert = true;
		ROS_INFO("WF POINT CONVERTER ON");
	}
	else
	{
		convert = false;
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
	
	// Node handles
	ros::NodeHandle nh3;
	ros::NodeHandle nh4;
	ros::NodeHandle nh5;
	ros::NodeHandle nh6;
	ros::NodeHandle nh7;
	ros::NodeHandle nh8;
	ros::NodeHandle nh9;
	ros::NodeHandle nh10;
  
	// Subscribers
	ros::Subscriber task_status = nh3.subscribe("/vrx/task/info", 1, update_task);
	ros::Subscriber ned_sub = nh4.subscribe("geonav_odom", 1000, NED_Func);
	ros::Subscriber pose_sub = nh5.subscribe("/vrx/wayfinding/waypoints", 100, goal_pose_sub);                         			// subscriber for goal pose given by Task2_WF
	
	// Publishers
	ros::Publisher waypoints_pub = nh7.advertise<amore::NED_waypoints>("waypoints_NED", 100); 							// NED waypoints publisher with Float64 []
	ros::Publisher usvstate_pub = nh8.advertise<nav_msgs::Odometry>("nav_odom", 1000);   										// USV state publisher
	ros::Publisher WF_Converter_status_pub = nh9.advertise<std_msgs::Bool>("WF_Converter_status", 1);                 // WF_Converter_status publisher
	ros::Publisher WF_geonav_transform_status_pub = nh10.advertise<std_msgs::Bool>("WF_geonav_transform_status", 1);                 // WF_geonav_transform_status publisher
	
	// Variables
	nav_msgs::Odometry nav_odom;
	geographic_msgs::GeoPath waypoints_NED;
	geographic_msgs::GeoPoseStamped pose_NED;
	std_msgs::Bool publish_status, geonav_transform_status;
	amore::NED_waypoints NED_waypoints;
	
	// creating the vector
	Point point_array[100];
	Point point;
	for (int i=0; i<100; i++)
	{
		point.x = i;
		point.y = i;
		point_array[i] = point;
	}
	std::vector<Point> point_vector (point_array, point_array + sizeof(point_array) / sizeof(Point));
	
	ros::Time current_time, last_time; 	// creates time variables
	last_time = ros::Time::now();      		// sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(100); // {Hz} THIS IS TOUCHY BUSINESS

	while(ros::ok())
	{
		current_time = ros::Time::now();
		
		// publish whether or not this code has published the waypoints
		if (!waypoints_published)
		{
		  publish_status.data = true;
		}
		else
		{
		  publish_status.data = false;
		}
		WF_Converter_status_pub.publish(publish_status);
		
		// publish whether or not this code is using geonav transform package
		if (!convert)
		{
		  geonav_transform_status.data = false;
		}
		else
		{
		  geonav_transform_status.data = true;
		}
		WF_geonav_transform_status_pub.publish(geonav_transform_status);
		
		if ((loop_count > 5) && (!waypoints_converted) && (goal_lat_long) && (convert))
		{
			// UPDATE NAV_ODOM MSG
			GPS_Position();
			GPS_Velocity();
			IMU_processor();
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
			
			if (!point_published)
			{
				usvstate_pub.publish(nav_odom);
				point_published = true;
			}
			
			// Update waypoint array in NED units
			if (point_num < goal_poses)
			{
				x_NED[point_num] = xNED;
				y_NED[point_num] = yNED;
				psi_NED[point_num] = psiNED;
				
				loop_confirm += 1;
				if (loop_confirm == CONFIRM)
				{
					loop_confirm = 0;								// reset loop_confirm counter
					point_num = point_num +1;
					point_published = false;
				}
			}
			if (point_num == goal_poses) 							// means all coordinates have been converted
			{
				waypoints_converted = true;
				ROS_INFO("WAYPOINTS HAVE BEEN CONVERTED");
			}
		} // if ((loop_count > 5) && (!waypoints_published))
		
	    if ((waypoints_converted) && (!waypoints_published))
		{
			NED_waypoints.points.clear();
			
			NED_waypoints.quantity = goal_poses;        // publish quantity of poses so the high level control knows
			
			for (int i = 0; i < goal_poses; i++)
			{
				geometry_msgs::Point point;
				point.x = x_NED[i];
				point.y = y_NED[i];
				point.z = psi_NED[i];
				NED_waypoints.points.push_back(point);
				ROS_INFO("point: %i", i);
				ROS_INFO("x: %f", x_NED[i]);
				ROS_INFO("y: %f", y_NED[i]);
				ROS_INFO("psi: %f", psi_NED[i]);
			}
			waypoints_pub.publish(NED_waypoints);
			waypoints_published = true;
			ROS_INFO("WAYPOINTS HAVE BEEN PUBLISHED");
		} // if ((waypoints_converted) && (!waypoints_published))
		
		if (waypoints_published)
		{
			waypoints_pub.publish(NED_waypoints);
		}
		
		last_time = current_time;
		ros::spinOnce();
		loop_rate.sleep();
		loop_count = loop_count + 1;
	}
	return 0;
} // end of main()
