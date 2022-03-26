// Filename:                     	gps_imu.cpp
// Creation Date:                01/22/2022
// Last Revision Date:        01/22/2022
// Author(s) [email]:			Taylor Lamorie [tlamorie@lssu.edu]
// Revisor(s) {Date}:        	Shaede Perzanowski [sperzanowski1@lssu.edu] {01/22/2022}
// Organization/Institution:	Lake Superior State University

//...................................................About gps_imu.cpp.....................................................................
// The gps_imu.cpp file is used 

// Inputs and Outputs of the gps_imu.cpp file
//				Inputs: 
//				Outputs:

//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"

#include "nav_msgs/Odometry.h"

#include "geographic_msgs/GeoPoseStamped.h"				// message type published by VRX Task 1
#include "geometry_msgs/Point.h"

#include "vrx_gazebo/Task.h"												// message published by VRX detailing current task and state

#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants....................................................................
#define PI 3.14159265
#define CONFIRM 15 // count of loops to hold the same data point for
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    						// loop counter, first 10 loops used to intitialize subscribers

double latitude, longitude, altitude;									// ECEF position variables in spherical coordinates
float qx, qy, qz, qw;														// 
float omega_x, omega_y, omega_z;								// angular velocities
float vx, vy, vz;																// linear velocities

double xNED, yNED, zNED;			        						// NED position
float q1NED, q2NED, q3NED, q0NED;
float phiNED, thetaNED, psiNED;
float omega_xNED, omega_yNED, omega_zNED;		// angular velocities
float vxNED, vyNED, vzNED;										// linear velocities

float SK_qx_goal, SK_qy_goal, SK_qz_goal, SK_qw_goal;
float SK_goal_lat, SK_goal_long;
float SK_x_NED, SK_y_NED, SK_psi_NED;

int SK_loops_sent = 0;                                  					// count of loops to hold the same data point for before sending next
int SK_point_num = 0;                                             	  	// used to keep track of the point being converted
int SK_goal_poses_quantity = 1;									// used to keep track of the number of goal poses to convert

bool SK_point_sent = false;              																			// if SK_point_sent = false, the current point has not been published to nav_odom for conversion
bool SK_goal_lat_long_acquired = false;     // if SK_goal_lat_long_acquired = false, goal position in lat and long coordinates has not been acquired
bool SK_pose_converted = false;     // if SK_pose_converted = false, goal pose has not been converted to NED
bool SK_NED_goal_published = false;     // if SK_NED_goal_published = false, goal waypoints in NED have not been published
bool SK_conv = true;                           // if SK_conv = false, this means according to the current task status, conversion shouldn't be done

//bool SK_planner_acquired_goal = false;   // if SK_planner_acquired_goal = false, goal position has not been aqcuired from SK_Planner
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// this function subscribes to the goal position in lat and long published by the station_keeping task
void SK_goal_pose_sub(const geographic_msgs::GeoPoseStamped::ConstPtr& goal) 
{
	if ((!SK_goal_lat_long_acquired) && (SK_conv))
	{
		SK_goal_lat = goal->pose.position.latitude;
	    SK_goal_long = goal->pose.position.longitude;
		SK_qx_goal = goal->pose.orientation.x;
		SK_qy_goal = goal->pose.orientation.y;
		SK_qz_goal = goal->pose.orientation.z;
		SK_qw_goal = goal->pose.orientation.w;
		SK_goal_lat_long_acquired = true;
		// UPDATES STATUSES TO USER ///////////////////////////////////////////////
		ROS_DEBUG("GOAL POSITION HAS BEEN ACQUIRED FROM THE VRX TASK 1 GOAL POSE NODE.");
		//ROS_DEBUG("goal_lat: %.2f", SK_goal_lat);
		//ROS_DEBUG("goal_long: %.2f", SK_goal_long);
	}
}

void SK_GPS_Position()
{
	latitude = SK_goal_lat;
	longitude = SK_goal_long;
	altitude = 0.0; 														//sets altitude to 0
} // end of SK_GPS_Position()

void SK_GPS_Velocity() 
{
	// set velocity to zero since it isn't pertinent
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
} // end of SK_GPS_Velocity()

 void SK_IMU_processor()
{
	// set orientation quaternion
	qx = SK_qx_goal;
	qy = SK_qy_goal;
	qz = SK_qz_goal;
	qw = SK_qw_goal;
	
	// set body-fixed angular velocity to zero since it isn't pertinent
	omega_x = 0.0;
	omega_y = 0.0;
	omega_z = 0.0;
} // end of SK_IMU_processor()

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
	// Convert the linear velocity to NED from NWU
	vxNED = vx;
	vyNED = -vy;
	vzNED = -vz;
	// Convert the angular velocity to NED
} // end of NED_Func()

void update_task(const vrx_gazebo::Task::ConstPtr& msg)
{
	if ((msg->name == "station_keeping") && (!SK_NED_goal_published) && ((msg->state == "ready") || (msg->state == "running")))
	{
		SK_conv = true;
		ROS_INFO("SK POINT CONVERTER ON");
	}
	else
	{
		SK_conv = false;
		//ROS_INFO("SK POINT CONVERER OFF");
	}
}

/* // this function subscribes to the SK_Planner node to see when goal pose has been acquired
void goal_publish_check(const std_msgs::Bool published) 
{
	if (published.data)
	{
		SK_planner_acquired_goal = true;
	}
	else
	{
		SK_planner_acquired_goal = false;
	}
} */

//............................................................End of Functions............................................................
//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "T1_POSE_CONVERTER");
	
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
	ros::Subscriber pose_sub = nh2.subscribe("/vrx/station_keeping/goal", 1, SK_goal_pose_sub);                         // subscriber for goal pose given by Task1_SK
	ros::Subscriber ned_sub = nh3.subscribe("geonav_odom", 1000, NED_Func);
	//ros::Subscriber SK_Planner_sub = nh4.subscribe("SK_Planner_status", 1, goal_publish_check);               // subscriber for whether or not goal has been acquired by SK_Planner
	
	// Publishers
	ros::Publisher usvstate_pub = nh5.advertise<nav_msgs::Odometry>("nav_odom", 1000); 										// USV state publisher
	ros::Publisher SK_NED_goal_pub = nh6.advertise<geometry_msgs::Point>("SK_NED_goal", 100); 							// SK goal pose converted to NED publisher
	ros::Publisher SK_Converter_status_pub = nh7.advertise<std_msgs::Bool>("SK_Converter_status", 1);				// SK_Converter_status publisher
	ros::Publisher SK_geonav_transform_status_pub = nh8.advertise<std_msgs::Bool>("SK_geonav_transform_status", 1);                 // SK_geonav_transform_status publisher
	
	// Local variables
	nav_msgs::Odometry nav_odom;
	std_msgs::Bool SK_publish_status, SK_geonav_transform_status;
	geometry_msgs::Point SK_NED_pose;
  
	// Initialize simulation time
	ros::Time::init();
	ros::Time current_time, last_time;  // creates time variables
	current_time = ros::Time::now();   // sets current time to the time it is now
	last_time = ros::Time::now();        // sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(100); //  {Hz}

	while(ros::ok())
	{
		current_time = ros::Time::now(); //time
		
		// publish whether or not the SK goal pose converter has published the pose for the SK planner
		if (!SK_pose_converted)
		{
		  SK_publish_status.data = false;
		}
		else
		{
		  SK_publish_status.data = true;
		}
		SK_Converter_status_pub.publish(SK_publish_status);
		
		// publish whether or not this code is using geonav transform package
		if (!SK_conv)
		{
		  SK_geonav_transform_status.data = false;
		}
		else
		{
		  SK_geonav_transform_status.data = true;
		}
		SK_Converter_status_pub.publish(SK_geonav_transform_status);
		
		if ((!SK_pose_converted) && (SK_goal_lat_long_acquired) && (SK_conv))
		{
			if (!SK_point_sent)
			{
				// UPDATE NAV_ODOM MSG
				SK_GPS_Position();
				SK_GPS_Velocity();
				SK_IMU_processor();
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
				SK_point_sent = true;
			}
			
			// Update goal pose in NED units
			if (SK_point_num < SK_goal_poses_quantity)
			{
				SK_x_NED = xNED;
				SK_y_NED = yNED;
				SK_psi_NED = psiNED;
				
				SK_loops_sent += 1;
				if (SK_loops_sent == CONFIRM)
				{
					SK_loops_sent = 0;								// reset WF_loops_sent counter
					SK_point_num += 1;
					SK_point_sent = false;
				}
			}
			if (SK_point_num == SK_goal_poses_quantity) 							// means all coordinates have been converted
			{
				SK_pose_converted = true;
				ROS_INFO("GOAL POSE HAS BEEN CONVERTED");
			}
		} // if ((!SK_pose_converted) && (SK_goal_lat_long_acquired) && (SK_conv))
		
		if ((SK_pose_converted) && (!SK_NED_goal_published))
		{
			SK_NED_pose.x = SK_x_NED;
			SK_NED_pose.y = SK_y_NED;
			SK_NED_pose.z = SK_psi_NED;
			ROS_INFO("x: %f", SK_x_NED);
			ROS_INFO("y: %f", SK_y_NED);
			ROS_INFO("psi: %f", SK_psi_NED);
			
			SK_NED_goal_pub.publish(SK_NED_pose);
			SK_NED_goal_published = true;
			ROS_INFO("WAYPOINTS HAVE BEEN PUBLISHED");
		} // if ((SK_pose_converted) && (!SK_NED_goal_published))
		
		if (SK_NED_goal_published)
		{
			SK_NED_goal_pub.publish(SK_NED_pose);
		}

		last_time = current_time;
		ros::spinOnce();
		loop_rate.sleep();
		loop_count = loop_count + 1;
	}
	return 0;
} // end of main()
