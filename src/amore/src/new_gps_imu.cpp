// Filename:                     	new_gps_imu.cpp
// Creation Date:					01/22/2022
// Last Revision Date:			03/24/2022
// Author(s) [email]:				Taylor Lamorie [tlamorie@lssu.edu]
// Revisor(s) {Date}:        	Shaede Perzanowski [sperzanowski1@lssu.edu] {01/22/2022}
//											Brad Hacker [bhacker@lssu.edu] {03/24/2022}
// Organization/Institution:	Lake Superior State University

//...................................................About new_gps_imu.cpp.....................................................................
// Used to interact with the geonav_transform package. This package is used to convert lat/long coordinates
// to a NED (North-x East-y Down-z) coordinate system that is more user friendly to work with. This code
// subscribes to all sensors to get information to fill message needed for geonav_transform package. It's purposes 
// are to provide the system with NED pose information, as well as to subscribe to task goal poses and convert
// them to NED to be published for planner to subscribe to.

// Inputs and Outputs of the new_gps_imu.cpp file
//				Inputs: vrx/task/info, GPS and IMU sensor data, task 1 and 2 goal poses
//				Outputs: USV_NED pose, WF_poses, SK_pose


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

#include "sensor_msgs/NavSatFix.h"									// message type of lat and long coordinates given by the gps_sensor
#include "sensor_msgs/Imu.h"												// message type of orientation quaternion and angular velocities given by the GPS
#include "geometry_msgs/Vector3Stamped.h"						// message type of linear velocities given by the IMU
 
 // Task 1 libraries and message types
#include "geographic_msgs/GeoPoseStamped.h"				// message type published by VRX Task 1

// Task 2 libraries and message types
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

bool NED_conv = false;												// NED_conv = false means that standard NED pose conv is not occuring

// Variables for SK converter
bool SK_conv = false;   												// SK_conv = true means the SK point converter is being used

// Variables for WF converter
int WF_loops_sent = 0;                                  																// count of loops to hold the same data point for before sending next
int WF_point_num=0;                                               														// used to keep track of the point being converted
int WF_goal_poses_quantity=-1;                                   												// used to keep track of the number of goal poses to convert

float WF_qx_goal[100], WF_qy_goal[100], WF_qz_goal[100], WF_qw_goal[100];
float WF_goal_lat[100], WF_goal_long[100];
float WF_x_NED[100], WF_y_NED[100], WF_psi_NED[100];

bool WF_point_sent = false;              																			// if WF_point_sent = false, the current point has not been published to nav_odom for conversion
bool WF_goal_lat_long_acquired = false;                  													// if WF_goal_lat_long_acquired = false, goal waypoint poses in lat and long coordinates have not been acquired
bool WF_waypoints_converted = false;     																	// if WF_waypoints_converted = false, goal waypoints in NED have not been converted
bool WF_waypoints_published = false;     																	// if WF_waypoints_published = false, goal waypoints in NED have not been published
bool WF_conv = false; 																									// WF_conv = true means the WF waypoint converter is being used
//........................................................End of Global Variables........................................................

void update_task(const vrx_gazebo::Task::ConstPtr& msg)
{
	// do not start anything until subscribers to sensor data are initialized
	if (loop_count > 10)
	{
		// if the task is wayfinding and the state is ready or running and the waypoints haven't been published, converter will run
		if ((msg->name == "wayfinding") && (!WF_waypoints_converted) && ((msg->state == "ready") || (msg->state == "running")))
		{
			WF_conv = true;
			SK_conv = false;
			NED_conv = false;
			ROS_INFO("WF POINT CONVERTER ON");
		}
		/* else if ((msg->name == "station_keeping") && (msg->state == "ready"))
		{
			WF_conv = false;
			SK_conv = true;
			NED_conv = false;
			ROS_INFO("WF POINT CONVERTER ON");
		} */
		else
		{
			WF_conv = false;
			SK_conv = false;
			NED_conv = true;
			ROS_INFO("USV_NED pose conversion happening");
		}
	} // if (loop_count > 10)
} // end of update_task()

//..................................................................NED_conv functions.................................................................
void GPS_Position(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	altitude = gps_msg->altitude; //sets altitude rom gps
} // end of GPS_Position()

void GPS_Velocity(const geometry_msgs::Vector3Stamped::ConstPtr& vel_msg) 
{
	// gather the velocity, which is in NWU, wrt the GPS sensor (which I think is almost the USV origin)
	vx = vel_msg->vector.x;
	vy = vel_msg->vector.y;
	vz = vel_msg->vector.z;
} // end of GPS_Velocity()

 void IMU_processor(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	// gather orientation quaternion
	qx = imu_msg->orientation.x;
	qy = imu_msg->orientation.y;
	qz = imu_msg->orientation.z;
	qw = imu_msg->orientation.w;
	
	// gather body-fixed angular velocity
	omega_x = imu_msg->angular_velocity.x;
	omega_y = imu_msg->angular_velocity.y;
	omega_z = imu_msg->angular_velocity.z;
} // end of IMU_processor()

/* void HMI(const nav_msgs::Odometry::ConstPtr& odom)
{
	printf("the x is: %f\n", odom->pose.pose.position.x); //extracts x coor from nav_odometery
	printf("the y is: %f\n", odom->pose.pose.position.y); //extracts y coor from nav_odometery
	printf("the x orientation is: %f\n", odom->pose.pose.orientation.x); //extracts x orientation
	printf("the y orientation is: %f\n", odom->pose.pose.orientation.y); //extracts y orientation
	printf("the z orientation is: %f\n", odom->pose.pose.orientation.z); //extracts z orientation
	printf("the w orientation is: %f\n", odom->pose.pose.orientation.w); //extracts w orientation
	printf("the velocity x is: %f\n", odom-> twist.twist.linear.x);//prints velocity x
	printf("the velocity y is: %f\n", odom-> twist.twist.linear.y);//prints velocity y
	printf("the velocity z is: %f\n", odom-> twist.twist.linear.z);//prints velocity z
	printf("the angular velocity x is: %f\n", odom-> twist.twist.angular.x);//prints velocity x
	printf("the angular velocity y is: %f\n", odom-> twist.twist.angular.y);//prints velocity x
	printf("the angular velocity z is: %f\n", odom-> twist.twist.angular.z);//prints velocity x
} // end of HMI() */

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
//............................................................End of NED_convs functions............................................................

//.....................................................................WF_conv functions...................................................................
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

// this function subscribes to the goal position in lat and long published by the station_keeping task
void WF_goal_pose_sub(const geographic_msgs::GeoPath::ConstPtr& goal)
{
	ROS_INFO("CHECK IF.");
	if ((!WF_goal_lat_long_acquired) && (WF_conv)) //(loop_count>5) && 
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
//............................................................End of WF_conv functions............................................................

//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "gps_imu");
	
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
	ros::NodeHandle nh10;
	ros::NodeHandle nh11;
	ros::NodeHandle nh12;
	ros::NodeHandle nh13;
  
	// Subscribers
	ros::Subscriber task_status = nh1.subscribe("/vrx/task/info", 1, update_task);
	ros::Subscriber gpspos_sub = nh2.subscribe("/wamv/sensors/gps/gps/fix", 10, GPS_Position); // subscribes to GPS position
	ros::Subscriber imu_sub = nh3.subscribe("/wamv/sensors/imu/imu/data", 1000, IMU_processor);  // subscribes to IMU
	ros::Subscriber gpsvel_sub = nh4.subscribe("/wamv/sensors/gps/gps/fix_velocity", 1000, GPS_Velocity);  // subscribes to GPS velocity
	ros::Subscriber ned_sub = nh5.subscribe("geonav_odom", 1000, NED_Func);
	
	ros::Subscriber WF_waypoints_sub; // = nh7.subscribe("/vrx/wayfinding/waypoints", 100, WF_goal_pose_sub);                         			// subscriber for goal waypoints given by Task2_WF
	// ros::Subscriber state_sub = nh2.subscribe("usv_ned", 10, HMI);  // subscribes to local odometry frame, "geonav_odom" was the topic
	
	// Publishers
	ros::Publisher nedstate_pub = nh10.advertise<nav_msgs::Odometry>("usv_ned", 1000); 											// USV NED state publisher
	ros::Publisher usvstate_pub = nh11.advertise<nav_msgs::Odometry>("nav_odom", 1000); 										// USV state publisher, this sends the current state to nav_odom, so geonav_transform package can publish the ENU conversion to geonav_odom
	
	ros::Publisher WF_waypoints_pub = nh12.advertise<amore::NED_waypoints>("waypoints_NED", 100); 							// WF goal poses converted to NED publisher
	ros::Publisher WF_Converter_status_pub = nh13.advertise<std_msgs::Bool>("WF_Converter_status", 1);               // WF_Converter_status publisher
	
	// Local variables
	nav_msgs::Odometry nav_odom, nav_NED;
	geographic_msgs::GeoPath waypoints_NED;
	geographic_msgs::GeoPoseStamped pose_NED;
	std_msgs::Bool WF_publish_status;
	amore::NED_waypoints NED_waypoints;
	
	// Initialize simulation time
	ros::Time::init();
	ros::Time current_time, last_time;	// creates time variables
	current_time = ros::Time::now();   	// sets current time to the time it is now
	last_time = ros::Time::now();        	// sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(20);						// {Hz} GPS update rate: 20 Hz, IMU update rate: 100 Hz

	while(ros::ok())
	{
		current_time = ros::Time::now(); //time
		
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
		
		if (NED_conv)
		{
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
		
			// publish the USV state to be transformed to ENU
			usvstate_pub.publish(nav_odom);
		
			// Fill the odometry header for nav_NED, the USV state in NED
			nav_NED.header.seq +=1;								// sequence number
			nav_NED.header.stamp = current_time;				// sets stamp to current time
			nav_NED.header.frame_id = "odom";					// header frame
			nav_NED.child_frame_id = "base_link";				// child frame
		
			// Fill the USV pose in NED
			nav_NED.pose.pose.position.x = xNED; //sets long
			nav_NED.pose.pose.position.y = yNED; //sets lat
			nav_NED.pose.pose.position.z = zNED; //sets altitude
			nav_NED.pose.pose.orientation.x = phiNED;
			nav_NED.pose.pose.orientation.y = thetaNED;
			nav_NED.pose.pose.orientation.z = psiNED;
			nav_NED.pose.pose.orientation.w = 1.23456;
			nav_NED.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		
			// Fill the USV velocities in NED
			nav_NED.twist.twist.linear.x = vxNED;
			nav_NED.twist.twist.linear.y = vyNED;
			nav_NED.twist.twist.linear.z = vzNED;
			nav_NED.twist.twist.angular.x = omega_x;
			nav_NED.twist.twist.angular.y = omega_y;
			nav_NED.twist.twist.angular.z = omega_z;
			nav_NED.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		
			// publish the NED USV state
			nedstate_pub.publish(nav_NED);
		} // if ((loop_count > 10) && (!WF_conv) && (!SK_conv))
		
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ START OF WF GOAL POSES CONVERSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		if ((!WF_waypoints_converted) && (WF_conv))
		{
			// first subscribe to goal waypointsand update global arrays
			if (!WF_goal_lat_long_acquired)
			{
				WF_waypoints_sub = nh5.subscribe("/vrx/wayfinding/waypoints", 100, WF_goal_pose_sub);                         			// subscriber for goal waypoints given by Task2_WF
				ros::spinOnce();
				loop_rate.sleep();
			}
			if ((!WF_point_sent)  && (WF_goal_lat_long_acquired))
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
			NED_waypoints.points.clear();
			
			NED_waypoints.quantity = WF_goal_poses_quantity;        // publish quantity of poses so the high level control knows
			
			for (int i = 0; i < WF_goal_poses_quantity; i++)
			{
				geometry_msgs::Point point;
				point.x = WF_x_NED[i];
				point.y = WF_y_NED[i];
				point.z = WF_psi_NED[i];
				NED_waypoints.points.push_back(point);
				ROS_INFO("point: %i", i);
				ROS_INFO("x: %f", WF_x_NED[i]);
				ROS_INFO("y: %f", WF_y_NED[i]);
				ROS_INFO("psi: %f", WF_psi_NED[i]);
			}
			WF_waypoints_pub.publish(NED_waypoints);
			WF_waypoints_published = true;
			ROS_INFO("WAYPOINTS HAVE BEEN PUBLISHED");
		} // if ((WF_waypoints_converted) && (!WF_waypoints_published))
		
		if (WF_waypoints_published)
		{
			WF_waypoints_pub.publish(NED_waypoints);
		}
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ END OF WF GOAL POSES CONVERSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		
		last_time = current_time;
		ros::spinOnce();
		loop_rate.sleep();
		loop_count = loop_count + 1;
	}
	return 0;
} // end of main()
