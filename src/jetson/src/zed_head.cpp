// Filename:                   	zed_head.cpp
// Creation Date:		04/20/2022
// Last Revision Date:			
// Author(s) [email]:		Shaede Perzanowski [sperzanowski1@lssu.edu]
// Revisor(s) {Date}:
// Organization/Institution:	Lake Superior State University

//...................................................About zed_head.cpp.....................................................................
// This program obtains the heading from the ZED 2i and converts it to NED degrees.

// Inputs and Outputs of the zed_head.cpp file
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
#include "jetson/state_msg.h"												// message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"										// message type used to hold the goal waypoints w/headings
#include "sensor_msgs/NavSatFix.h"									// message type of lat and long coordinates given by the GPS
#include "sensor_msgs/Imu.h"												// message type of orientation quaternion and angular velocities given by the GPS
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants..................................................................
#define PI 3.14159265
#define CONFIRM 15 // count of loops to hold the same data point for before sending next pose to convert
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    							// loop counter, first 10 loops used to intitialize subscribers

// Variables
float q1, q2, q3, q0;                                                             //	imu orientation
float phi, theta, psi;                                                             //	conversion to radians 

double xNED, yNED, zNED;			        							// NED position					
float phiNED, thetaNED, psiNED;									
float omega_x, omega_y, omega_z;			// angular velocities

int NA_state = 0;							// 0 = On standby; 1 = USV NED pose conversion

std_msgs::Bool na_initialization_status;																		// "na_initialization_state" message
ros::Publisher na_initialization_state_pub;																	// "na_initialization_state" publisher

nav_msgs::Odometry nav_odom_msg, nav_ned_msg;												// "nav_odom" and "nav_ned" messages, respectively
ros::Publisher nav_odom_pub;																						// "nav_odom" publisher
ros::Publisher nav_ned_pub;																							// "nav_ned" publisher

ros::Time current_time, last_time;																				// creates time variables
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// THIS FUNCTION: Updates and publishes initialization status to "na_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void NAVIGATION_ARRAY_inspector()
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;									// increment loop counter
	if (loop_count > 5)
	{
		na_initialization_status.data = true;
		//ROS_INFO("navigation_array_initialized -- NA");
	}
	else
	{
		na_initialization_status.data = false;
		ROS_INFO("!navigation_array_initialized -- NA");
	}
	na_initialization_state_pub.publish(na_initialization_status);						// publish the initialization status of the navigation_array to "na_initialization_state"
} // END OF NAVIGATION_ARRAY_inspector()

// THIS FUNCTION: Updates the state of navigation_array given by mission_control
// ACCEPTS: state_msg from "na_state"
// RETURNS: (VOID)
// =============================================================================
void state_update(const jetson::state_msg::ConstPtr& msg)
{
	// do not start anything until subscribers to sensor data are initialized
	if (na_initialization_status.data)
	{
		NA_state = msg->state.data;
	}
} // END OF state_update()

//..................................................................Sensor functions.................................................................
// THIS FUNCTION: Updates the USV position in spherical ECEF coordinates 
// ACCEPTS: sensor_msgs::NavSatFix from "/wamv/sensors/gps/gps/fix"
// RETURNS: (VOID)
// =============================================================================
void GPS_Position_update(const geometry_msgs::Point::ConstPtr& gps_msg)
{
	//	indoor gps gives distance in cm so divide by 100 to get to meters
	xNED = (gps_msg->x) / 100; //sets x from gps
	yNED = (gps_msg->y) / 100; //sets y from gps
} // END OF GPS_Position_update()

// THIS FUNCTION: Updates the USV heading quarternion and angular velocities
// ACCEPTS: sensor_msgs::Imu from Sparton
// RETURNS: (VOID)
// =============================================================================
 void IMU_processor(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	// gather orientation quaternion
	q1 = imu_msg->orientation.x;
	q2 = imu_msg->orientation.y;
	q3 = imu_msg->orientation.z;
	q0 = imu_msg->orientation.w;
	
	/* // gather body-fixed angular velocity
	omega_x = imu_msg->angular_velocity.x;
	omega_y = imu_msg->angular_velocity.y;
	omega_z = imu_msg->angular_velocity.z; */
	
	//converts to radians 
	/* phi = atan2((2.0*(q1*q0 + q3*q2)) , (1.0 - 2.0*(pow(q1,2.0) + pow(q2,2.0)))); 
	theta = asin(2.0*(q0*q2 - q1*q3)); */
	psi = atan2((2.0*(q3*q0 + q1*q2)) , (1.0 - 2.0*(pow(q2,2.0) + pow(q3,2.0)))); // orientation off x-axis
	
	// Convert the orientation to NED from NWU
	/* phiNED = theta;
	thetaNED = phi; */
	psiNED = -1.0*psi;
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// important camera IMU in NWU
	//is this correct for NWU to NED
	//psiNED = psi - PI/2.0;
	
	// Adjust psiNED back within -PI and PI
	if (psiNED < -PI)
	{
		psiNED = psiNED + 2.0*PI;
	}
	if (psiNED > PI)
	{
		psiNED = psiNED - 2.0*PI;
	}
} // END OF IMU_processor()

// THIS FUNCTION: Converts the current pose from ENU -> NED
// ACCEPTS: nav_msgs::Odometry from "geonav_odom"
// RETURNS: (VOID)
// =============================================================================

//..............................................................End of sensor functions.................................................................


//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "navigation_array");
	
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  
	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8;
  
	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, state_update);                                    // Gives navigationn array status update
	//ros::Subscriber gpspos_sub = nh2.subscribe("point", 10, GPS_Position_update);							// subscribes to GPS position from indoor gps
	//ros::Subscriber imu_sub = nh3.subscribe("/zed/zed_node/imu/data", 100, IMU_processor);			// subscribes to IMU
	
	// Publishers
	na_initialization_state_pub = nh5.advertise<std_msgs::Bool>("na_initialization_state", 1);				// publisher for state of initialization
	nav_ned_pub = nh6.advertise<nav_msgs::Odometry>("nav_ned", 100); 											// USV NED state publisher //gps already in NED but will publish anyways using odometry message type	
	
	// Initialize global variables
	na_initialization_status.data = false;
	current_time = ros::Time::now();   											// sets current time to the time it is now
	last_time = ros::Time::now();        											// sets last time to the time it is now
	  
	// Set the loop sleep rate
	ros::Rate loop_rate(100);															// {Hz} GPS update rate: 20 Hz, IMU update rate: 100 Hz

	while(ros::ok())
	{		
		NAVIGATION_ARRAY_inspector();									// check, update, and publish na_initialization_status
		
		switch(NA_state)
		{
			case 0:						// On standby
				// reset all variables to be used for next run
				break;
			case 1:						// Station-Keeping
				// Fill the odometry header for nav_ned_msg, the USV state in NED
				nav_ned_msg.header.seq +=1;										// sequence number
				nav_ned_msg.header.stamp = current_time;				// sets stamp to current time
				nav_ned_msg.header.frame_id = "odom";					// header frame
				nav_ned_msg.child_frame_id = "base_link";				// child frame
			
				// Fill the USV pose in NED
				nav_ned_msg.pose.pose.position.x = 0.0; //xNED;														// FIX THIS BACK LATER 
				nav_ned_msg.pose.pose.position.y = 0.0; //yNED;														// FIX THIS BACK LATER 
				nav_ned_msg.pose.pose.position.z = 0.0; //zNED;
				nav_ned_msg.pose.pose.orientation.x = 0.0; //phiNED;
				nav_ned_msg.pose.pose.orientation.y = 0.0; //thetaNED;
				nav_ned_msg.pose.pose.orientation.z = 0.0; //psiNED;												// FIX THIS BACK LATER 
				nav_ned_msg.pose.pose.orientation.w = 1.23456;
				nav_ned_msg.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			
				// Fill the USV velocities in NED
				nav_ned_msg.twist.twist.linear.x = 0.0;
				nav_ned_msg.twist.twist.linear.y = 0.0;
				nav_ned_msg.twist.twist.linear.z = 0.0;
				nav_ned_msg.twist.twist.angular.x = 0.0; //omega_x;
				nav_ned_msg.twist.twist.angular.y = 0.0; //omega_y;
				nav_ned_msg.twist.twist.angular.z = 0.0; //omega_z;
				nav_ned_msg.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			
				// publish the NED USV state
				nav_ned_pub.publish(nav_ned_msg);
				break;
			default:
				break;
		}	// switch(NA_state)
		ros::spinOnce();
		loop_rate.sleep();
		last_time = current_time;
	}
	return 0;
} // end of main()
