// Filename:                   	new_gps_imu.cpp
// Creation Date:		01/22/2022
// Last Revision Date:		03/24/2022
// Author(s) [email]:		Taylor Lamorie [tlamorie@lssu.edu]
// Revisor(s) {Date}:        	Shaede Perzanowski [sperzanowski1@lssu.edu] {01/22/2022}
//				Brad Hacker [bhacker@lssu.edu] {03/24/2022}
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
#include "nav_msgs/Odometry.h"
#include "jetson/state_msg.h"			// message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"		// message type used to hold the goal waypoints w/headings
#include "sensor_msgs/NavSatFix.h"		// message type of lat and long coordinates given by the GPS
#include "sensor_msgs/MagneticField.h"		// message type for compass
#include "sensor_msgs/Imu.h"			// message type of orientation quaternion and angular velocities given by the GPS
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants..................................................................
#define PI 3.14159265
#define CONFIRM 15 // count of loops to hold the same data point for before sending next pose to convert
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;	// loop counter, first 10 loops used to intitialize subscribers

// Variables
// imu and MagneticField 
float q1, q2, q3, q0, q1A, q2A, q3A, q0A;	// imu orientation
float phi, theta, psi, psiA;			// conversion to radians
float comp_x, comp_y, comp_psi;			// compass data
// Sparkfun compass
float compass_psi;
double xNED, yNED, zNED;			// NED position
float phiNED, thetaNED, psiNED, psiNEDA;
double latitude, longitude, altitude;		// LAT and LONG from ardusimple
float omega_x, omega_y, omega_z;		// angular velocities

int NA_state = 0;				// 0 = On standby; 1 = USV NED pose conversion

std_msgs::Bool na_initialization_status;	// "na_initialization_state" message
ros::Publisher na_initialization_state_pub;	// "na_initialization_state" publisher

nav_msgs::Odometry nav_odom_msg, nav_ned_msg;	// "nav_odom" and "nav_ned" messages, respectively
ros::Publisher nav_odom_pub;			// "nav_odom" publisher
ros::Publisher nav_ned_pub;			// "nav_ned" publisher

ros::Time current_time, last_time;		// creates time variables
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// THIS FUNCTION: Updates and publishes initialization status to "na_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void NAVIGATION_ARRAY_inspector()
{
	current_time = ros::Time::now();   	// sets current_time to the time it is now
	loop_count += 1;			// increment loop counter
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
	na_initialization_state_pub.publish(na_initialization_status);	// publish the initialization status of the navigation_array to "na_initialization_state"
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

//...................................................Sensor functions..............................................................
// THIS FUNCTION: Updates the USV position in spherical ECEF coordinates
// ACCEPTS: sensor_msgs::NavSatFix from "/wamv/sensors/gps/gps/fix"
// RETURNS: (VOID)
// =============================================================================
void GPS_Position_update(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	altitude = gps_msg->altitude; //sets altitude rom gps
} // END OF GPS_Position_update()

void sparkfun_update(const std_msgs::Float32::ConstPtr& compass_msg)
{
	compass_psi = compass_msg->data;
	psiNED = compass_psi*(PI/180.0);
	while ((psiNED < -PI) || (psiNED > PI))
        {
                // Adjust psiNED back within -PI and PI
                if (psiNED < -PI)
                {
                        psiNED = psiNED + 2.0*PI;
                }
                if (psiNED > PI)
                {
                        psiNED = psiNED - 2.0*PI;
                }
        }
}

/*
void compass_update(const sensor_msgs::MagneticField::ConstPtr& compass_msg)
{
	// gather magnetic field in x,y,z micro Teslas
	comp_x = compass_msg->magnetic_field.x;
	comp_y = compass_msg->magnetic_field.y;
	//ROS_INFO("comp_x: %.10f comp_y: %.10f", comp_x, comp_y);
	comp_psi = atan2(comp_y,comp_x)+PI;
	psiNED = comp_psi;
	while ((psiNED < -PI) || (psiNED > PI))
	{
                // Adjust psiNED back within -PI and PI
                if (psiNED < -PI)
                {
                        psiNED = psiNED + 2.0*PI;
                }
                if (psiNED > PI)
                {
                        psiNED = psiNED - 2.0*PI;
                }
        }
}
*/

/*
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

	// gather body-fixed angular velocity
	//omega_x = imu_msg->angular_velocity.x;
	//omega_y = imu_msg->angular_velocity.y;
	//omega_z = imu_msg->angular_velocity.z;

	//converts to radians
	//phi = atan2((2.0*(q1*q0 + q3*q2)) , (1.0 - 2.0*(pow(q1,2.0) + pow(q2,2.0))));
	//theta = asin(2.0*(q0*q2 - q1*q3));
	psi = atan2((2.0*(q3*q0 + q1*q2)) , (1.0 - 2.0*(pow(q2,2.0) + pow(q3,2.0)))); // orientation off x-axis
	// Convert the orientation to NED from NWU
	//phiNED = theta;
	//thetaNED = phi;
	psiNED = -1.0*psi + PI;// + 1.57;	// the heading of the Zed imu is offset from the gps by -90 degrees

	while ((psiNED < -PI) || (psiNED > PI))
	{
		// Adjust psiNED back within -PI and PI
		if (psiNED < -PI)
		{
			psiNED = psiNED + 2.0*PI;
		}
		if (psiNED > PI)
		{
			psiNED = psiNED - 2.0*PI;
		}
	}							// NEW
//	ROS_INFO("Zed_heading: %6.2f  -- NA", psiNED);
} // END OF IMU_processor()
*/

/*
// THIS FUNCTION: Updates the USV heading quarternion from the ardusimple
// ACCEPTS: heading from ardusimple
// RETURNS: (VOID)
// =============================================================================
 void GPS_heading(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	// gather orientation quaternion
	q1A = imu_msg->orientation.x;
	q2A = imu_msg->orientation.y;
	q3A = imu_msg->orientation.z;
	q0A = imu_msg->orientation.w;

	psiA = atan2((2.0*(q3A*q0A + q1A*q2A)) , (1.0 - 2.0*(pow(q2A,2.0) + pow(q3A,2.0)))); // orientation off x-axis

	psiNEDA = psiA;

	// Adjust psiNED back within -PI and PI
	if (psiNEDA < -PI)
	{
		psiNEDA = psiNEDA + 2.0*PI;
	}
	if (psiNEDA > PI)
	{
		psiNEDA = psiNEDA - 2.0*PI;
	}
	ROS_INFO("GPS_heading: %6.2f  -- NA", psiNEDA);
} // END OF GPS_heading()
*/

// THIS FUNCTION: Converts the current pose from ENU -> NED
// ACCEPTS: nav_msgs::Odometry from "geonav_odom"
// RETURNS: (VOID)
// =============================================================================
void NED_Func(const nav_msgs::Odometry::ConstPtr& enu_state)
{
	// Convert the position to NED from ENU
	xNED = enu_state->pose.pose.position.y;
	yNED = enu_state->pose.pose.position.x;
	zNED = -(enu_state->pose.pose.position.z);
} // END OF NED_Func(const nav_msgs::Odometry::ConstPtr& enu_state)

void nav_odom_publish()
{
	// Fill the odometry header for nav_odom_msg, state in ENU and NWU
	nav_odom_msg.header.seq +=1;			// sequence number
	nav_odom_msg.header.stamp = current_time;	// sets stamp to current time
	nav_odom_msg.header.frame_id = "odom";		// header frame
	nav_odom_msg.child_frame_id = "base_link";	// child frame

	// Fill the pose
	nav_odom_msg.pose.pose.position.x = longitude;
	nav_odom_msg.pose.pose.position.y = latitude;
	nav_odom_msg.pose.pose.position.z = altitude;
	nav_odom_msg.pose.pose.orientation.x = 0.0;
	nav_odom_msg.pose.pose.orientation.y = 0.0;
	nav_odom_msg.pose.pose.orientation.z = 0.0;
	nav_odom_msg.pose.pose.orientation.w = 0.0;
	nav_odom_msg.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// Fill the velocities
	nav_odom_msg.twist.twist.linear.x = 0.0;
	nav_odom_msg.twist.twist.linear.y = 0.0;
	nav_odom_msg.twist.twist.linear.z = 0.0;
	nav_odom_msg.twist.twist.angular.x = 0.0;
	nav_odom_msg.twist.twist.angular.y = 0.0;
	nav_odom_msg.twist.twist.angular.z = 0.0;
	nav_odom_msg.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// publish the state to be transformed to ENU
	nav_odom_pub.publish(nav_odom_msg);
} // END OF nav_odom_publish()
// =============================================================================

//..............................................................End of sensor functions.................................................................


//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "navigation_array_outdoor");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8;

	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, state_update);			// Gives navigationn array status update
	ros::Subscriber gpspos_sub = nh2.subscribe("/gps/fix", 10, GPS_Position_update);		// subscribes to GPS position from indoor gps
	ros::Subscriber compass_sub = nh3.subscribe("compass_value", 10, sparkfun_update);
	//ros::Subscriber imu_sub = nh3.subscribe("/zed2i/zed_node/imu/data", 100, IMU_processor);	// subscribes to IMU
	//ros::Subscriber compass_sub = nh3.subscribe("/zed2i/zed_node/imu/mag", 10, compass_update);      // subscribes to IMU
	ros::Subscriber geonav_odom_sub = nh4.subscribe("geonav_odom", 10, NED_Func);
	//ros::Subscriber gpsheading_sub = nh5.subscribe("/gps/navheading", 10, GPS_heading);		// subscribes to GPS position from indoor gps

	// Publishers
	na_initialization_state_pub = nh6.advertise<std_msgs::Bool>("na_initialization_state", 1);	// publisher for state of initialization
	nav_ned_pub = nh7.advertise<nav_msgs::Odometry>("nav_ned", 10); 				// USV NED state publisher //gps already in NED but will publish anyways using odometry message type	
	nav_odom_pub = nh8.advertise<nav_msgs::Odometry>("nav_odom", 10); 				// USV state publisher, this sends the current state to nav_odom, so geonav_transform package can publish the ENU conversion to geonav_odom

	// Initialize global variables
	na_initialization_status.data = false;
	current_time = ros::Time::now();   		// sets current time to the time it is now
	last_time = ros::Time::now();        		// sets last time to the time it is now

	// Set the loop sleep rate
	ros::Rate loop_rate(100);			// {Hz} GPS update rate: 20 Hz, IMU update rate: 100 Hz

	while(ros::ok())
	{
		NAVIGATION_ARRAY_inspector();		// check, update, and publish na_initialization_status

		switch(NA_state)
		{
			case 0:				// On standby
				// reset all variables to be used for next run
				break;
			case 1:				// Station-Keeping

				nav_odom_publish();	// THIS FUNCTION FILLS OUT nav_odom_msg AND PUBLISHES TO "nav_odom"

				// Fill the odometry header for nav_ned_msg, the USV state in NED
				nav_ned_msg.header.seq +=1;			// sequence number
				nav_ned_msg.header.stamp = current_time;	// sets stamp to current time
				nav_ned_msg.header.frame_id = "odom";		// header frame
				nav_ned_msg.child_frame_id = "base_link";	// child frame

				// Fill the USV pose in NED
				nav_ned_msg.pose.pose.position.x = xNED;	// FIX THIS BACK LATER
				nav_ned_msg.pose.pose.position.y = yNED;	// FIX THIS BACK LATER
				nav_ned_msg.pose.pose.position.z = 0.0;		// zNED;
				nav_ned_msg.pose.pose.orientation.x = 0.0;	// phiNED;
				nav_ned_msg.pose.pose.orientation.y = 0.0;	// thetaNED;
				nav_ned_msg.pose.pose.orientation.z = psiNED;	// FIX THIS BACK LATER
				nav_ned_msg.pose.pose.orientation.w = 1.23456;
				nav_ned_msg.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

				// Fill the USV velocities in NED
				nav_ned_msg.twist.twist.linear.x = 0.0;
				nav_ned_msg.twist.twist.linear.y = 0.0;
				nav_ned_msg.twist.twist.linear.z = 0.0;
				nav_ned_msg.twist.twist.angular.x = 0.0;	//omega_x;
				nav_ned_msg.twist.twist.angular.y = 0.0;	//omega_y;
				nav_ned_msg.twist.twist.angular.z = 0.0;	//omega_z;
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
