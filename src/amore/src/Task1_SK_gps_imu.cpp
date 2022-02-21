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
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "tf/transform_broadcaster.h"
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants..................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
int loop_count=0;            // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
double latitude, longitude, altitude;			// The ECEF position variables in spherical coordinates
float qx_goal, qy_goal, qz_goal, qw_goal;								//
float qx, qy, qz, qw;								//
float omega_x, omega_y, omega_z;		// angular velocities
float vx, vy, vz;										// linear velocities
double xNED, yNED, zNED;										// The NED position
float q1NED, q2NED, q3NED, q0NED;						//
float phiNED, thetaNED, psiNED;
float omega_xNED, omega_yNED, omega_zNED;		// angular velocities
float vxNED, vyNED, vzNED;										// linear velocities
float goal_lat, goal_long;
bool goal_lat_long = false;     // if goal_lat_long = false, goal position in lat and long coordinates has not been acquired
bool goal_aqcuired = false;   // if goal_aqcuired = false, goal position has not been aqcuired from SK_Planner
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// this function subscribes to the goal position in lat and long published by the station_keeping task
void goal_pose_sub(const geographic_msgs::GeoPoseStamped::ConstPtr& goal) 
{
	if ((loop_count>5) && (!goal_lat_long))
	{
		goal_lat = goal->pose.position.latitude;
	    goal_long = goal->pose.position.longitude;
		qx_goal = goal->pose.orientation.x;
		qy_goal = goal->pose.orientation.y;
		qz_goal = goal->pose.orientation.z;
		qw_goal = goal->pose.orientation.w;
		goal_lat_long = true;
		// UPDATES STATUSES TO USER ///////////////////////////////////////////////
		ROS_DEBUG("GOAL POSITION HAS BEEN ACQUIRED FROM THE VRX TASK 1 GOAL POSE NODE.");
		ROS_DEBUG("goal_lat: %.2f", goal_lat);
		ROS_DEBUG("goal_long: %.2f", goal_long);
	}
}

// this function subscribes to the SK_Planner node to see when goal pose has been acquired
void goal_publish_check(const std_msgs::Bool published) 
{
	if (published.data)
	{
		goal_aqcuired = true;
	}
	else
	{
		goal_aqcuired = false;
	}
}

void GPS_Position(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	if ((goal_lat_long) && (!goal_aqcuired))
	{
		latitude = goal_lat; //sets latitude to goal for SK_Controller
		longitude = goal_long; //sets longitude to goal for SK_Controller
		altitude = gps_msg->altitude; //sets altitude from gps
	}
	else
	{
		latitude = gps_msg->latitude; //sets latitude from gps
		longitude = gps_msg->longitude; //sets longitude from gps
		altitude = gps_msg->altitude; //sets altitude rom gps
	}
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
	 if ((goal_lat_long) && (!goal_aqcuired))
	 {
		 // gather orientation quaternion
		 qx = qx_goal;
		 qy = qy_goal;
		 qz = qz_goal;
		 qw = qw_goal;
	 }
	 else
	 {
		 // gather orientation quaternion
		 qx = imu_msg->orientation.x;
		 qy = imu_msg->orientation.y;
		 qz = imu_msg->orientation.z;
		 qw = imu_msg->orientation.w;
	 }
	
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
	// Convert the linear velocity to NED from NWU
	vxNED = vx;
	vyNED = -vy;
	vzNED = -vz;
	// Convert the angular velocity to NED
} // end of NED_Func()
//............................................................End of Functions............................................................

//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "gps_imu");
  
	// Variables
	nav_msgs::Odometry nav_odom, nav_NED;
  
	// Node handles
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
	ros::NodeHandle nh3;
	ros::NodeHandle nh4;
	ros::NodeHandle nh5;
	ros::NodeHandle nh6;
	ros::NodeHandle nh7;
	ros::NodeHandle nh8;
  
	// Subscribers
	ros::Subscriber gpspos_sub = nh1.subscribe("/wamv/sensors/gps/gps/fix", 10, GPS_Position); // subscribes to GPS position
	ros::Subscriber imu_sub = nh2.subscribe("/wamv/sensors/imu/imu/data", 1000, IMU_processor);  // subscribes to IMU
	ros::Subscriber gpsvel_sub = nh3.subscribe("/wamv/sensors/gps/gps/fix_velocity", 1000, GPS_Velocity);  // subscribes to GPS velocity
	ros::Subscriber ned_sub = nh4.subscribe("geonav_odom", 1000, NED_Func);
	ros::Subscriber pose_sub = nh5.subscribe("/vrx/station_keeping/goal", 1, goal_pose_sub);                         // subscriber for goal pose given by Task1_SK
	ros::Subscriber SK_Planner_sub = nh6.subscribe("SK_Planner_status", 1, goal_publish_check);               // subscriber for whether or not goal has been acquired by SK_Planner
	// ros::Subscriber state_sub = nh2.subscribe("usv_ned", 10, HMI);  // subscribes to local odometry frame, "geonav_odom" was the topic
	
	// Publishers
	ros::Publisher usvstate_pub = nh7.advertise<nav_msgs::Odometry>("nav_odom", 1000); // USV state publisher
	ros::Publisher nedstate_pub = nh8.advertise<nav_msgs::Odometry>("usv_ned", 1000); // USV state publisher in NED	
  
	// Initialize simulation time
	ros::Time::init();
	ros::Time current_time, last_time;  // creates time variables
	current_time = ros::Time::now();   // sets current time to the time it is now
	last_time = ros::Time::now();        // sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(200); //  {Hz}

	while(ros::ok())
	{
		current_time = ros::Time::now(); //time

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

	//	last_time = current_time
		ros::spinOnce();
		loop_rate.sleep();
		loop_count = loop_count + 1;
	}
	return 0;
} // end of main()
