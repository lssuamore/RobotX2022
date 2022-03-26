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
#include "geographic_msgs/GeoPath.h"
#include "tf/transform_broadcaster.h"
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants..................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
int loop_count=0;                                    // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
double latitude, longitude, altitude;			// The ECEF position variables in spherical coordinates
float qx, qy, qz, qw;								//
float omega_x, omega_y, omega_z;		// angular velocities
float vx, vy, vz;										// linear velocities
double xNED, yNED, zNED;			        // The NED position
float q1NED, q2NED, q3NED, q0NED;
float phiNED, thetaNED, psiNED;
float omega_xNED, omega_yNED, omega_zNED;		// angular velocities
float vxNED, vyNED, vzNED;										// linear velocities
float qx_goal[100], qy_goal[100], qz_goal[100], qw_goal[100];
float goal_lat[100], goal_long[100];
bool WF_conv = true; 							// WF_conv = true means the WF waypoint converter is being used
//bool SK_conv = true;   							// SK_conv = true means the SK point converter is being used
bool function = false;								// function = false means this should not function, thus not publish to nodes that are in use elsewhere
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// this function subscribes to the WF_Converter_status node to see when goal waypoints have been converted and nav_odom is no longer being published to
void WF_conv_status(const std_msgs::Bool published) 
{
	if (published.data)
	{
		WF_conv = true; // this means WF_Converter_status is active
	}
	else
	{
		WF_conv = false;
	}
}

/* // this function subscribes to the SK_Converter_status node to see when goal pose has been converted and nav_odom is no longer being published to
void SK_conv_status(const std_msgs::Bool published) 
{
	if (published.data)
	{
		SK_conv = true; // this means SK_Converter_status is active
	}
	else
	{
		SK_conv = false;
	}
} */

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
//............................................................End of Functions............................................................

//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "gps_imu");
	
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	
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
	//ros::Subscriber SK_Converter_sub = nh5.subscribe("SK_Converter_status", 1, SK_conv_status);              // subscriber for whether or not goal SK pose has been converted yet
	ros::Subscriber WF_Converter_sub = nh6.subscribe("WF_geonav_transform_status", 1, WF_conv_status);            // subscriber for whether or not goal waypoints have been converted yet
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
		
		if ((loop_count > 10) &&(!WF_conv)) // && (!SK_conv)
		{
			function = true;
			//ROS_INFO("POSE CONVERTER ON");
		}
		else
		{
			function = false;
			//ROS_INFO("POSE CONVERTER OFF");
		}
		
		if (function)
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
		
		//last_time = current_time
		ros::spinOnce();
		loop_rate.sleep();
		loop_count = loop_count + 1;
	}
	return 0;
} // end of main()
