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
#include "nav_msgs/Odometry.h"
#include "amore/state_msg.h"												// message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"
#include "vrx_gazebo/Task.h"												// message published by VRX detailing current task and state
#include "amore/NED_waypoints.h"										// message created to hold an array of the converted WF goal waypoints w/ headings and the number of goal poses
#include "geographic_msgs/GeoPoseStamped.h"				// message type published by VRX Task 1
#include "geographic_msgs/GeoPath.h"								// message type published by VRX Task 2
#include "geometry_msgs/Point.h"										// message type used to hold the goal waypoints w/headings

#include "sensor_msgs/NavSatFix.h"									// message type of lat and long coordinates given by the GPS
#include "sensor_msgs/Imu.h"												// message type of orientation quaternion and angular velocities given by the GPS
#include "geometry_msgs/Vector3Stamped.h"						// message type of linear velocities given by the IMU

//#include "std_msgs/Int32.h"
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants..................................................................
#define PI 3.14159265
#define CONFIRM 15 // count of loops to hold the same data point for before sending next pose to convert
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
int loop_count = 0;                                    					// loop counter, first 10 loops used to intitialize subscribers

double latitude, longitude, altitude;									// ECEF position variables in spherical coordinates
float qx, qy, qz, qw;														
float omega_x, omega_y, omega_z;								// angular velocities
float vx, vy, vz;																// linear velocities

double xNED, yNED, zNED;			        						// NED position // WHY DOUBLE, LETS JUST FLOAT
float q1NED, q2NED, q3NED, q0NED;							
float phiNED, thetaNED, psiNED;									
float omega_xNED, omega_yNED, omega_zNED;			// angular velocities
float vxNED, vyNED, vzNED;										// linear velocities

int NA_state = 0;							// 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)

// Variables for WF converter
int WF_loops_sent = 0;                                  															// count of loops to hold the same data point for before sending next
int point_num = 0;                                               														// used to keep track of the point being converted
int goal_poses_quantity = -1;                                   											// used to keep track of the number of goal poses to convert

float qx_goal[100], qy_goal[100], qz_goal[100], qw_goal[100];									// waypoint headings in quarternion form
float goal_lat[100], goal_long[100];																			// waypoint spherical ECEF lat/long coordinates
float NED_x_goal[100], NED_y_goal[100], NED_psi_goal[100];										// NED goal waypoint array

bool lat_lon_point_sent = false;              																	// lat_lon_point_sent = false, the current point has not been published to nav_odom for conversion
bool lat_lon_goal_recieved = false;                  															// lat_lon_goal_recieved = false, goal waypoint poses in lat and long coordinates have not been acquired
bool NED_waypoints_converted = false;     																// NED_waypoints_converted = false, goal waypoints in NED have not been converted
bool NED_waypoints_published = false;     																// NED_waypoints_published = false, goal waypoints in NED units have not been published

bool navigation_array_initialized = false;																	// navigation_array_initialized = false means navigation_array is not initialized
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// THIS FUNCTION SUBSCRIBES TO THE NAVIGATION_ARRAY TO CHECK INITIALIZATION
void NAVIGATION_ARRAY_inspector()
{
	if (loop_count > 10)
	{
		navigation_array_initialized = true;
		//ROS_INFO("navigation_array_initialized -- NA");
	}
	else
	{
		navigation_array_initialized = false;
		ROS_INFO("!navigation_array_initialized -- NA");
	}	
} // END OF NAVIGATION_ARRAY_inspector()

// THIS FUNCTION SUBSCRIBES "na_state" TO SEE THE CURRENT STATE OF NAVIGATION_ARRAY
void state_update(const amore::state_msg::ConstPtr& msg)
{
	// do not start anything until subscribers to sensor data are initialized
	if (navigation_array_initialized)
	{
		NA_state = msg->state.data;
	}
} // END OF state_update(const amore::state_msg::ConstPtr& msg)

//..................................................................USV_NED_conv functions.................................................................
void GPS_Position_update(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	altitude = gps_msg->altitude; //sets altitude rom gps
} // END OF GPS_Position_update()

void GPS_Velocity_update(const geometry_msgs::Vector3Stamped::ConstPtr& vel_msg) 
{
	// gather the velocity, which is in NWU, wrt the GPS sensor (which I think is almost the USV origin)
	vx = vel_msg->vector.x;
	vy = vel_msg->vector.y;
	vz = vel_msg->vector.z;
} // END OF GPS_Velocity_update()

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
} // END OF IMU_processor()

// THIS FUNCTION CONVERTS THE CURRENT POSE FROM ENU -> NED
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
} // END OF NED_Func(const nav_msgs::Odometry::ConstPtr& enu_state)
//............................................................End of USV_NED_convs functions............................................................

//.....................................................................goal pose conversion functions...................................................................
// THIS FUNCTION FILLS THE LAT/LONG OF THE DESIRED POSE TO BE CONVERTED
void GPS_Position_update()
{
	latitude = goal_lat[point_num];
	longitude = goal_long[point_num];
	altitude = 0.0; 														//sets altitude to 0
} // END OF GPS_Position_update()

// THIS FUNCTION FILLS THE VELOCITIES WITH ZERO SINCE IT IS NOT PERTINENT
void GPS_Velocity_update() 
{
	// set velocity to zero since it isn't pertinent
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
} // END OF GPS_Velocity_update()

// THIS FUNCTION FILLS THE QUARTERNION OF THE DESIRED POSE TO BE CONVERTED
 void IMU_processor()
{
	// set orientation quaternion
	qx = qx_goal[point_num];
	qy = qy_goal[point_num];
	qz = qz_goal[point_num];
	qw = qw_goal[point_num];
	
	// set body-fixed angular velocity to zero since it isn't pertinent
	omega_x = 0.0;
	omega_y = 0.0;
	omega_z = 0.0;
} // END OF IMU_processor()

// THIS FUNCTION SUBSCRIBES TO "/vrx/station_keeping/goal" TO GET THE SK GOAL POSE IN LAT/LONG
void VRX_T1_goal_update(const geographic_msgs::GeoPoseStamped::ConstPtr& goal) 
{
	if ((!lat_lon_goal_recieved) && (NA_state == 2))
	{
		point_num = 0;		// reset point_num to begin at 0
		
		goal_lat[point_num] = goal->pose.position.latitude;
		goal_long[point_num] = goal->pose.position.longitude;
		qx_goal[point_num] = goal->pose.orientation.x;
		qy_goal[point_num] = goal->pose.orientation.y;
		qz_goal[point_num] = goal->pose.orientation.z;
		qw_goal[point_num] = goal->pose.orientation.w;
		lat_lon_goal_recieved = true;
		goal_poses_quantity = 1;
		// UPDATES STATUSES TO USER ///////////////////////////////////////////////
		ROS_DEBUG("GOAL POSITION HAS BEEN ACQUIRED FROM THE VRX TASK 1 GOAL POSE NODE.");
		//ROS_DEBUG("goal_lat: %.2f", SK_goal_lat);
		//ROS_DEBUG("goal_long: %.2f", SK_goal_long);
	}
} // END OF VRX_T1_goal_update(const geographic_msgs::GeoPoseStamped::ConstPtr& goal)

// THIS FUNCTION SUBSCRIBES TO "/vrx/wayfinding/waypoints" TO GET THE WF GOAL POSES IN LAT/LONG
void VRX_T2_goal_update(const geographic_msgs::GeoPath::ConstPtr& goal)
{
	if ((!lat_lon_goal_recieved) && (NA_state == 3))
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
		lat_lon_goal_recieved = true;
		// UPDATES STATUSES TO USER ///////////////////////////////////////////////
		ROS_INFO("GOAL POSITIONS HAVE BEEN ACQUIRED FROM THE VRX TASK 2 WAYPOINTS NODE.");
		ROS_INFO("Size of array: %i", (int)sizeof(goal->poses)/8);
		goal_poses_quantity = (int)sizeof(goal->poses)/8;
	}
	else
	{
		ROS_INFO("NOT ENTERING IF.");
	}
} // END OF VRX_T2_goal_update(const geographic_msgs::GeoPath::ConstPtr& goal)
//............................................................End of goal pose conversion functions............................................................

//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "navigation_array");
	
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  
	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh11, nh12, nh13;
  
	// Subscribers
	ros::Subscriber NA_status_sub = nh1.subscribe("na_state", 1, state_update);
	ros::Subscriber gpspos_sub = nh2.subscribe("/wamv/sensors/gps/gps/fix", 10, GPS_Position_update);						// subscribes to GPS position
	ros::Subscriber imu_sub = nh3.subscribe("/wamv/sensors/imu/imu/data", 100, IMU_processor);								// subscribes to IMU
	ros::Subscriber gpsvel_sub = nh4.subscribe("/wamv/sensors/gps/gps/fix_velocity", 100, GPS_Velocity_update);		// subscribes to GPS velocity
	ros::Subscriber ned_sub = nh5.subscribe("geonav_odom", 100, NED_Func);
	ros::Subscriber VRX_T1_goal_sub; // = nh6.subscribe("/vrx/station_keeping/goal", 1, VRX_T1_goal_update);				// subscriber for goal pose given by Task1_SK
	ros::Subscriber VRX_T2_goal_sub; // = nh6.subscribe("/vrx/wayfinding/waypoints", 100, VRX_T2_goal_update);			// subscriber for goal waypoints given by Task2_WF
	// ros::Subscriber state_sub = nh2.subscribe("usv_ned", 10, HMI);  // subscribes to local odometry frame, "geonav_odom" was the topic
	
	// Publishers
	ros::Publisher NA_initializer_pub = nh8.advertise<std_msgs::Bool>("NA_initialization_state", 1);							// publishes state of initialization
	ros::Publisher nedstate_pub = nh10.advertise<nav_msgs::Odometry>("usv_ned", 100); 										// USV NED state publisher
	ros::Publisher usvstate_pub = nh11.advertise<nav_msgs::Odometry>("nav_odom", 100); 									// USV state publisher, this sends the current state to nav_odom, so geonav_transform package can publish the ENU conversion to geonav_odom
	ros::Publisher waypoints_NED_pub = nh12.advertise<amore::NED_waypoints>("waypoints_NED", 100); 				// goal poses converted to NED publisher
	ros::Publisher goal_publish_state_pub = nh13.advertise<std_msgs::Bool>("goal_publish_state", 1);						// goal_publish_state publisher
	
	// Local variables
	nav_msgs::Odometry nav_odom, nav_NED;
	std_msgs::Bool goal_publish_status;									// "goal_publish_state"
	amore::NED_waypoints NED_waypoints;							// "waypoints_NED"
	std_msgs::Bool NA_initialization_status;							// "NA_initialization_state"
	
	// Initialize simulation time
	ros::Time::init();
	ros::Time current_time, last_time;	// creates time variables
	current_time = ros::Time::now();   	// sets current time to the time it is now
	last_time = ros::Time::now();        	// sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(20);						// {Hz} GPS update rate: 20 Hz, IMU update rate: 100 Hz

	while(ros::ok())
	{
		current_time = ros::Time::now();													// update current_time
		
		NAVIGATION_ARRAY_inspector();												// check initialization status and update navigation_array_initialized
		NA_initialization_status.data = navigation_array_initialized;
		NA_initializer_pub.publish(NA_initialization_status);						// publish the initialization status of the navigation_array to "NA_initialization_state"
	  
		// publish whether or not goal waypoints have been converted and published for mission planner
		if (!NED_waypoints_published)
		{
		  goal_publish_status.data = false;
		}
		else
		{
		  goal_publish_status.data = true;
		}
		goal_publish_state_pub.publish(goal_publish_status);
		
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ START OF STANDARD USV POSE CONVERSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		if (NA_state == 1)														// standard USV Pose Conversion mode
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
		} // if USV Pose Conversion mode
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ END OF STANDARD USV POSE CONVERSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ START OF GOAL POSE CONVERSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		if ((!NED_waypoints_converted) && ((NA_state == 2) || (NA_state == 3)))						// if Goal Pose Conversion mode 
		{
			// first subscribe to goal poses dependent upon which task
			if (!lat_lon_goal_recieved)
			{
				if (NA_state == 2)						// TASK 1: STATION_KEEPING
				{
					VRX_T1_goal_sub = nh6.subscribe("/vrx/station_keeping/goal", 1, VRX_T1_goal_update);					// subscriber for goal pose given by Task1_SK
				}
				else if (NA_state == 3)				// TASK 2: WAYFINDING
				{
					VRX_T2_goal_sub = nh6.subscribe("/vrx/wayfinding/waypoints", 1, VRX_T2_goal_update);             	// subscriber for goal waypoints given by Task2_WF
				}
				ros::spinOnce();
				loop_rate.sleep();
			} // if (!lat_lon_goal_recieved)	
			
			if ((!lat_lon_point_sent)  && (lat_lon_goal_recieved))
			{
				// UPDATE NAV_ODOM MSG
				GPS_Position_update();
				GPS_Velocity_update();
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
				usvstate_pub.publish(nav_odom);
				lat_lon_point_sent = true;
			}
			
			// Update waypoint array in NED units
			if (point_num < goal_poses_quantity)
			{
				NED_x_goal[point_num] = xNED;
				NED_y_goal[point_num] = yNED;
				NED_psi_goal[point_num] = psiNED;
				
				WF_loops_sent += 1;
				if (WF_loops_sent == CONFIRM)
				{
					WF_loops_sent = 0;								// reset WF_loops_sent counter
					point_num += 1;
					lat_lon_point_sent = false;
				}
			}
			if (point_num == goal_poses_quantity) 							// means all coordinates have been converted
			{
				NED_waypoints_converted = true;
				ROS_INFO("WAYPOINTS HAVE BEEN CONVERTED");
			}
		} // if Goal Pose Conversion mode
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ START OF GOAL POSE CONVERSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			
		if ((NED_waypoints_converted) && (!NED_waypoints_published))
		{
			NED_waypoints.points.clear();
			
			NED_waypoints.quantity = goal_poses_quantity;        // publish quantity of poses so the high level control knows
			
			for (int i = 0; i < goal_poses_quantity; i++)
			{
				geometry_msgs::Point point;
				point.x = NED_x_goal[i];
				point.y = NED_y_goal[i];
				point.z = NED_psi_goal[i];
				NED_waypoints.points.push_back(point);
				ROS_INFO("point: %i", i);
				ROS_INFO("x: %f", NED_x_goal[i]);
				ROS_INFO("y: %f", NED_y_goal[i]);
				ROS_INFO("psi: %f", NED_psi_goal[i]);
			}
			waypoints_NED_pub.publish(NED_waypoints);
			NED_waypoints_published = true;
			ROS_INFO("WAYPOINTS HAVE BEEN PUBLISHED");
		} // if ((NED_waypoints_converted) && (!NED_waypoints_published))
		
		if (NED_waypoints_published)
		{
			waypoints_NED_pub.publish(NED_waypoints);
		}
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ END OF WF GOAL POSES CONVERSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		
		last_time = current_time;
		ros::spinOnce();
		loop_rate.sleep();
		loop_count += 1;
	}
	return 0;
} // end of main()