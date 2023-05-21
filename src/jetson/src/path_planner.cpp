
]//  Filename:											path_planner.cpp
//  Creation Date:									04/07/2022
//  Last Revision Date:						04/07/2022
//  Author(s) [email]:								Bradley Hacker [bhacker@lssu.edu]
//  Revisor(s) [email] {Revision Date}:	Bradley Hacker [bhacker@lssu.edu] {04/07/2022}
//  Organization/Institution:						Lake Superior State University - Team AMORE
//
// ...............................About path_planner.cpp......................................
//  This code acts as the autonomous state machine of the WAM-V USV.
//  It will subscribe to the vrx/task/info to control the state of the system.
//  This code will subscribe to goal poses given from the gps_imu node.
//  Dependent on the current task state and system state, mission_control
//  will publish whether or not the low level controllers should be on, as well
//  as the goal of the low level controllers.
//
//  Inputs and Outputs of the path_planner.cpp file
//				Inputs [subscribers]: "waypoints_NED" (converted goal pose array), "usv_ned", "/vrx/task/info",
//				Outputs [publishers]: state and goal of low level controllers


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"					// message type used for receiving NED USV state from navigation_array
#include "jetson/state_msg.h"					// message type used to recieve state of operation from mission_control
#include "std_msgs/Bool.h"					// message used to communicate publish state to propulsion_system
#include "jetson/usv_pose_msg.h"				// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "jetson/zed2i_msg.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
#include "jetson/NED_objects.h"  // message type that has an array of pointstamped
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    		// loop counter, first 10 loops used to intitialize subscribers

//	STATES CONCERNED WITH "navigation_array"
int NA_state = 0;	//	0 = On standby		//	1 = USV NED pose converter

//	STATES CONCERNED WITH "path_planner"
int PP_state = 0;		//	0 = On standby		//	1 = Task 1: Station-Keeping			//	2 = Task 2: Wayfinding

//	STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;		//	0 = On standby		//	1 = Propulsion system ON

// Task Number //From Task 0 to Task 9
int task;

int point = 0;                     		    		// running total of points reached on path
int goal_poses = 0;              				// total number of poses to reach in current path 
int loop_goal_recieved;  // this is kept in order to ensure planner doesn't start controller until the goal is published		// CHECK IF UNEEDED AFTER REBUILD
int goal_poses_quantity;  // total number of poses to reach

float x_goal[100], y_goal[100], psi_goal[100];			// arrays to hold the NED goal poses
float x_North, y_North, x_mid, y_mid, x_South, y_South;				// positions of North, mid, and South points for figure 8
float x_usv_NED, y_usv_NED, psi_NED; 				// vehicle position and heading (pose) in NED
float e_x, e_y, e_xy, e_psi;					// current errors between goal pose and usv pose

// initialize previous errors for calculating differential term
float e_x_prev = 0;
float e_y_prev = 0;
float e_xy_prev = 0;
float e_psi_prev = 0;

bool E_reached = false;        					// false means the last point has not been reached
bool calculations_done = false; 				// false means the array of waypoints have not been created
bool calculations_done_3 = false; 				// false means the array of waypoints have not been created
bool calculations_done_4 = false; 				// false means the array of waypoints have not been created

float e_xy_allowed = 3.0;       				// positional error tolerance threshold; NOTE: make as small as possible
float e_psi_allowed = 0.4;      				// heading error tolerance threshold; NOTE: make as small as possible
float position_tolerance;
//Array of poses for making the circle for the turtle
float x_N[5];
float y_N[5];
float psi_N[5];

//Array of poses for making the circle for the platypus
float x_S[5];
float y_S[5];
float psi_S[5];

float r = 6.0;									// [m] radius of circles around north and south points of figure 8
float r1 = 2.5;
float r2 = 5.5;

float theta_0 = 0.5;
float theta_usv, r_usv;

//std_msgs::String Object;			//String of what object the camera detects
//std_msgs::Int64 Obj_order;		//Order number of the object
//float x_obj;								//x value from zed2i local frame
//float y_obj;								//y value from zed2i local frame
//float z_obj;								//z value from zed2i local frame

// Task 3 Variables
std::string Object[9];  // string array to hold the names of the objects from the zed2i
float x_object_NED[9], y_object_NED[9];  // arrays to hold animal locations
float db1_USV, db2_USV, db3_USV , db4_USV, db5_USV;  // [m] distances between animals and USV
float alpha, alpha1,alpha2;							//Alpha angles
float beta1,beta2;									//Beta Angles 
float g,c;

//Task 4 Variables
std::string Animal[3];  // string array to hold the names of the animals
float x_animals_NED[3], y_animals_NED[3];  // arrays to hold animal locations

float x_c_NED, y_c_NED, psi_c_NED;  // crocodile position and heading (pose) in NED
float x_p_NED, y_p_NED, psi_p_NED;  // platypus position and heading (pose) in NED
float x_t_NED, y_t_NED, psi_t_NED;  // turtle position and heading (pose) in NED
float dc_USV, dt_USV, dp_USV , dp_c, dt_c;  // [m] distances between animals and USV
bool animal_usv_distances_calculated = false;  // false means the distances between the usv and the turtle and platypus have not been calculated


//Task 7
int Shoot = 0;

// Array of poses for making the turtle circle
float x_turt_g[9];
float y_turt_g[9];
float psi_turt_g[9];

// Array of poses for making the platypus circle
float x_plat_g[9];
float y_plat_g[9];
float psi_plat_g[9];

std_msgs::Bool pp_initialization_status;			// "pp_initialization_state" message
ros::Publisher pp_initialization_state_pub;			// "pp_initialization_state" publisher

std_msgs::Bool pp_USV_pose_update_status;			// "pp_USV_pose_update_state" message; false means NED usv pose has not been updated
ros::Publisher pp_USV_pose_update_state_pub;			// "pp_USV_pose_update_state" publisher

std_msgs::Bool goal_pose_publish_status;			// "goal_pose_publish_state" message; false means goal NED pose has not been published
ros::Publisher goal_pose_publish_state_pub;			// "goal_pose_publish_state" publisher

jetson::usv_pose_msg current_goal_pose_msg;			// "current_goal_pose" message
ros::Publisher current_goal_pose_pub;				// "current_goal_pose" publisher

std_msgs::Int32 Flinging_status;					//Flinging status
ros::Publisher Flinging_pub;												//Flinging publisher




ros::Time current_time, last_time;				// creates time variables
//..............................................................End of Global Variables..........................................................


//..................................................................Functions...........................................................................
// THIS FUNCTION UPDATES THE USV GOAL POSE TO STATION KEEP AT
// ACCEPTS NOTHING (VOID)
// RETURNS NOTHING (VOID)
// =====================================================
void parameters_function()
{
	if (PP_state == 1)	//	1 = Task 1: Station-Keeping
	{
		ros::param::get("/x_G", x_goal[0]);
		ros::param::get("/y_G", y_goal[0]);
		ros::param::get("/psi_G", psi_goal[0]);
		while ((psi_goal[0] < -PI) || (psi_goal[0] > PI))
		{
			// Adjust psi_goal[0] back within -PI and PI
			if (psi_goal[0] < -PI)
			{
				psi_goal[0] = psi_goal[0] + 2.0*PI;
			}
			if (psi_goal[0] > PI)
			{
				psi_goal[0] = psi_goal[0] - 2.0*PI;
			}
		}
		point = 0;
		goal_poses = 1;
	}
	ros::param::get("/x_north", x_North);
	ros::param::get("/y_north", y_North);
	ros::param::get("/x_south", x_South);
	ros::param::get("/y_south", y_South);
} // END OF parameters_function()

// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "pp_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void PATH_PLANNER_inspector()
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;				// increment loop counter
	parameters_function();
	if (loop_count > 5)
	{
		pp_initialization_status.data = true;
		//ROS_INFO("path_planner_initialized -- PP");
	}
	else
	{
		pp_initialization_status.data = false;
		ROS_INFO("!path_planner_initialized -- PP");
	}
	pp_initialization_state_pub.publish(pp_initialization_status);		// publish the initialization status of the path_planner to "pp_initialization_state"
	goal_pose_publish_state_pub.publish(goal_pose_publish_status);		// publish whether NED goal pose has been published to propulsion_system for mission_control to know when to turn propulsion_system ON
	pp_USV_pose_update_state_pub.publish(pp_USV_pose_update_status);	// publish whether or not NED usv pose has been updated
} // END OF PATH_PLANNER_inspector()
/////////////////////////////////////////////////////////////////		VISION UPDATES	///////////////////////////////////////////////////////////////////
// THIS FUNCTION: Update the object recognition from Zed2i
// ACCEPTS: jetson::zed2i_msg
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void object_rec(const jetson::zed2i_msg::ConstPtr& msg)
{
	for (int i = 0; i < msg->quantity; i++)
		{
		   Object[i] = msg->objects[i].header.frame_id;  // Getting array of object name. Based on on array location, the object is closer or farther from USV
		   x_object_NED[i] = msg->objects[i].point.x;  // Getting x position of objects 
		   y_object_NED[i] = msg->objects[i].point.y;  // Getting y position of objects
		   //ROS_INFO("PATH_PLANNER: Animal: %s    x: %4.2f    y: %4.2f", Animal[i].c_str(), x_animals_NED[i], y_animals_NED[i]);  // UPDATE USER
		}
	
} // END OF object_rec()

/////////////////////////////////////////////////////////////////		STATE UPDATERS		///////////////////////////////////////////////////////////////////
// THIS FUNCTION: Updates the state of "navigation_array" given by "mission_control"
// ACCEPTS: navigation_array state_msg from "na_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void na_state_update(const jetson::state_msg::ConstPtr& msg)
{
	if (pp_initialization_status.data)
	{
		NA_state = msg->state.data;
	}
} // END OF na_state_update()

// THIS FUNCTION: Updates the state of "path_planner" given by "mission_control"
// ACCEPTS: path_planner state_msg from "pp_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void pp_state_update(const jetson::state_msg::ConstPtr& msg)
{
	if (pp_initialization_status.data)
	{
		PP_state = msg->state.data;
	}
} // END OF pp_state_update()

// THIS FUNCTION: Updates the state of "propulsion_system" given by "mission_control"
// ACCEPTS: propulsion_system state_msg from "ps_state"
// RETURNS: (VOID)
// =============================================================================
void ps_state_update(const jetson::state_msg::ConstPtr& msg)
{
	if (pp_initialization_status.data)
	{
		PS_state = msg->state.data;
	}
} // END OF ps_state_update()
//////////////////////////////////////////////////////////////		STATE UPDATERS END		///////////////////////////////////////////////////////////////////

// THIS FUNCTION: Updates the current NED USV pose converted through the navigation_array
// ACCEPTS: Current NED USV pose and velocities from "nav_ned"
// RETURNS: (VOID)
// =============================================================================
void pose_update(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (NA_state == 1) // if navigation_array is in standard USV NED pose converter mode
	{
		// Update NED USV pose
		x_usv_NED = odom->pose.pose.position.x;
		y_usv_NED = odom->pose.pose.position.y;
		psi_NED = odom->pose.pose.orientation.z;
		pp_USV_pose_update_status.data = true;
	}
} // END OF pose_update()

// THIS FUNCTION: Fills out current_goal_pose_msg and publishes to "current_goal_pose" for the propulsion_system
// ACCEPTS: Nothing. Uses global variable pose arrays
// RETURNS: (VOID)
// =============================================================================
void current_goal_pose_publish()
{
	// PUBLISH THE CURRENT GOAL POSE
	current_goal_pose_msg.header.seq = 0;
	current_goal_pose_msg.header.seq +=1;				// sequence number
	current_goal_pose_msg.header.stamp = current_time;		// sets stamp to current time
	current_goal_pose_msg.header.frame_id = "mission_control";	// header frame
	current_goal_pose_msg.position.x = x_goal[point];		// sets x-location
	current_goal_pose_msg.position.y = y_goal[point];		// sets y-location
	current_goal_pose_msg.position.z = 0.0;				// sets z-location
	current_goal_pose_msg.psi.data = psi_goal[point];		// sets psi

	//ROS_INFO("Publishing point --PP");
	//ROS_INFO("Point x: %4.2f		Point y: %4.2f --PP", x_goal[point], y_goal[point]);

	current_goal_pose_pub.publish(current_goal_pose_msg);		// publish goal usv pose to "current_goal_pose"
	goal_pose_publish_status.data = true;
} // END OF current_goal_pose_publish()

// THIS FUNCTION: Generates the figure 8 pattern
// ACCEPTS: (VOID) Uses global North and South points
// RETURNS: (VOID) Populates array of poses
// =============================================================================
void calculate_path()
{
	// first calculate the midpoint
	x_mid = (x_North + x_South)/2.0;
	y_mid = (y_North + y_South)/2.0;

	// now fill array of poses
	point = 0;

	// first pose - mid point
	x_goal[point] = x_mid;
	y_goal[point] = y_mid;
	psi_goal[point] = 0.0;
	point++;

	// next 5 poses around North point
	// second pose - left
	x_goal[point] = x_North;
	y_goal[point] = y_North-r;
	psi_goal[point] = 0.0;
	point++;
	// third pose
	x_goal[point] = x_North+r*0.707107;
	y_goal[point] = y_North-r*0.707107;
	psi_goal[point] = PI/4.0;;
	point++;
	// fourth pose - top
	x_goal[point] = x_North+r;
	y_goal[point] = y_North;
	psi_goal[point] = PI/2.0;
	point++;
	// fifth pose
	x_goal[point] = x_North+r*0.707107;
	y_goal[point] = y_North+r*0.707107;
	psi_goal[point] = 3.0*PI/4.0;
	point++;
	// sixth pose - right
	x_goal[point] = x_North;
	y_goal[point] = y_North+r;
	psi_goal[point] = PI;
	point++;

	// seventh pose - mid point
//	x_goal[point] = x_mid;
//	y_goal[point] = y_mid;
//	psi_goal[point] = PI;
//	point++;

	// next 5 poses around North point
	// eighth pose - left
	x_goal[point] = x_South;
	y_goal[point] = y_South-r;
	psi_goal[point] = PI;
	point++;
	// ninth pose
	x_goal[point] = x_South-r*0.707107;
	y_goal[point] = y_South-r*0.707107;
	psi_goal[point] = 3.0*PI/4.0;
	point++;
	// tenth pose - bottom
	x_goal[point] = x_South-r;
	y_goal[point] = y_South;
	psi_goal[point] = PI/2.0;
	point++;
	// eleventh pose
	x_goal[point] = x_South-r*0.707107;
	y_goal[point] = y_South+r*0.707107;
	psi_goal[point] = PI/4.0;
	point++;
	// twelfth pose - right
	x_goal[point] = x_South;
	y_goal[point] = y_South+r;
	psi_goal[point] = 0.0;
	point++;

	// thirteenth pose - mid point
	x_goal[point] = x_mid;
	y_goal[point] = y_mid;
	psi_goal[point] = 1.57;
	point++;

	goal_poses = point;
	point = 0;		// reset to start by feeding first point
	calculations_done = true;
} // end of calculate_path()
// THIS FUNCTION: Generates the path for Task 3
// ACCEPTS: (VOID) Uses global North and South points
// RETURNS: (VOID) Populates array of poses
// =============================================================================
void Task3_Path ()
{
	y_object_NED[0]+=1;			//Conversion to y coordinate from the GPS not the camera
	y_object_NED[1]+=1;			//Conversion to y coordinate from the GPS not the camera
	
	alpha1 = atan2(x_object_NED[0], y_object_NED[0]);		//Get angles of the first rotation to bouy 1
	alpha2 = atan2(x_object_NED[1], y_object_NED[1]);		//Get angles of the first rotation to bouy 2
	
	db1_USV = x_object_NED[0]/sin(alpha1);			//Get distance between USV and bouy 1
	db2_USV = x_object_NED[1]/sin(alpha2);			//Get distance between USV and bouy 2
	
	alpha = abs(alpha1) + abs(alpha2);						// Angle between USV. and both bouys
	
	c = sqrt(pow(db1_USV,2)+ pow(db2_USV,2)- 2*db1_USV*db2_USV*cos(alpha));
	beta1 = asin(db2_USV*sin(alpha)/10);					//Assuming both buoys are between 10 m
	beta2 = asin(db1_USV*sin(alpha)/10);					//Assuming both buoys are between 10 m (We could get the distance doing sqrt((db1)^2 + (db2)^2 - 2*db1*db2*cos(alpha))
	
	g = (c/2)*sin(beta1)/sin(alpha/2);								//Get distance to the center between buoys
	
	x_goal[1] = g*sin(alpha/2)+x_usv_NED;				//Final Goal
	y_goal[1] = g*sin(alpha/2)+y_usv_NED + 3;			//Final Goal pose
	psi_goal[1] = alpha1;										//Angle from USV to the closest bouy
	
	x_goal[0];						//Make 1 or 2 intermidiate points
	y_goal[0];					// 
	psi_goal[0]; 
	calculations_done_3= true;
} //end of Task3_Path

// TASK 4: Wildlife Encounter and Avoid
// THIS FUNCTION: Updates the converted local NED positions of the animals
// ACCEPTS: jetson::NED_objects from "NED_animals"
// RETURNS: (VOID)
//=============================================================================================================
void CC_animals_ned_update(const jetson::NED_objects::ConstPtr& object)
{
	//if (!CC_goal_recieved)  // if the NED goal waypoints have been published but not recieved yet
	//{
		//ROS_INFO("PATH_PLANNER: GOAL ANIMAL POSES ACQUIRED BY PLANNER.");  // UPDATE USER
		for (int i = 0; i < object->quantity; i++)
		{
		   Animal[i] = object->objects[i].header.frame_id;  // Getting array of animal names
		   if (Animal[i] == "crocodile")
		   {
			   x_c_NED = object->objects[i].point.x;
			   y_c_NED = object->objects[i].point.y;
		   }
		   if (Animal[i] == "turtle")
		   {
			   x_t_NED = object->objects[i].point.x;
			   y_t_NED = object->objects[i].point.y;
		   }
		   if (Animal[i] == "platypus")
		   {
			   x_p_NED = object->objects[i].point.x;
			   y_p_NED = object->objects[i].point.y;
		   }
		   x_animals_NED[i] = object->objects[i].point.x;  // Getting x position of animals 
		   y_animals_NED[i] = object->objects[i].point.y;  // Getting y position of animals
		   //ROS_INFO("PATH_PLANNER: Animal: %s    x: %4.2f    y: %4.2f", Animal[i].c_str(), x_animals_NED[i], y_animals_NED[i]);  // UPDATE USER
		}
		loop_goal_recieved = loop_count;
		//if ((x_t_NED != y_t_NED) || (x_t_NED != y_p_NED) || (x_t_NED != x_p_NED))
		//{
		//	CC_goal_recieved = true;  // true means NED goal poses have been acquired from coordinate_converter
		//}
	//}  // END OF if (!CC_goal_recieved)
}  // END OF CC_animals_ned_update()

// THIS FUNCTION: Calculates the distances between the USV and the animals
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void animal_distances_calculate()
{
	dc_USV = sqrt(pow(x_usv_NED - x_c_NED, 2.0)+pow(y_usv_NED - y_c_NED, 2.0));  // Distance from USV to crocodile
	dp_USV = sqrt(pow(x_usv_NED - x_p_NED, 2.0)+pow(y_usv_NED - y_p_NED, 2.0));  // Distance from USV to platypus
	dt_USV = sqrt(pow(x_usv_NED - x_t_NED, 2.0)+pow(y_usv_NED - y_t_NED, 2.0));  // Distance from USV to turtle
	dt_c = sqrt(pow(x_t_NED - x_c_NED, 2.0)+pow(y_t_NED - y_c_NED, 2.0));  // Distance from turtle to crocodile
	dp_c  = sqrt(pow(x_p_NED - x_c_NED, 2.0)+pow(y_p_NED - y_c_NED, 2.0));  // Distance from platypus to crocodile
	
	ROS_INFO("PATH_PLANNER:----DISTANCES CALCULATED----");
	ROS_INFO("PATH_PLANNER: x_USV: %4.2f     y_USV: %4.2f", x_usv_NED, y_usv_NED);
	ROS_INFO("PATH_PLANNER: x_plat: %4.2f    y_plat: %4.2f", x_p_NED, y_p_NED);
	ROS_INFO("PATH_PLANNER: x_turt: %4.2f    y_turt: %4.2f", x_t_NED, y_t_NED);
	ROS_INFO("PATH_PLANNER: dp_USV: %4.2f     dt_USV: %4.2f", dp_USV, dt_USV);
	ROS_INFO("PATH_PLANNER:----DISTANCES CALCULATED----\n");
	animal_usv_distances_calculated = true;  // true means the distances between the usv and the turtle and platypus have been calculated
}  // END OF animal_distances_calculate()

// THIS FUNCTION: Generates the updated array of poses to accomplish the task
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void update_animal_path()
{
	if (!animal_usv_distances_calculated)  // if the distances between the USV and the animals has not yet been calculated
	{
		animal_distances_calculate();  // updates distances from USV to each animal
	}
	else if (animal_usv_distances_calculated)
	{
		r = 5;
		// Making the circle ccw around turtle
		// NOTE: currently there is 3.83 [m] between each point
		x_turt_g[0] = x_t_NED - r;
		y_turt_g[0] = y_t_NED;
		psi_turt_g[0] = 90.0 * (PI/180);
		x_turt_g[1] = x_t_NED - (sqrt(2.0)/2.0)*r;
		y_turt_g[1] = y_t_NED + (sqrt(2.0)/2.0)*r;
		psi_turt_g[1] = 45.0 * (PI/180);
		x_turt_g[2] = x_t_NED;
		y_turt_g[2] = y_t_NED + r;
		psi_turt_g[2] = 0.0 * (PI/180);
		x_turt_g[3] = x_t_NED + (sqrt(2.0)/2.0)*r;
		y_turt_g[3] = y_t_NED + (sqrt(2.0)/2.0)*r;
		psi_turt_g[3] = -45.0 * (PI/180);
		x_turt_g[4] = x_t_NED + r;
		y_turt_g[4] = y_t_NED;
		psi_turt_g[4] = -90.0 * (PI/180);
		x_turt_g[5] = x_t_NED + (sqrt(2.0)/2.0)*r;
		y_turt_g[5] = y_t_NED - (sqrt(2.0)/2.0)*r;
		psi_turt_g[5] = -135.0 * (PI/180);
		x_turt_g[6] = x_t_NED;
		y_turt_g[6] = y_t_NED - r;
		psi_turt_g[6] = 180.0 * (PI/180);
		x_turt_g[7] = x_t_NED - (sqrt(2.0)/2.0)*r;
		y_turt_g[7] = y_t_NED - (sqrt(2.0)/2.0)*r;
		psi_turt_g[7] = 135.0 * (PI/180);
		x_turt_g[8] = x_t_NED - r;
		y_turt_g[8] = y_t_NED;
		psi_turt_g[8] = 90.0 * (PI/180);  // change to point away from animal 

		// Making the circle cw around platypus
		x_plat_g[0] = x_p_NED + r;
		y_plat_g[0] = y_p_NED;
		psi_plat_g[0] = 90.0 * (PI/180.0);
		x_plat_g[1] = x_p_NED + (sqrt(2.0)/2.0)*r;
		y_plat_g[1] = y_p_NED + (sqrt(2.0)/2.0)*r;
		psi_plat_g[1] = 135.0 * (PI/180);
		x_plat_g[2] = x_p_NED;
		y_plat_g[2] = y_p_NED + r;
		psi_plat_g[2] = -180.0 * (PI/180);
		x_plat_g[3] = x_p_NED - (sqrt(2.0)/2.0)*r;
		y_plat_g[3] = y_p_NED + (sqrt(2.0)/2.0)*r;
		psi_plat_g[3] = -135.0 * (PI/180);
		x_plat_g[4] = x_p_NED - r;
		y_plat_g[4] = y_p_NED;
		psi_plat_g[4] = -90.0 * (PI/180);
		x_plat_g[5] = x_p_NED - (sqrt(2.0)/2.0)*r;
		y_plat_g[5] = y_p_NED - (sqrt(2.0)/2.0)*r;
		psi_plat_g[5] = -45.0 * (PI/180);
		x_plat_g[6] = x_p_NED;
		y_plat_g[6] = y_p_NED - r;
		psi_plat_g[6] = 0.0 * (PI/180);
		x_plat_g[7] = x_p_NED + (sqrt(2.0)/2.0)*r;
		y_plat_g[7] = y_p_NED - (sqrt(2.0)/2.0)*r;
		psi_plat_g[7] = 45.0 * (PI/180);
		x_plat_g[8] = x_p_NED + r;
		y_plat_g[8] = y_p_NED;
		psi_plat_g[8] = 90.0 * (PI/180);

		if (dt_USV <= dp_USV)  // if the distance between the USV and turtle is less than or equal to the distance between the USV and the platypus
		{
			for (int i=0; i<9; i++)
			{
				// go to the turtle first
				x_goal[i] = x_turt_g[i];
				y_goal[i] = y_turt_g[i];
				psi_goal[i] = psi_turt_g[i];
			}
			//ROS_INFO("x_t_NED: %4.2f    y_t_NED: %4.2f    x_p_NED: %4.2f    y_p_NED: %4.2f", x_t_NED, y_t_NED, x_p_NED, y_p_NED);

			for (int i=9; i<18; i++)
			{
				// go to the platypus second
				x_goal[i] = x_plat_g[i-9];
				y_goal[i] = y_plat_g[i-9];
				psi_goal[i] = psi_plat_g[i-9];
			}
		}
		else  // if the distance between the USV and turtle is greater than the distance between the USV and the platypus
		{
			for (int i=0; i<9; i++)
			{
				// go to the platypus first
				x_goal[i] = x_plat_g[i];
				y_goal[i] = y_plat_g[i];
				psi_goal[i] = psi_plat_g[i];
			}
			for (int i=9; i<18; i++)
			{
				// go to the turtle second
				x_goal[i] = x_turt_g[i-9];
				y_goal[i] = y_turt_g[i-9];
				psi_goal[i] = psi_turt_g[i-9];
			}
		}
		calculations_done_4 = true;
		goal_poses_quantity = 18;
	}
}  // END OF update_animal_path()

//THIS FUNCTION:
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
//............................................................End of Functions............................................................

int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "path_planner");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8,nh9,nh10,nh11,nh12,nh13,nh14,nh15,nh16;

	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, na_state_update);
	ros::Subscriber pp_state_sub = nh2.subscribe("pp_state", 1, pp_state_update);
	ros::Subscriber ps_state_sub = nh3.subscribe("ps_state", 1, ps_state_update);
	ros::Subscriber nav_NED_sub = nh4.subscribe("nav_ned", 1, pose_update);				// Obtains the USV pose in global NED from mission_control
	ros::Subscriber obj_rec_sub = nh9.subscribe("obj_rec", 1, object_rec);						//Obtains what object the camera recognise, pose of the object and order of recognition
	
	// Publishers
	pp_initialization_state_pub = nh5.advertise<std_msgs::Bool>("pp_initialization_state", 1);	// publisher for state of initialization
	current_goal_pose_pub = nh6.advertise<jetson::usv_pose_msg>("current_goal_pose", 1);		// current goal for low level controller (propulsion_system)
	goal_pose_publish_state_pub = nh7.advertise<std_msgs::Bool>("goal_pose_publish_state", 1);	// "goal_pose_publish_state" publisher for whether NED converted waypoints have been published
	pp_USV_pose_update_state_pub = nh8.advertise<std_msgs::Bool>("pp_USV_pose_update_state", 1);	// publisher for whether usv NED pose has been updated or not
	//Flinging_pub = nh10.advertise<std_msgs::Int32>("shoot_ball",1);
	ros::Publisher Vertical_pub = nh11.advertise<std_msgs::Int32>("Vertical_dist", 10);			// speed to right thruster
	ros::Publisher Horizontal_pub = nh12.advertise<std_msgs::Int32>("Horizontal_dist", 10);			// speed to left thruster
	ros::Publisher Adjustment_pub = nh13.advertise<std_msgs::Int32>("Adjustment_dist", 10);			// angle to right thruster
	ros::Publisher Speed_pub = nh14.advertise<std_msgs::Int32>("Speed_shooter", 10);				// angle to left thruster
	ros::Publisher Angle_shoot_pub = nh15.advertise<std_msgs::Int32>("Angle_shoot", 10);				
	ros::Publisher Ready_shoot_pub = nh16.advertise<std_msgs::Int32>("Ready_shoot", 10);				// angle to left thruster

	// Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize simulation time
	ros::Time::init();

	// Initialize global variables
	goal_pose_publish_status.data = false;
	pp_USV_pose_update_status.data = false;
	pp_initialization_status.data = false;
	Flinging_status.data = 1;
	current_time = ros::Time::now();							// sets current time to the time it is now
	last_time = current_time;								// sets last time to the current_time
	
	//Local variables 
	std_msgs::Int32 Vert_targ, Horiz_targ, Adj_shoot, Speed_shoot;									// LT is left thrust, RT is right thrust, LA is left thruster angle, RA is right thruster angle

	//sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(5);		// was 50

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		PATH_PLANNER_inspector();							// check that entire system is initialized before starting calculations

		//	0 = On standby
		//	1 = Station-Keeping
		//	2 = Wayfinding
		switch(PP_state)
		{
			case 0:						// On standby
				// reset all variables to be used for next run
				goal_pose_publish_status.data = false;
				E_reached = false;
				calculations_done = false;
				point = 0;
				goal_poses = 0;
				break;
			case 1:						// Station-Keeping
				current_goal_pose_publish();
				break;
			case 2:						// Wayfinding
				if (calculations_done)
				{
					if ((NA_state == 1) && (PS_state == 1))		// if the navigation_array is providing NED USV state, and the propulsion_system is ON
					{
						e_x_prev = e_x;
						e_y_prev = e_y;
						e_xy_prev = e_xy;
						e_psi_prev = e_psi;
						// determine error in x and y (position)
						e_x = x_goal[point] - x_usv_NED;                                       // calculate error in x position
						e_y = y_goal[point] - y_usv_NED;                                       // calculate error in y position
						e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
						e_psi = psi_goal[point] - psi_NED;

						while ((e_psi < -PI) || (e_psi > PI))
						{
							// Adjust e_psi back within -PI and PI
							if (e_psi < -PI)
							{
								e_psi = e_psi + 2.0*PI;
							}
							if (e_psi > PI)
							{
								e_psi = e_psi - 2.0*PI;
							}
						}

						if ((point == 0) || (point == 6))
						{
							position_tolerance = e_xy_allowed*1.5;
						}
						else
						{
							position_tolerance = e_xy_allowed;
						}
						//float sign_change_check = e_x/e_x_prev;
						if ((e_xy < e_xy_allowed) && (!E_reached))	//&& (abs(e_psi) < e_psi_allowed) || ((sign_change_check < 0) && (x_goal[point] == 0))
						{
							point += 1;
							ROS_INFO("Point %i of %i reached. --MC", point, goal_poses);
							if (point==goal_poses)
							{
							  E_reached = true;
							  ROS_INFO("End point has been reached. --MC\n");
							}
						}

						/* // New method for feeding point
						r_usv = sqrt(pow(e_x,2.0)+pow(e_y,2.0));
						if ((e_xy < e_xy_allowed) && (!E_reached))	//  && (abs(e_psi) < e_psi_allowed)  || ((sign_change_check < 0) && (x_goal[point] == 0))
						{
							point += 1;
							ROS_INFO("Point %i of %i reached. --MC", point, goal_poses);
							if (point==goal_poses)
							{
							  E_reached = true;
							  ROS_INFO("End point has been reached. --MC\n");
							}
						} */

						if (E_reached)
						{
							point = 0;			// station-keep at first point in array
						}
					} // if ((NA_state == 1) && (PS_state == 1))
					current_goal_pose_publish();
				}
				
				else
				{
					calculate_path();																// this function will generate the updated array of poses
					E_reached = false;
					goal_pose_publish_status.data = false;
				}
				break;
			case 3:          																		// Task 3: Follow the path
			{
					if (calculations_done_3)
					{
						if ((NA_state == 1) && (PS_state == 1))		// if the navigation_array is providing NED USV state, and the propulsion_system is ON
						{
						e_x_prev = e_x;
						e_y_prev = e_y;
						e_xy_prev = e_xy;
						e_psi_prev = e_psi;
						// determine error in x and y (position)
						e_x = x_goal[point] - x_usv_NED;                                       // calculate error in x position
						e_y = y_goal[point] - y_usv_NED;                                       // calculate error in y position
						e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
						e_psi = psi_goal[point] - psi_NED;

						while ((e_psi < -PI) || (e_psi > PI))
							{
							// Adjust e_psi back within -PI and PI
							if (e_psi < -PI)
								{
								e_psi = e_psi + 2.0*PI;
								}
							if (e_psi > PI)
								{
								e_psi = e_psi - 2.0*PI;
								}
							}

						if ((point == 0) || (point == 6))										//Xavi: I do not think we need this
							{
							position_tolerance = e_xy_allowed*1.5;
							}
						else
							{
							position_tolerance = e_xy_allowed;
							}
						//float sign_change_check = e_x/e_x_prev;
						if ((e_xy < e_xy_allowed) && (!E_reached))	//&& (abs(e_psi) < e_psi_allowed) || ((sign_change_check < 0) && (x_goal[point] == 0))
						{
							point += 1;
							ROS_INFO("Point %i of %i reached. --MC", point, goal_poses);
							if (point==goal_poses)
							{
							  E_reached = true;
							  ROS_INFO("End point has been reached. --MC\n");
							  calculations_done_3 = false;
							}
						}
					}
				}	
			}
			case 4:
			{
				if (calculations_done_4)
				{
					if ((NA_state == 1) && (PS_state == 1))		// if the navigation_array is providing NED USV state, and the propulsion_system is ON
						{
						e_x_prev = e_x;
						e_y_prev = e_y;
						e_xy_prev = e_xy;
						e_psi_prev = e_psi;
						// determine error in x and y (position)
						e_x = x_goal[point] - x_usv_NED;                                       // calculate error in x position
						e_y = y_goal[point] - y_usv_NED;                                       // calculate error in y position
						e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
						e_psi = psi_goal[point] - psi_NED;

						while ((e_psi < -PI) || (e_psi > PI))
							{
							// Adjust e_psi back within -PI and PI
							if (e_psi < -PI)
								{
								e_psi = e_psi + 2.0*PI;
								}
							if (e_psi > PI)
								{
								e_psi = e_psi - 2.0*PI;
								}
							}

						if ((point == 0) || (point == 8)|| (point == 17))											
							{
							position_tolerance = e_xy_allowed*1.5;
							}
						else
							{
							position_tolerance = e_xy_allowed;
							}
						//float sign_change_check = e_x/e_x_prev;
						if ((e_xy < e_xy_allowed) && (!E_reached))	//&& (abs(e_psi) < e_psi_allowed) || ((sign_change_check < 0) && (x_goal[point] == 0))
						{
							point += 1;
							ROS_INFO("Point %i of %i reached. --MC", point, goal_poses);
							if (point==goal_poses)
							{
							  E_reached = true;
							  ROS_INFO("End point has been reached. --MC\n");
							  calculations_done_4 = false;
							}
						}
				}	}
			}
			case 5:
			{
					current_goal_pose_publish();
			}
			case 6:
			{
			}
			case 7:
			{
				if (Shoot==1)
				{
					Flinging_status.data = 1;
				}
				
				Vert_targ.data = 0;
				Horiz_targ.data = 0;
				Adj_shoot.data = 0;
				Speed_shoot.data = 0;
				Vertical_pub.publish(Vert_targ);
				Horizontal_pub.publish(Horiz_targ);
				Adjustment_pub.publish(Adj_shoot);
				Speed_pub.publish(Speed_shoot);
				Flinging_pub.publish(Flinging_status);
				
					
			}
			default:
				break;
		}	// switch(PP_state)

		ros::spinOnce();					// update subscribers
		loop_rate.sleep();				// sleep for set loop_rate
		last_time = current_time;	// update last_time
	} // while(ros::ok())

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}
