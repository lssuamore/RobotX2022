//  Filename:											path_planner.cpp
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
#include "nav_msgs/Path.h"  // message type that has an array of pointstamped
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
#define max_next_position_distance 5.0  // [m] this is the maximum next position distance from the USV position to place a goal pose
#define red_buoy "mb_marker_buoy_red"  // Red marker buoy
#define green_buoy "mb_marker_buoy_green"  // Green marker buoy
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

//float x_goal[100], y_goal[100], psi_goal[100];			// arrays to hold the NED goal poses
float x_goal_poses[100], y_goal_poses[100], psi_goal_poses[100];  // arrays to hold the NED goal poses for tasks
float x_North, y_North, x_mid, y_mid, x_South, y_South;				// positions of North, mid, and South points for figure 8
float x_usv_NED, y_usv_NED, psi_NED,psi_usv_NED; 				// vehicle position and heading (pose) in NED
float e_x, e_y, e_xy, e_psi;					// current errors between goal pose and usv pose
float x_goal_pose, y_goal_pose, psi_goal_pose;  // current goal pose to be published to propulsion_system
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

//------------------------------------CHANNEL NAVIGATION--------------------------------
float x_gate,y_gate;
float x_gate_prev,y_gate_prev;
std::string gate;		  // string array to hold the names of the objects from the zed2i
float alpha;
bool same_point = true;
float x_min_entr,x_max_entr,y_min_entr,y_max_entr;
float x_min_speed,x_max_speed,y_min_speed,y_max_speed;

//----------------------------END OF CHANNEL NAVIGATION VARIABLES-----------------------
//Task 4 Variables
std::string Animal[3];  // string array to hold the names of the animals
float x_animals_NED[3], y_animals_NED[3];  // arrays to hold animal locations

float x_c_NED, y_c_NED, psi_c_NED;  // crocodile position and heading (pose) in NED
float x_p_NED, y_p_NED, psi_p_NED;  // platypus position and heading (pose) in NED
float x_t_NED, y_t_NED, psi_t_NED;  // turtle position and heading (pose) in NED
float dc_USV, dt_USV, dp_USV , dp_c, dt_c;  // [m] distances between animals and USV
bool animal_usv_distances_calculated = false;  // false means the distances between the usv and the turtle and platypus have not been calculated




// Array of poses for making the turtle circle
float x_turt_g[9];
float y_turt_g[9];
float psi_turt_g[9];

// Array of poses for making the platypus circle
float x_plat_g[9];
float y_plat_g[9];
float psi_plat_g[9];

//--------------------------------------------BRAD'S STUFF---------------------------------------------------
//	STATES CONCERNED WITH STATE 1 - "DnD_path_planner"
//  1 = Start point search and approach
//  2 = Search for entrance buoy pair
//  3 = Going to calculated approach- and mid- points of entrance buoy pair
//	4 = Start search and approach for exit buoy pair while continuing on current updated path
//	5 = Exit point has been reached and task is complete
int DnD_path_planner;

//	STATES CONCERNED WITH STATE 1 OF  "FtP_path_planner"
//  1 = Start point search and approach
//  2 = Search for entrance buoy pair
//  3 = Going to calculated approach- and mid- points of entrance buoy pair
//	4 = Start search and approach for exit buoy pair while continuing on current updated path
//	5 = Exit point has been reached and task is complete
int FtP_path_planner;
int buoy_pairs_found = 0;  // this keeps track of the number of times the buoy pairs have been found and had waypoints calculated to navigate through them

int buoy_pair_updated;  // 0 = false  1 = true
float x_DnD_start, y_DnD_start, psi_DnD_start;  // start location for the dynamic navigation demonstration
float x_FtP_start, y_FtP_start, psi_FtP_start;  // start location for the follow the path task

bool params_updated = false;  // false means that the params for current task aren't ready to be GOTTED
bool params_GOT = false;  // false means that the params for current task haven't done been GOTTED

bool M_reached = false;  // false means the mid-point has not been reached for travelling through pair of buoys
// VARIABLES FOR THE BUOY NAVIGATION CALCULATIONS - calculate_buoy_waypoints()
// Centroid locations of left and right buoys given by "perception_array"
float CL_x;  // x-location of left buoy wrt USV
float CL_y;  // y-location of left buoy wrt USV
float CR_x;  // x-location of right buoy wrt USV
float CR_y;  // y-location of right buoy wrt USV

float CL_x_NED;  // x-location of left buoy centroid in global frame
float CL_y_NED;  // y-location of left buoy centroid in global frame
float CR_x_NED;  // x-location of right buoy centroid in global frame
float CR_y_NED;  // y-location of right buoy centroid in global frame

float I_x;  // x-coord. of intermediate point wrt global
float I_y;  // y-coord. of intermediate point wrt global

float M_x;  // x-location of midpoint
float M_y;  // y-location of midpoint

float E_x;  // x-coord. of exit point wrt global
float E_y;  // y-coord. of exit point wrt global

float d_L;  // distance from USV to left buoy
float d_M;  // distance from USV to midpoint
float d_R;  // distance from USV to right buoy
float d_I;  // distance from midpoint to approach point

float d_LM;  // half distance between left and right buoys
float a_L;  // distance between left buoy and the approach point
float theta;  // angle created by d_I and d_LM

float x_I_CL;  // x-coord. of intermediate point wrt left buoy
float y_I_CL;  // y-coord. of intermediate point wrt left buoy

float x_E_CL;  // x-coord. of exit point wrt left buoy
float y_E_CL;  // y-coord. of exit point wrt left buoy

float s_M;  // slope of of line from CL to CR
//float alpha;  // angle of frame CL wrt global frame

bool buoy_positions_recieved = false; // false means the NED_buoys array has not yet been acquired USED FOR BUOY NAVIGATION POSSIBLY
bool left_buoy_found = false;  // false means that the left buoy position was not found
bool right_buoy_found = false;  // false means that the right buoy position was not found

//	STATES CONCERNED WITH "navigation_array"
std_msgs::Bool pp_initialization_status;			// "pp_initialization_state" message
ros::Publisher pp_initialization_state_pub;			// "pp_initialization_state" publisher

std_msgs::Bool pp_USV_pose_update_status;			// "pp_USV_pose_update_state" message; false means NED usv pose has not been updated
ros::Publisher pp_USV_pose_update_state_pub;			// "pp_USV_pose_update_state" publisher

std_msgs::Bool goal_pose_publish_status;			// "goal_pose_publish_state" message; false means goal NED pose has not been published
ros::Publisher goal_pose_publish_state_pub;			// "goal_pose_publish_state" publisher

jetson::usv_pose_msg current_goal_pose_msg;			// "current_goal_pose" message
ros::Publisher current_goal_pose_pub;				// "current_goal_pose" publisher

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
		ros::param::get("/x_G", x_goal_poses[0]);
		ros::param::get("/y_G", y_goal_poses[0]);
		ros::param::get("/psi_G", psi_goal_poses[0]);
		while ((psi_goal_poses[0] < -PI) || (psi_goal_poses[0] > PI))
		{
			// Adjust psi_goal[0] back within -PI and PI
			if (psi_goal_poses[0] < -PI)
			{
				psi_goal_poses[0] = psi_goal_poses[0] + 2.0*PI;
			}
			if (psi_goal_poses[0] > PI)
			{
				psi_goal_poses[0] = psi_goal_poses[0] - 2.0*PI;
			}
		}
		point = 0;
		goal_poses = 1;
	}

	if (PP_state == 3)
	{
		ros::param::get("/x_min_entr", x_min_entr);
		ros::param::get("/x_max_entr", x_max_entr);
		ros::param::get("/y_min_entr", y_min_entr);
		ros::param::get("/y_max_entr", y_max_entr);

		ros::param::get("/x_min_speed", x_min_speed);
		ros::param::get("/x_max_speed", x_max_speed);
		ros::param::get("/y_min_speed", y_min_speed);
		ros::param::get("/y_max_speed", y_max_speed);
	}
	
	//----------------------------------BRAD'S STAFF-----------------------------
	if (PP_state == 6)
	{
		ros::param::get("/x_DnD", x_DnD_start);
		ros::param::get("/y_DnD", y_DnD_start);
		ros::param::get("/psi_DnD", psi_DnD_start);
		ros::param::get("/x_FtP", x_FtP_start);
		ros::param::get("/y_FtP", y_FtP_start);
		ros::param::get("/psi_FtP", psi_FtP_start);
	}
	
	ros::param::get("/updated_buoy_pair", buoy_pair_updated);
	if (buoy_pair_updated == 1)
	{
		ros::param::get("/x_LB", CL_x);
		ros::param::get("/y_LB", CL_y);
		ros::param::get("/x_RB", CR_x);
		ros::param::get("/y_RB", CR_y);
		buoy_positions_recieved = true;
		ros::param::set("/updated_buoy_pair", 0);
	} //                                                             _/_                                                             
	// The Scarecrow watches, his position is unpredictable          _()_          
	//                                                             // !! \\               
	// CATION: "MC" must be set for parameters to be updated successfully ^   
	ros::param::set("/params_Updated", false);  // reset status ensuring oneshot functionality    

	// Figure 8
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
		psi_usv_NED = odom->pose.pose.orientation.z;
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
	current_goal_pose_msg.position.x = x_goal_poses[point];		// sets x-location
	current_goal_pose_msg.position.y = y_goal_poses[point];		// sets y-location
	current_goal_pose_msg.position.z = 0.0;				// sets z-location
	current_goal_pose_msg.psi.data = psi_goal_poses[point];		// sets psi

	//ROS_INFO("Publishing point --PP");
	ROS_DEBUG("Pointer = %d Point x: %4.2f		Point y: %4.2f --PP",point, x_goal_poses[point], y_goal_poses[point]);

	current_goal_pose_pub.publish(current_goal_pose_msg);		// publish goal usv pose to "current_goal_pose"
	goal_pose_publish_status.data = true;
} // END OF current_goal_pose_publish()

// THIS FUNCTION: Calculates pose errors
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void calculate_pose_errors()
{
	e_x = x_goal_pose - x_usv_NED;  // calculate error in x position
	e_y = y_goal_pose - y_usv_NED;  // calculate error in y position
	e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));  // calculate magnitude of positional error
	e_psi = psi_goal_pose - psi_usv_NED;  // calculate error in heading
	while ((e_psi < -PI) || (e_psi > PI))
	{
		// Adjust e_psi back within -PI and PI
		if (e_psi < -PI)
		{
			e_psi += 2.0*PI;
		}
		if (e_psi > PI)
		{
			e_psi -= 2.0*PI;
		}
	}
}  // END OF calculate_pose_errors()

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
	x_goal_poses[point] = x_mid;
	y_goal_poses[point] = y_mid;
	psi_goal_poses[point] = 0.0;
	point++;

	// next 5 poses around North point
	// second pose - left
	x_goal_poses[point] = x_North;
	y_goal_poses[point] = y_North-r;
	psi_goal_poses[point] = 0.0;
	point++;
	// third pose
	x_goal_poses[point] = x_North+r*0.707107;
	y_goal_poses[point] = y_North-r*0.707107;
	psi_goal_poses[point] = PI/4.0;;
	point++;
	// fourth pose - top
	x_goal_poses[point] = x_North+r;
	y_goal_poses[point] = y_North;
	psi_goal_poses[point] = PI/2.0;
	point++;
	// fifth pose
	x_goal_poses[point] = x_North+r*0.707107;
	y_goal_poses[point] = y_North+r*0.707107;
	psi_goal_poses[point] = 3.0*PI/4.0;
	point++;
	// sixth pose - right
	x_goal_poses[point] = x_North;
	y_goal_poses[point] = y_North+r;
	psi_goal_poses[point] = PI;
	point++;

	// seventh pose - mid point
//	x_goal[point] = x_mid;
//	y_goal[point] = y_mid;
//	psi_goal[point] = PI;
//	point++;

	// next 5 poses around North point
	// eighth pose - left
	x_goal_poses[point] = x_South;
	y_goal_poses[point] = y_South-r;
	psi_goal_poses[point] = PI;
	point++;
	// ninth pose
	x_goal_poses[point] = x_South-r*0.707107;
	y_goal_poses[point] = y_South-r*0.707107;
	psi_goal_poses[point] = 3.0*PI/4.0;
	point++;
	// tenth pose - bottom
	x_goal_poses[point] = x_South-r;
	y_goal_poses[point] = y_South;
	psi_goal_poses[point] = PI/2.0;
	point++;
	// eleventh pose
	x_goal_poses[point] = x_South-r*0.707107;
	y_goal_poses[point] = y_South+r*0.707107;
	psi_goal_poses[point] = PI/4.0;
	point++;
	// twelfth pose - right
	x_goal_poses[point] = x_South;
	y_goal_poses[point] = y_South+r;
	psi_goal_poses[point] = 0.0;
	point++;

	// thirteenth pose - mid point
	x_goal_poses[point] = x_mid;
	y_goal_poses[point] = y_mid;
	psi_goal_poses[point] = 1.57;
	point++;

	goal_poses = point;
	point = 0;		// reset to start by feeding first point
	calculations_done = true;
} // end of calculate_path()

//---------------------------------------------CHANNEL NAVIGATION-------------------------------------------
// THIS FUNCTION: Generates the path for Task 3
// ACCEPTS: (VOID) Uses global North and South points
// RETURNS: (VOID) Populates array of poses
// =============================================================================
void Task3_Path()
{
     
	if(same_point)
	{
		//x_goal_poses[0] = x_usv_NED + cos(psi_usv_NED)*1;
        	//y_goal_poses[0] = y_usv_NED +  sin(psi_usv_NED)*1;
        	//psi_goal_poses[0]= psi_usv_NED;
	}
	else
	{    
		
			alpha = atan2(y_gate,x_gate);
			psi_goal_pose= psi_usv_NED + alpha;
			x_goal_pose = x_usv_NED + cos(psi_goal_pose)*x_gate;
			y_goal_pose= y_usv_NED + sin(psi_goal_pose)*y_gate;
		if (gate == "Entrance_Exit")
		{
		
			if((x_goal_pose > x_min_entr)&&(x_goal_pose < x_max_entr) && (y_goal_pose > y_min_entr) && (y_goal_pose < y_max_entr))
			{ 
				psi_goal_poses[0]= psi_usv_NED + alpha;
				x_goal_poses[0]= x_usv_NED + cos(psi_goal_poses[point])*x_gate;
				y_goal_poses[0]= y_usv_NED + sin(psi_goal_poses[point])*y_gate;
				ROS_DEBUG("--------------ENTRANCE GATE-------------------");
				ROS_DEBUG("------------x_gate = %f----------------------",x_gate);
				ROS_DEBUG("------------y_gate = %f----------------------",y_gate);
				ROS_DEBUG("------------alpha = %f-----------------------",alpha);
				ROS_DEBUG("------------x_GOAL = %f----------------------",x_goal_poses[0]);
				ROS_DEBUG("------------y_GOAL = %f----------------------",y_goal_poses[0]);
				ROS_DEBUG("------------psi_GOAL = %f--------------------",psi_goal_poses[0]);
			}	
			
		}
		else if (gate == "Speed Gate")
		{
			if((x_goal_pose > x_min_speed)&&(x_goal_pose < x_max_speed) && (y_goal_pose > y_min_speed) && (y_goal_pose < y_max_speed))
			{ 
				psi_goal_poses[2]= psi_usv_NED + alpha;
				x_goal_poses[2]= x_usv_NED + cos(psi_goal_poses[point])*x_gate;
				y_goal_poses[2]= y_usv_NED + sin(psi_goal_poses[point])*y_gate;
				ROS_DEBUG("--------------SPEED GATE----------------------");
				ROS_DEBUG("------------x_gate = %f----------------------",x_gate);
				ROS_DEBUG("------------y_gate = %f----------------------",y_gate);
				ROS_DEBUG("------------alpha = %f-----------------------",alpha);
				ROS_DEBUG("------------x_GOAL = %f----------------------",x_goal_poses[point]);
				ROS_DEBUG("------------y_GOAL = %f----------------------",y_goal_poses[point]);
				ROS_DEBUG("------------psi_GOAL = %f--------------------",psi_goal_poses[point]);
			}	
			
		}
		//calculations_done_3 = true;
	}
//	current_goal_pose_publish();
				//Middle of the channel
                                psi_goal_poses[1]= 3.14;
                                x_goal_poses[1]= 0;
                                y_goal_poses[1]= 0;

	goal_poses = 3;
} //end of Task3_Path

// THIS FUNCTION: Subscribes buoy topic
// ACCEPTS: (VOID) Uses global North and South points
// RETURNS: (VOID) Populates array of poses
// =============================================================================
void gates_pose(const nav_msgs::Path::ConstPtr& msg)
{

    
		//ROS_INFO("Subscribed");
		x_gate = msg->poses[1].pose.position.x;
		y_gate = msg->poses[1].pose.position.y;
	
		gate = msg->poses[1].header.frame_id;
		
		if ((x_gate == -69) && (y_gate == -69))
		{
			same_point = true;
			ROS_INFO("-----------------------------SAME POINT------------------");
		}
		else 
		{
			x_gate_prev = x_gate;
			y_gate_prev = y_gate;
			same_point = false;
		}
		

	if (gate == "No Gate Found")
		ROS_DEBUG("--------------NO GATE FOUND-------------------");
	//else if(gate == "Speed Gate")
		//ROS_INFO("Speed Gate");
} //end of Task3_Path

//------------------------------------------END CHANNEL NAVIGATION---------------------------------------

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
				x_goal_poses[i] = x_turt_g[i];
				y_goal_poses[i] = y_turt_g[i];
				psi_goal_poses[i] = psi_turt_g[i];
			}
			//ROS_INFO("x_t_NED: %4.2f    y_t_NED: %4.2f    x_p_NED: %4.2f    y_p_NED: %4.2f", x_t_NED, y_t_NED, x_p_NED, y_p_NED);

			for (int i=9; i<18; i++)
			{
				// go to the platypus second
				x_goal_poses[i] = x_plat_g[i-9];
				y_goal_poses[i] = y_plat_g[i-9];
				psi_goal_poses[i] = psi_plat_g[i-9];
			}
		}
		else  // if the distance between the USV and turtle is greater than the distance between the USV and the platypus
		{
			for (int i=0; i<9; i++)
			{
				// go to the platypus first
				x_goal_poses[i] = x_plat_g[i];
				y_goal_poses[i] = y_plat_g[i];
				psi_goal_poses[i] = psi_plat_g[i];
			}
			for (int i=9; i<18; i++)
			{
				// go to the turtle second
				x_goal_poses[i] = x_turt_g[i-9];
				y_goal_poses[i] = y_turt_g[i-9];
				psi_goal_poses[i] = psi_turt_g[i-9];
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

// THIS FUNCTION: Sets goal pose
// ACCEPTS: float values for x,y, and psi of goal pose to be set in local NED frame
// RETURNS: (VOID)
//=============================================================================================================
void set_goal_pose(float x_goal, float y_goal, float psi_goal)
{
	x_goal_pose = x_goal;
	y_goal_pose = y_goal;
	psi_goal_pose = psi_goal;
	// ensure psi_goal_pose stays within -PI and PI 
	while ((psi_goal_pose < -PI) || (psi_goal_pose > PI))
	{
		if (psi_goal_pose < -PI)
		{
			psi_goal_pose += 2.0*PI;
		}
		if (psi_goal_pose > PI)
		{
			psi_goal_pose -= 2.0*PI;
		}
	}
}  // END OF set_goal_pose()

// THIS FUNCTION: Sets next goal pose x meters in front of USV
// ACCEPTS: float values for distance to move forward
// RETURNS: (VOID)
//=============================================================================================================
void go_forward(float dist)
{
	x_goal_poses[point] = x_usv_NED + cos(psi_usv_NED)*dist;
	y_goal_poses[point] = y_usv_NED + sin(psi_usv_NED)*dist;
	psi_goal_poses[point] = psi_usv_NED;
	
	// x_goal_pose = x_usv_NED + cos(psi_usv_NED)*dist;
	// y_goal_pose = y_usv_NED + sin(psi_usv_NED)*dist;
	// psi_goal_pose = psi_usv_NED;
}  // END OF go_forward()



// THIS FUNCTION: Display USV and goal pose
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void display_usv_and_goal_pose()
{
	ROS_INFO("PATH_PLANNER:-----USV POSE-----");
	ROS_INFO("PATH_PLANNER:     x   :  %4.2f", x_usv_NED);  // x posn.
	ROS_INFO("PATH_PLANNER:     y   :  %4.2f", y_usv_NED);  // y posn.
	ROS_INFO("PATH_PLANNER:     psi :  %4.2f", psi_usv_NED);  // heading
	ROS_INFO("PATH_PLANNER:-----USV POSE-----\n");
	ROS_INFO("PATH_PLANNER:-----GOAL POSE-----");
	ROS_INFO("PATH_PLANNER:     x   :  %4.2f", x_goal_pose);  // x posn.
	ROS_INFO("PATH_PLANNER:     y   :  %4.2f", y_goal_pose);  // y posn.
	ROS_INFO("PATH_PLANNER:     psi :  %4.2f", psi_goal_pose);  // heading
	ROS_INFO("PATH_PLANNER:-----GOAL POSE-----\n");
}  // END OF display_usv_and_goal_pose()

// THIS FUNCTION: Display pose errors
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void display_pose_errors()
{
	ROS_INFO("PATH_PLANNER:-----POSE ERRORS-----");
	ROS_INFO("PATH_PLANNER:     e_x   :  %4.2f", e_x);  // x posn. error
	ROS_INFO("PATH_PLANNER:     e_y   :  %4.2f", e_y);  // y posn. error
	ROS_INFO("PATH_PLANNER:     e_xy  :  %4.2f", e_xy);  // magnitude of posn. error
	ROS_INFO("PATH_PLANNER:     e_psi :  %4.2f", e_psi);  // heading error
	ROS_INFO("PATH_PLANNER:-----POSE ERRORS-----\n");
}  // END OF display_pose_errors()

// START OF FUNCTIONS FOR VISION PATH PLANNER 
// update left and right buoy centroid location (x,y) in global frame
void NED_buoys_update (const jetson::NED_objects::ConstPtr& buoys)
{
	ROS_INFO("Number of bouys detected: %i", buoys->quantity);
	if ((goal_pose_publish_status.data) && (!buoy_positions_recieved))
	{
		if ((PP_state == 6) || (DnD_path_planner == 2) || (DnD_path_planner == 4) || (PP_state == 2) || (PP_state == 3))
		{
			for (int i = 0; i < buoys->quantity; i++)
			{
				ROS_INFO("Check%i", i);
				if ((buoys->objects[i].point.x < 15.0) && (buoys->objects[i].point.x > 5.0) && (buoys->objects[i].point.y < 10.0) && (buoys->objects[i].point.y > -10.0))
				{
					ROS_INFO("GOOD");
				}
				else
				{
					ROS_INFO("BAD");
				}
				if (strcmp(buoys->objects[i].header.frame_id.c_str(), red_buoy) == 0)
				{
					ROS_INFO("RED");
				}
				if (strcmp(buoys->objects[i].header.frame_id.c_str(), green_buoy) == 0)
				{
					ROS_INFO("GREEN");
				}
				if ((strcmp(buoys->objects[i].header.frame_id.c_str(), red_buoy) == 0) && (buoys->objects[i].point.x < 15.0) && (buoys->objects[i].point.x > 5.0) && (buoys->objects[i].point.y < 10.0) && (buoys->objects[i].point.y > -10.0))
				{
					CL_x = buoys->objects[i].point.x;
					CL_y = buoys->objects[i].point.y;
					left_buoy_found = true;
				}
				if ((strcmp(buoys->objects[i].header.frame_id.c_str(), green_buoy) == 0) && (buoys->objects[i].point.x < 15.0) && (buoys->objects[i].point.x > 5.0) && (buoys->objects[i].point.y < 10.0) && (buoys->objects[i].point.y > -10.0))
				{
					CR_x = buoys->objects[i].point.x;
					CR_y = buoys->objects[i].point.y;
					right_buoy_found = true;
				}
			}
			if (!left_buoy_found)
			{
				ROS_INFO("PATH_PLANNER:---LEFT BUOY NOT FOUND---");
			}
			if (!right_buoy_found)
			{
				ROS_INFO("PATH_PLANNER:---RIGHT BUOY NOT FOUND---");
			}
			if ((left_buoy_found) && (right_buoy_found))
			{
				buoy_positions_recieved = true;
			}
			// PERHAPS ADD CODE HERE TO SLIGHLTY ADJUST CURRENT GOAL HEADING TO ROTATE THE VEHICLE SIGHT BACK TOWARDS NEXT SET OF BUOYS
			if (!buoy_positions_recieved)
			{
				if ((left_buoy_found) && (!right_buoy_found))
				{
					ROS_INFO("PATH_PLANNER:---Adjusting goal heading to find buoy pair location---");
					psi_goal_pose += PI/6;  // WAS PI/18
				}
				else if ((right_buoy_found) && (!left_buoy_found))
				{
					ROS_INFO("PATH_PLANNER:---Adjusting goal heading to find buoy pair location---");
					psi_goal_pose -= PI/6;  // WAS PI/18
				}
				// ensure psi_goal_pose stays within -PI and PI 
				while ((psi_goal_pose < -PI) || (psi_goal_pose > PI))
				{
					if (psi_goal_pose < -PI)
					{
						psi_goal_pose += 2.0*PI;
					}
					if (psi_goal_pose > PI)
					{
						psi_goal_pose -= 2.0*PI;
					}
				}
			}
			// reset for next update
			left_buoy_found = false;
			right_buoy_found = false;
		}
	}
}  // END OF NED_buoys_update()

// THIS FUNCTION: Calculates the updated approach-, mid-, and exit- points to go through the current pair of buoys
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void calculate_buoy_waypoints()
{
	// hardcode values
	//CL_x = 37.85;
	//CL_y = 2.2;
	//CR_x = 37.85;
	//CR_y = 15.75;
	//x_usv_NED = 24.24;
	//y_usv_NED = 11.88;
	//psi_usv_NED = 0.0;
	ROS_INFO("________vvvvvvvvvvv  {PP}  vvvvvvvvvvv___________________\n");
	ROS_INFO("~~~USV pose wrt local NED frame~~~");
	ROS_INFO("psi: %f", psi_usv_NED);
	ROS_INFO("x: %f", x_usv_NED);
	ROS_INFO("y: %f\n", y_usv_NED);
	ROS_INFO("~~~Buoy locations wrt USV~~~");
	ROS_INFO("LB_x: %f", CL_x);
	ROS_INFO("LB_y: %f", CL_y);
	ROS_INFO("RB_x: %f", CR_x);
	ROS_INFO("RB_y: %f\n", CR_y);
	CL_x_NED = cos(psi_usv_NED)*CL_x - sin(psi_usv_NED)*CL_y + x_usv_NED;
	CL_y_NED = sin(psi_usv_NED)*CL_x + cos(psi_usv_NED)*CL_y + y_usv_NED;
	CR_x_NED = cos(psi_usv_NED)*CR_x - sin(psi_usv_NED)*CR_y + x_usv_NED;
	CR_y_NED = sin(psi_usv_NED)*CR_x + cos(psi_usv_NED)*CR_y + y_usv_NED;
	ROS_INFO("~~~Buoy locations wrt local NED frame~~~");
	ROS_INFO("LB_x: %f", CL_x_NED);
	ROS_INFO("LB_y: %f", CL_y_NED);
	ROS_INFO("RB_x: %f", CR_x_NED);
	ROS_INFO("RB_y: %f\n", CR_y_NED);

	M_x = (CL_x_NED+CR_x_NED)/2.0;  // x-location of midpoint
	M_y = (CL_y_NED+CR_y_NED)/2.0;  // y-location of midpoint

	d_L = sqrt(pow((CL_x_NED-x_usv_NED),2.0)+pow((CL_y_NED-y_usv_NED),2.0));  // distance from USV to left buoy
	d_M = sqrt(pow((M_x-x_usv_NED),2.0)+pow((M_y-y_usv_NED),2.0));  // distance from USV to midpoint
	d_R = sqrt(pow((CR_x_NED-x_usv_NED),2.0)+pow((CR_y_NED-y_usv_NED),2.0));  // distance from USV to right buoy
	d_I = (d_L+d_M+d_R)/3.0;  // distance from midpoint to approach point

	// intermediate approach point doesn't need to be more than 4 meters back from midpoint
	if (d_I > 5.0)
	{
	  d_I = 5.0;
	}

	d_LM = 0.5*sqrt(pow((CR_x_NED-CL_x_NED),2.0)+pow((CR_y_NED-CL_y_NED),2.0));  // half distance between left and right buoys
	a_L = sqrt(pow(d_LM,2.0)+pow(d_I,2.0));  // distance between left buoy and the approach point
	theta = atan(d_I/d_LM);  // angle created by d_I and d_LM

	x_I_CL = a_L*cos(theta);  // x-coord. of intermediate point wrt left buoy 
	y_I_CL = a_L*sin(theta);  // y-coord. of intermediate point wrt left buoy

	x_E_CL = a_L*cos(theta);  // x-coord. of exit point wrt left buoy 
	y_E_CL = -a_L*sin(theta);  // y-coord. of exit point wrt left buoy

	// calculate intermediate position wrt global
	s_M = (CR_y_NED-CL_y_NED)/(CR_x_NED-CL_x_NED);  // slope of of line from CL to CR 
	alpha = atan2((CR_y_NED-CL_y_NED),(CR_x_NED-CL_x_NED));  // angle of frame CL wrt global frame

	ROS_INFO("ALPHA: %4.2f    THETA: %4.2f\n", alpha, theta);

	I_x = cos(alpha)*x_I_CL - sin(alpha)*y_I_CL + CL_x_NED;  // x-coord. of intermediate point wrt global
	I_y = sin(alpha)*x_I_CL + cos(alpha)*y_I_CL + CL_y_NED;  // y-coord. of intermediate point wrt global

	E_x = cos(alpha)*x_E_CL - sin(alpha)*y_E_CL + CL_x_NED;  // x-coord. of exit point wrt global
	E_y = sin(alpha)*x_E_CL + cos(alpha)*y_E_CL + CL_y_NED;  // y-coord. of exit point wrt global

	x_goal_poses[0] = I_x;
	y_goal_poses[0] = I_y;
	psi_goal_poses[0] = alpha - PI/2;
	x_goal_poses[1] = M_x;
	y_goal_poses[1] = M_y;
	psi_goal_poses[1] = alpha - PI/2;
	x_goal_poses[2] = E_x;
	y_goal_poses[2] = E_y;
	psi_goal_poses[2] = alpha - PI/2;

	// display calculated goal points to reach
	ROS_INFO("~~~Desired points~~~");
	ROS_INFO("I_x: %f", x_goal_poses[0]);
	ROS_INFO("I_y: %f", y_goal_poses[0]);
	ROS_INFO("M_x: %f", x_goal_poses[1]);
	ROS_INFO("M_y: %f", y_goal_poses[1]);
	ROS_INFO("E_x: %f", x_goal_poses[2]);
	ROS_INFO("E_y: %f\n", y_goal_poses[2]);
	ROS_INFO("________^^^^^^^^^^^  {PP}  ^^^^^^^^^^^___________________|\n");

	point = 0;
	goal_poses_quantity = 3;
	calculations_done = true;
}  // END OF calculate_buoy_waypoints()
//............................................................End of Functions............................................................

int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "path_planner");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8,nh9;

	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, na_state_update);
	ros::Subscriber pp_state_sub = nh2.subscribe("pp_state", 1, pp_state_update);
	ros::Subscriber ps_state_sub = nh3.subscribe("ps_state", 1, ps_state_update);
	ros::Subscriber nav_NED_sub = nh4.subscribe("nav_ned", 1, pose_update);				// Obtains the USV pose in global NED from mission_control
	//ros::Subscriber obj_rec_sub = nh9.subscribe("obj_rec", 1, object_rec);						//Obtains what object the camera recognise, pose of the object and order of recognition
	ros::Subscriber gates_pose_sub = nh4.subscribe("/path_vision", 1, gates_pose);	
	ros::Subscriber PA_NED_buoys_sub = nh7.subscribe("PA_NED_buoys", 1, NED_buoys_update);  // current buoy IDs with respective locations for planner to use 

	// Publishers
	pp_initialization_state_pub = nh5.advertise<std_msgs::Bool>("pp_initialization_state", 1);	// publisher for state of initialization
	current_goal_pose_pub = nh6.advertise<jetson::usv_pose_msg>("current_goal_pose", 1);		// current goal for low level controller (propulsion_system)
	goal_pose_publish_state_pub = nh7.advertise<std_msgs::Bool>("goal_pose_publish_state", 1);	// "goal_pose_publish_state" publisher for whether NED converted waypoints have been published
	pp_USV_pose_update_state_pub = nh8.advertise<std_msgs::Bool>("pp_USV_pose_update_state", 1);	// publisher for whether usv NED pose has been updated or not

	// Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize simulation time
	ros::Time::init();

	// Initialize global variables
	goal_pose_publish_status.data = false;
	pp_USV_pose_update_status.data = false;
	pp_initialization_status.data = false;
	current_time = ros::Time::now();							// sets current time to the time it is now
	last_time = current_time;								// sets last time to the current_time

	//sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(5);		// was 50
	
	psi_goal_poses[2]= 3.14;
        x_goal_poses[2]= 0;
        y_goal_poses[2]= 0;
	psi_goal_poses[0]= 3.14;
        x_goal_poses[0]= 50;
        y_goal_poses[0]= 50;

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		PATH_PLANNER_inspector();							// check that entire system is initialized before starting calculations


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
				psi_goal_poses[0] = -0.21;			
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
						e_x = x_goal_poses[point] - x_usv_NED;                                       // calculate error in x position
						e_y = y_goal_poses[point] - y_usv_NED;                                       // calculate error in y position
						e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
						e_psi = psi_goal_poses[point] - psi_NED;

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
							// VARIABLES FOR THE BUOY NAVIGATION CALCULATIONS - calculate_buoy_waypoints()

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
					calculate_path();				// this function will generate the updated array of poses
					E_reached = false;
					goal_pose_publish_status.data = false;
				}
				break;

//-------------------------------------------------------------------CHANNEL NAVIGATION---------------------------------------------------------------
			case 3:          				               // Task 3: Follow the path
			//ROS_DEBUG("Task 3");
				
					//if (calculations_done_3)
					//{
						//if ((NA_state == 1) && (PS_state == 1))		// if the navigation_array is providing NED USV state, and the propulsion_system is ON
						//{	
							ROS_DEBUG("Task 3");
							Task3_Path();
							e_x_prev = e_x;
							e_y_prev = e_y;
							e_xy_prev = e_xy;
							e_psi_prev = e_psi;
							// determine error in x and y (position)
							e_x = x_goal_poses[point] - x_usv_NED;                                       // calculate error in x position
							e_y = y_goal_poses[point] - y_usv_NED;                                       // calculate error in y position
							e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
							e_psi = psi_goal_poses[0] - psi_NED;

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

							
								
								
								
							//float sign_change_check = e_x/e_x_prev;
							if ((e_xy < 1.0) && (!E_reached))	//FOR NOW e_xy_allowed = 3.0 
							{
								point += 1;
								ROS_DEBUG("Point %i of %i reached. FIRST GATE! --MC", point, goal_poses);
								if (point==goal_poses)
								{
								  E_reached = true;	
								  ROS_DEBUG("End point has been reached. BOTH GATES! --MC\n");
								
								}
							}
							
						current_goal_pose_publish();

					//}	
			break;
//-------------------------------------------------------------END OF CHANNEL NAVIGATION---------------------------------------------------------------------
		/*	case 4:
			{
				if (calculations_done_4)
				{
					if ((NA_state == 1) && (PS_state == 1))		// if the navigation_array is providing NED USV state, and the propulsion_system is ON
						{
						e_x_prev = e_x;
						e_y_prev = e_y;// THIS FUNCTION: Calculates pose errors
						e_xy_prev = e_xy;
						e_psi_prev = e_psi;
						// determine error in x and y (position)
						e_x = x_goal_poses[point] - x_usv_NED;                                       // calculate error in x position
						e_y = y_goal_poses[point] - y_usv_NED;                                       // calculate error in y position
						e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
						e_psi = psi_goal_poses[point] - psi_NED;

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
			
			case 6:  // Dynamic navigation demonstration - More Complex approach using vision
				ROS_INFO("PATH_PLANNER: DnD State %i\n", DnD_path_planner);
				//  1 = Start point search and approach
				//  2 = Search for next buoy pair
				//  3 = Going to calculated approach and mid points of entrance buoy pair
				//  4 = Start search and approach for exit buoy pair while continuing on current path
				//	5 = Exit point has been reached and task is complete
				switch(DnD_path_planner)
				{
					case 1:  // Start point search and approach
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.03;  // [rad] (about 2 degrees) heading error tolerance threshold
						if ((params_GOT) && (pp_USV_pose_update_status.data) && (!goal_pose_publish_status.data))  //&& (!propulsion_system_topic_published) // if the USV pose updated but the propulsion_system_topic not published  
						{
							set_goal_pose(x_DnD_start, y_DnD_start, psi_DnD_start);  // set the goal pose to the start pose of the dynamic navigation demonstration
							//propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							current_goal_pose_publish();
							calculate_pose_errors();
							display_pose_errors();
							display_usv_and_goal_pose();
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed))
							{
								DnD_path_planner = 2;  //// THIS FUNCTION: Calculates pose errors

								ROS_INFO("PATH_PLANNER:-----Start point reached-----\n");
							}
						}
						break;
					case 2:  // Search for entrance buoy pair
						if ((pp_USV_pose_update_status.data) && (!goal_pose_publish_status.data))  // && (!propulsion_system_topic_published) if the USV pose updated but the propulsion_system_topic not published
						{
							//propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							current_goal_pose_publish();
							if ((buoy_positions_recieved) && (!calculations_done))  // if buoy positions wrt the USV have been recieved but the path through the buoy pair has not been (re)calculated
							{
								calculate_buoy_waypoints();
							}
							if (calculations_done)
							{
								DnD_path_planner = 3;  // Going to calculated approach and mid points of entrance buoy pair
								ROS_INFO("PATH_PLANNER:-----Entrance gate approach calculated-----\n");
								// reset for next search
								buoy_positions_recieved = false;
								calculations_done = false;
							}
						}
						break;
					case 3:  // Going to calculated approach and mid points of entrance buoy pair
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.03;  // [rad] (about 2 degrees) heading error tolerance threshold
						if ((pp_USV_pose_update_status.data) && (!goal_pose_publish_status.data))  //&& (!propulsion_system_topic_published) if the USV pose updated but the propulsion_system_topic has not been published
						{
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!M_reached))  // if within pose tolerance and mid-point not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == (goal_poses_quantity-1))  // if the mid-point has been reached
								{
								  M_reached = true;  // true means the mid-point has been reached
								}
							}
							else if (M_reached)  // if mid point has been reached
							{
								go_forward(20);  // tell the USV to go 30 meters forward
								DnD_path_planner = 4;  // Start search and approach for exit buoy pair while continuing on current updated path
								ROS_INFO("PATH_PLANNER:-----Mid-point of entrance gates reached-----\n");
								// reset for next time doing task
								M_reached = false;  // false means the mid point has not been reached
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								//propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
								current_goal_pose_publish();
							}
							else if (e_xy <= max_next_position_distance)
							{
								//propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
								current_goal_pose_publish();
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							display_usv_and_goal_pose();
						}  // END OF if ((pp_USV_pose_update_status.data) && (!propulsion_system_topic_published))
						break;
					case 4:  // Start search and approach for exit buoy pair while continuing on current path
						e_xy_allowed = 1.5;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.785;  // [rad] (about 45 degrees) heading error tolerance threshold
						if ((pp_USV_pose_update_status.data) && (!goal_pose_publish_status.data)) // CHECK!! if the USV pose updated but the propulsion_system_topic has not been published
						{
							if ((buoy_positions_recieved) && (!calculations_done))  // if buoy positions wrt the USV have been recieved but the path through the buoy pair has not been (re)calculated
							{
								calculate_buoy_waypoints();
								// reset for next search
								buoy_positions_recieved = false;
								calculations_done = false;
								ROS_INFO("PATH_PLANNER:-----Path through exit gate recalculated-----");
							}
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!E_reached))  // if within pose tolerance and end point not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == goal_poses_quantity)  // if the exit point has been reached
								{
								  E_reached = true;  // true means the exit point has been reached
								}
							}
							else if (E_reached)  // if exit point has been reached
							{
								set_goal_pose(x_usv_NED, y_usv_NED, psi_usv_NED);  // have the USV station-keep until next task
								DnD_path_planner = 5;  // Exit point has been reached and task is complete
								ROS_INFO("PATH_PLANNER:-----End point of exit gates reached-----");
								// reset for next time doing task
								E_reached = false;  // false means the exit point has not been reached
								point -= 1;  // decrement point for feeding end point goal to station-keep until new task
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								//propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
								current_goal_pose_publish();
							}
							else if (e_xy <= max_next_position_distance)
							{
								//propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
								current_goal_pose_publish();
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							display_usv_and_goal_pose();
						}  // END OF if ((pp_USV_pose_update_status.data) && (!propulsion_system_topic_published))
						break;
					case 5:  // Exit point has been reached and task is complete
						if ((pp_USV_pose_update_status.data) && (goal_pose_publish_status.data))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							//propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							current_goal_pose_publish();
						}
						break;
				}*/
			//	break;
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
