//  Filename:				mission_control.cpp
//  Creation Date:			03/25/2022
//  Last Revision Date:			04/04/2022
//  Author(s) [email]:			Bradley Hacker [bhacker@lssu.edu]
//  Revisor(s) [email] {Revision Date}:	Bradley Hacker [bhacker@lssu.edu] {03/28/2022}
//  Organization/Institution:		Lake Superior State University - Team jetson
//
// ...............................About mission_control.cpp......................................
//  This code acts as the autonomous state machine of the WAM-V USV.
//  It will subscribe to the vrx/task/info to control the state of the system.
//  This code will subscribe to goal poses given from the gps_imu node.
//  Dependent on the current task state and system state, mission_control
//  will publish whether or not the low level controllers should be on, as well
//  as the goal of the low level controllers.
//
//  Inputs and Outputs of the mission_control.cpp file
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
#include "nav_msgs/Odometry.h"
#include "jetson/state_msg.h"					// message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"
#include "jetson/usv_pose_msg.h"				// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "jetson/NED_waypoints.h"				// message that holds array of converted WF goal waypoints w/ headings and number of waypoints
#include "jetson/task_info.h"					// task name and state
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    		// loop counter, first 10 loops used to intitialize subscribers
bool system_initialized = false;				// false means the system has not been initialized

//	STATES CONCERNED WITH "mission_control"
int MC_state = 0;	//	0 = On standby		//	1 = Station-Keeping			//	2 = Wayfinding

//	STATES CONCERNED WITH "navigation_array"
int NA_state = 0;	//	0 = On standby		//	1 = USV NED pose converter

//	STATES CONCERNED WITH "path_planner"
int PP_state = 0;	//	0 = On standby		//	1 = Task 1: Station-Keeping		//	2 = Task 2: Wayfinding

//	STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;	//	0 = On standby		//	1 = Propulsion system ON

int goal_poses;              				// total number of poses to reach
int loop_goal_published;         			// this is kept in order to ensure propulsion_system doesn't start until the goal is published by path_planner

float x_usv_NED, y_usv_NED, psi_NED; 			// vehicle position and heading (pose) in NED

bool NED_goal_pose_published = false;			// false means the NED goal pose has not yet been published to the propulsion_system by the path_planner
bool usv_NED_pose_updated = false;			// false means usv NED pose has not been acquired from navigation_array by the path_planner

// THE FOLLOWING FIVE BOOLS ARE USED TO DETERMINE IF MISSION_CONTROL AND SUBSYTEMS HAVE BEEN INITIALIZED
bool navigation_array_initialized = false;
bool path_planner_initialized = false;
bool propulsion_system_initialized = false;

// STATE MESSAGES AND PUBLISHERS
jetson::state_msg na_state_msg;						// "na_state" message
ros::Publisher na_state_pub;								// "na_state" publisher
jetson::state_msg pp_state_msg;						// "pp_state" message
ros::Publisher pp_state_pub;								// "pp_state" publisher
jetson::state_msg ps_state_msg;						// "ps_state" message
ros::Publisher ps_state_pub;								// "ps_state" publisher

ros::Time current_time, last_time;						// creates time variables
//..............................................................End of Global Variables..........................................................

//..................................................................Functions...........................................................................
// THIS FUNCTION UPDATES THE STATE OF mission_control
// ACCEPTS NOTHING (VOID)
// RETURNS NOTHING (VOID)
// =====================================================
void parameters_function()
{
	ros::param::get("/MC", MC_state);
} // END OF parameters_function()

// THIS FUNCTION: Updates and publishes all subsytem states, as well as mission_control state dependent on current system statuses
// ACCEPTS: Current task given through launch file
// RETURNS: (VOID)
// =============================================================================
void state_update()						// NOTE: To simplify, use just message variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
{
	// FIRST RESET STATES TO BE SET ACCORDINGLY TO CURRENT SYSTEM STATUSES
	//	STATES CONCERNED WITH "navigation_array"
	NA_state = 0;		//	0 = On standby	//	1 = USV NED pose converter

	//	STATES CONCERNED WITH "path_planner"
	PP_state = 0;		//	0 = On standby	//	1 = Task 1: Station-Keeping	//	2 = Task 2: Wayfinding

	//	STATES CONCERNED WITH "propulsion_system"
	PS_state = 0;		//	0 = On standby	//	1 = Propulsion system ON

	if (system_initialized)											// Do not begin subsytem activity until system is initialized
	{
		if (MC_state == 0)
		{
			// ALL CODES ON STANDBY
			NA_state = 0;
			PP_state = 0;
			PS_state = 0;
			NED_goal_pose_published = false;
		}
		else if (MC_state == 1)
		{
			// Station Keeping
			NA_state = 1;
			PP_state = 1;
			if ((NED_goal_pose_published) && (loop_count > loop_goal_published))	// if the goal pose has been published to propulsion_system and time has been given for goal and usv states to be attained by subsytems
			{
				PS_state = 1;									// Propulsion system ON
			}
			else
			{
				PS_state = 0;									// Propulsion system on standby
			}
		}
		else if (MC_state == 2)
		{
			// Task 2 (Entrance and exit gates)
			NA_state = 1;
			PP_state = 2;
			if ((NED_goal_pose_published) && (loop_count > loop_goal_published))	// if the goal pose has been published to propulsion_system and time has been given for goal and usv states to be attained by subsytems
			{
				PS_state = 1;										// Propulsion system ON
			}
			else
			{
				PS_state = 0;										// Propulsion system on standby
			}
		}
		else if (MC_state == 3)
		{
			// Task 3(Follow the path)
			NA_state = 1;
			PP_state = 3;
			if ((NED_goal_pose_published) && (loop_count > loop_goal_published))	// if the goal pose has been published to propulsion_system and time has been given for goal and usv states to be attained by subsytems
			{
				PS_state = 1;									// Propulsion system ON
			}
			else
			{
				PS_state = 0;									// Propulsion system on standby
			}
		}
		else if (MC_state == 4)
		{
			// Task 4(Wildlife finding)
			NA_state = 1;
			PP_state = 1;
			if ((NED_goal_pose_published) && (loop_count > loop_goal_published))	// if the goal pose has been published to propulsion_system and time has been given for goal and usv states to be attained by subsytems
			{
				PS_state = 1;									// Propulsion system ON
			}
			else
			{
				PS_state = 0;									// Propulsion system on standby
			}
		}
		else if (MC_state == 5)
		{
			// Task 5(Station keep)
			NA_state = 1;
			PP_state = 1;
			if ((NED_goal_pose_published) && (loop_count > loop_goal_published))	// if the goal pose has been published to propulsion_system and time has been given for goal and usv states to be attained by subsytems
			{
				PS_state = 1;									// Propulsion system ON
			}
			else
			{
				PS_state = 0;									// Propulsion system on standby
			}
		}
		else if (MC_state == 6)
		{
			// Task 6 (Docking)
			NA_state = 1;
			PP_state = 1;
			if ((NED_goal_pose_published) && (loop_count > loop_goal_published))	// if the goal pose has been published to propulsion_system and time has been given for goal and usv states to be attained by subsytems
			{
				PS_state = 1;									// Propulsion system ON
			}
			else
			{
				PS_state = 0;									// Propulsion system on standby
			}
		}
		else if (MC_state == 7)
		{
			// Task 7 (For now Station Keep)
			NA_state = 1;
			PP_state = 1;
			if ((NED_goal_pose_published) && (loop_count > loop_goal_published))	// if the goal pose has been published to propulsion_system and time has been given for goal and usv states to be attained by subsytems
			{
				PS_state = 1;									// Propulsion system ON
			}
			else
			{
				PS_state = 0;									// Propulsion system on standby
			}
		}
		
		else
		{
			// ALL CODES ON STANDBY
			NA_state = 0;
			PP_state = 0;
			PS_state = 0;
		}

		// PUBLISH EACH SUBSYTEMS STATE
		// SEND STATE TO NAVIGATION_ARRAY
		na_state_msg.header.seq +=1;											// sequence number
		na_state_msg.header.stamp = current_time;					// set stamp to current time
		na_state_msg.header.frame_id = "mission_control";		// header frame
		na_state_msg.state.data = NA_state;								// set navigation_array_state
		na_state_pub.publish(na_state_msg);								// publish na_state_msg to "na_state"

		// SEND STATE TO PATH_PLANNER
		pp_state_msg.header.seq +=1;											// sequence number
		pp_state_msg.header.stamp = current_time;					// set stamp to current time
		pp_state_msg.header.frame_id = "mission_control";		// header frame
		pp_state_msg.state.data = PP_state;								// set path_planner_state
		pp_state_pub.publish(pp_state_msg);								// publish pp_state_msg to "pp_state"

		// SEND STATE TO PROPULSION_SYSTEM
		ps_state_msg.header.seq +=1;											// sequence number
		ps_state_msg.header.stamp = current_time;					// set stamp to current time
		ps_state_msg.header.frame_id = "mission_control";		// header frame
		ps_state_msg.state.data = PS_state;								// set propulsion_system_state
		ps_state_pub.publish(ps_state_msg);								// publish ps_state_msg to "ps_state"

/*		// UPDATE USER OF EACH CODES STATE
		ROS_DEBUG("\n----------------- CURRENT STATES --------------------");
		ROS_DEBUG("MC_state: %6i ~ 0 = On standby, 1 = USV NED pose converter, 2 = Task 2: Wayfinding ~ --MC", MC_state);
		ROS_DEBUG("NA_state: %i --MC", NA_state);
		ROS_DEBUG("PP_state: %i --MC", PP_state);
		ROS_DEBUG("PS_state: %i --MC\n", PS_state);
*/	} // if (system_initialized)
} // END OF state_update()

// SYSTEM INITIALIZATION CHECK FUNCTIONS /////////////////////////////////////////////////////////////////////////////////
// THIS FUNCTION: Checks initialization status of entire system using global variable initialization statuses
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void MISSION_CONTROL_inspector()
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;									// increment loop counter
	parameters_function();
	state_update();
	//ROS_INFO("Loop count = %i", loop_count);
	if ((loop_count > 5) && (navigation_array_initialized) && (path_planner_initialized) && (propulsion_system_initialized))
	{
		system_initialized = true;
		//ROS_INFO("mission_control_initialized");
	}
	else
	{
		system_initialized = false;
		ROS_INFO("!mission_control_initialized --MC");
	}
} // END OF MISSION_CONTROL_inspector()

// THIS FUNCTION: Subscribes to the navigation_array to check initialization status
// ACCEPTS: Initialization status from "na_initialization_state"
// RETURNS: (VOID)
// =============================================================================
void NAVIGATION_ARRAY_inspector(const std_msgs::Bool status)
{
	navigation_array_initialized = status.data;
} // END OF NAVIGATION_ARRAY_inspector()

// THIS FUNCTION: Subscribes to the path_planner to check initialization status
// ACCEPTS: Initialization status from "pp_initialization_state"
// RETURNS: (VOID)
// =============================================================================
void PATH_PLANNER_inspector(const std_msgs::Bool status)
{
	path_planner_initialized = status.data;
} // END OF PROPULSION_SYSTEM_inspector()

// THIS FUNCTION: Subscribes to the propulsion_system to check initialization status
// ACCEPTS: Initialization status from "ps_initialization_state"
// RETURNS: (VOID)
// =============================================================================
void PROPULSION_SYSTEM_inspector(const std_msgs::Bool status)
{
	propulsion_system_initialized = status.data;
} // END OF PROPULSION_SYSTEM_inspector()
// END OF SYSTEM INITIALIZATION CHECK FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////////////////////////

// THIS FUNCTION: Checks to see if the path_planner has got the updated usv pose
// ACCEPTS: Status from "pp_USV_pose_update_state"
// RETURNS: (VOID)
// =============================================================================
void pp_USV_pose_update_state_update(const std_msgs::Bool status)
{
	usv_NED_pose_updated = status.data;
} // END OF pp_USV_pose_update_state_update()

// THIS FUNCTION: Updates when NED goal pose has been published to propulsion_system by path_planner
// ACCEPTS: Goal pose publish status from "goal_waypoints_publish_state"
// RETURNS: (VOID)
// =============================================================================
void NED_goal_pose_published_update(const std_msgs::Bool status)
{
	if (status.data)
	{
		if (!NED_goal_pose_published)			// if the goal pose has been published and hasn't been updated
		{
			NED_goal_pose_published = true;
			loop_goal_published = loop_count;
		}
	}
	else
	{
		NED_goal_pose_published = false;
	}
} // END OF NED_goal_pose_published_update()

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

		/* printf("the x is: %f\n", odom->pose.pose.position.x); //extracts x coor from nav_odometery
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
		printf("the angular velocity z is: %f\n", odom-> twist.twist.angular.z);//prints velocity x */
	} // if navigation_array is in standard USV Pose Conversion mode
} // END OF pose_update()
//............................................................End of Functions............................................................

int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "mission_control");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9;

	// Subscribers
	ros::Subscriber na_initialization_state_sub = nh1.subscribe("na_initialization_state", 1, NAVIGATION_ARRAY_inspector);		// initialization status of navigation_array
	ros::Subscriber pp_initialization_state_sub = nh2.subscribe("pp_initialization_state", 1, PATH_PLANNER_inspector);		// initialization status of path_planner
	ros::Subscriber ps_initialization_state_sub = nh3.subscribe("ps_initialization_state", 1, PROPULSION_SYSTEM_inspector);		// initialization status of propulsion_system
	//ros::Subscriber nav_NED_sub = nh4.subscribe("nav_ned", 1, pose_update);							// Obtains the USV pose in global NED from mission_control
	ros::Subscriber goal_pose_publish_state_sub = nh5.subscribe("goal_pose_publish_state", 1, NED_goal_pose_published_update);	// whether or not goal pose has been published to propulsion_system yet 
	ros::Subscriber pp_USV_pose_update_state_sub = nh6.subscribe("pp_USV_pose_update_state", 1, pp_USV_pose_update_state_update);	// topic containing the status of whether or not the USV pose has been updated in path_planner

	// Publishers
	na_state_pub = nh7.advertise<jetson::state_msg>("na_state", 1);			// current navigation_array state
	pp_state_pub = nh8.advertise<jetson::state_msg>("pp_state", 1);			// current path_planner state
	ps_state_pub = nh9.advertise<jetson::state_msg>("ps_state", 1);			// current propulsion_system state

	// Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize simulation time
	ros::Time::init();

	// Initialize global variables
	current_time = ros::Time::now();						// sets current time to the time it is now
	last_time = current_time;							// sets last time to the time it is now

	//sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(100);

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		MISSION_CONTROL_inspector();		// check that entire system is initialized before starting calculations
		ros::spinOnce();							// update subscribers
		loop_rate.sleep();							// sleep for set loop_rate
		last_time = current_time;						// update last_time
	} // while(ros::ok())

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}
