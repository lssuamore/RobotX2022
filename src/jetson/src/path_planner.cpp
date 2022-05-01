//  Filename:											path_planner.cpp
//  Creation Date:									04/07/2022
//  Last Revision Date:							04/07/2022
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

int point = 0;                     		    		// number of points on trajectory reached
int goal_poses;              					// total number of poses to reach

float x_goal[100], y_goal[100], psi_goal[100];			// arrays to hold the NED goal poses
float x_usv_NED, y_usv_NED, psi_NED; 				// vehicle position and heading (pose) in NED
float e_x, e_y, e_xy, e_psi;					// current errors between goal pose and usv pose

bool E_reached = false;        					// false means the last point has not been reached

float e_xy_allowed = 1.0;       				// positional error tolerance threshold; NOTE: make as small as possible
float e_psi_allowed = 0.4;      				// heading error tolerance threshold; NOTE: make as small as possible

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
	ros::param::get("/x_goal_G", x_goal[0]);
	ros::param::get("/y_goal_G", y_goal[0]);
	ros::param::get("/psi_goal_G", psi_goal[0]);
	point = 0;				// take this out with wayfinding addition
} // END OF parameters_function()

// THIS FUNCTION: Updates the current NED USV goal pose
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void goal_pose_update()
{
	if (PP_state == 1) // if path_planner is in station keeping mode
	{
		// Update NED USV goal pose to station keep at from launch file
		parameters_function();
		goal_poses = 1;
	}
} // END OF goal_pose_update()

// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "pp_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void PATH_PLANNER_inspector()
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;				// increment loop counter
	goal_pose_update();
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
//............................................................End of Functions............................................................

int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "path_planner");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8;

	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, na_state_update);
	ros::Subscriber pp_state_sub = nh2.subscribe("pp_state", 1, pp_state_update);
	ros::Subscriber ps_state_sub = nh3.subscribe("ps_state", 1, ps_state_update);
	ros::Subscriber nav_NED_sub = nh4.subscribe("nav_ned", 1, pose_update);				// Obtains the USV pose in global NED from mission_control

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
	ros::Rate loop_rate(50);

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
				e_xy_allowed = 0.4;    			// positional error tolerance threshold; NOTE: make as small as possible
				e_psi_allowed = 0.4;   			// heading error tolerance threshold; NOTE: make as small as possible
				break;
			case 1:						// Station-Keeping
				current_goal_pose_publish();
				break;
			case 2:						// Wayfinding
				if ((NA_state == 1) && (PP_state == 2) && (PS_state == 1))	// if the navigation_array is providing NED USV state, Wayfinding planner is active, and the propulsion_system is ON
				{
					// determine error in x and y (position)
					e_x = x_goal[point] - x_usv_NED;		// calculate error in x position
					e_y = y_goal[point] - y_usv_NED;		// calculate error in y position
					e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));		// calculate magnitude of positional error
					e_psi = psi_goal[point] - psi_NED;

					if ((e_xy < e_xy_allowed) && (e_psi < e_psi_allowed) && (!E_reached))
					{
						point += 1;
						ROS_INFO("Point %i of %i reached. --MC", point, goal_poses);
						if (point==goal_poses)
						{
						  E_reached = true;
						  ROS_INFO("End point has been reached. --MC\n");
						}
					}

					if (E_reached)		// reset and go to points again once last point has been reached with a smaller tolerance threshold
					{
						point = 0;
						e_xy_allowed /= 2;
						e_psi_allowed = e_psi_allowed - 0.1;
						E_reached = false;
					}
				} // if (PP_state == 2)
				current_goal_pose_publish();
				break;
			default:
				break;
		}	// switch(PP_state)

		ros::spinOnce();			// update subscribers
		loop_rate.sleep();			// sleep for set loop_rate
		last_time = current_time;		// update last_time
	} // while(ros::ok())

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}
