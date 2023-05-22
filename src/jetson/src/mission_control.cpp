//  Filename:  mission_control.cpp
//  Creation Date:  3/25/2022
//  Last Revision Date:  9/21/2022
//  Author [email]:  Brad Hacker [bhacker@lssu.edu]
//  Revisor(s) [Revision Date]:
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
// 
//  Notes from author: PLEASE MAKE ANY PRINTOUTS START WITH "MISSION_CONTROL: " TO KNOW THAT IT IS FROM MISSION_CONTROL
//
// ...............................About mission_control.cpp......................................
//  This code acts as the autonomous state machine of the WAM-V USV.
//  It will subscribe to the "vrx/task/info" to control the state of the system.
//  This code will subscribe to goal poses given from the navigation_array.
//  Dependent on the current task state and system state, mission_control
//  will publish whether or not the low level controllers should be on.
//
//		Inputs [subscribers]: "waypoints_NED" (converted goal pose array), "/vrx/task/info", "initialization_states"
//		Outputs [publishers]: states of all other executables

//...............................................................................................Included Libraries and Message Types.........................................................................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "jetson/state.h"  // message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"  // message type used for communicating initialization status to mission_control
#include "std_msgs/Int64.h"  // message type used for acoustic status feedback
//......................................................................................End of Included Libraries and Message Types.....................................................................................

//.......................................................................................................................Constants.....................................................................................................................
#define PI 3.14159265
//................................................................................................................End of Constants...............................................................................................................

//.................................................................................................................Global Variables..............................................................................................................
int loop_count = 0;  // loop counter
// initialization bools
bool system_initialized = false;  // false means the system has not been initialized
// THE FOLLOWING SIX BOOLS ARE USED TO DETERMINE IF SUBSYTEMS HAVE BEEN INITIALIZED
bool navigation_array_initialized = false;
bool coordinate_converter_initialized = false;
bool path_planner_initialized = false;
bool propulsion_system_initialized = false;
bool perception_array_initialized = false;

bool PP_params_GOTTEN = false;  // false means that the params for current task haven't done been GOTTED by path_planner
bool vrx_mode;  // false means  vrx mode is not true, and that mission_control should get the mission from /MC parameter in launch file
bool CC_goal_poses_published = false;  // false means the NED poses have not yet been calculated and published by navigation_array

float start_sec, current_sec;
ros::Duration duration;  // used for calculations of time durations
ros::Time start_time, current_time, last_time;  // creates time variables

//	STATES CONCERNED WITH "mission_control"
//	0 = On standby
//  1 = Dynamic navigation demonstration
//  2 = Entrance and Exit gates
//  3 = Follow the path
//  4 = Wildlife Encounter - React and Report
//  5 = Scan the code
//  6 = Detect and dock
//  7 = Find and Fling
//  8 = UAV replenishment
int MC_state;

//=====STATE MESSAGES AND PUBLISHERS=====
//	STATES CONCERNED WITH "navigation_array"
//	0 = On standby
//	1 = USV NED state converter
jetson::state MC_na_state_msg;  // "MC_na_state" message
ros::Publisher MC_na_state_pub;  // "MC_na_state" publisher

//	STATES CONCERNED WITH "coordinate_converter"
//	0 = On standby
//	1 = VRX1: Station-Keeping NED goal pose converter
//	2 = VRX2: Wayfinding NED goal pose converter
//	4 = VRX4: Wildlife NED animals converter
jetson::state MC_cc_state_msg;  // "MC_cc_state" message
ros::Publisher MC_cc_state_pub;  // "MC_cc_state" publisher

//	STATES CONCERNED WITH "path_planner"
//	0 = On standby
//  1 = Dynamic navigation demonstration
//  2 = Entrance and Exit gates
//  3 = Follow the path
//  4 = Wildlife Encounter - React and Report
//  5 = Scan the code
//  6 = Detect and dock
//  7 = Find and Fling
//  8 = UAV replenishment
//	11 = VRX1: Station-Keeping
//	12 = VRX2: Wayfinding
//	14 = VRX4: Wildlife Encounter and Avoid
//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	16 = VRX6: Scan and Dock and Deliver
jetson::state MC_pp_state_msg;  // "MC_pp_state" message
ros::Publisher MC_pp_state_pub;  // "MC_pp_state" publisher

//	STATES CONCERNED WITH "propulsion_system"
//	0 = On standby
//	1 = Propulsion system ON
jetson::state MC_ps_state_msg;  // "MC_ps_state" message
ros::Publisher MC_ps_state_pub;  // "MC_ps_state" publisher

//	STATES CONCERNED WITH "perception_array"
//	0 = On standby
//	1 = General State
//	13 = VRX3: Landmark Localization and Characterization
//	14 = VRX4: Wildlife Encounter and Avoid
//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	16 = VRX6: Scan and Dock and Deliver
jetson::state MC_pa_state_msg;  // "MC_pa_state" message
ros::Publisher MC_pa_state_pub;  // "MC_pa_state" publisher
//..........................................................................................................End of Global Variables........................................................................................................

//.....................................................................................................................Functions.......................................................................................................................
// THIS FUNCTION: Updates the state of mission_control when not in vrx_mode
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =====================================================
void parameters_function()
{
	ros::param::get("/MC", MC_state);
} // END OF parameters_function()

// THIS FUNCTION: Publishes all subsystem states
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void publish_states()
{
	// PUBLISH EACH SUBSYTEMS STATE
	// SEND STATE TO NAVIGATION_ARRAY
	MC_na_state_msg.header.seq +=1;  // sequence number
	MC_na_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_na_state_msg.header.frame_id = "mission_control";  // header frame
	MC_na_state_msg.sim_mode.data = vrx_mode;
	MC_na_state_pub.publish(MC_na_state_msg);  // publish MC_na_state_msg to "MC_na_state"

	// SEND STATE TO COORDINATE_CONVERTER
	MC_cc_state_msg.header.seq +=1;  // sequence number
	MC_cc_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_cc_state_msg.header.frame_id = "mission_control";  // header frame
	MC_cc_state_msg.sim_mode.data = vrx_mode;
	MC_cc_state_pub.publish(MC_cc_state_msg);  // publish MC_cc_state_msg to "MC_cc_state"

	// SEND STATE TO PATH_PLANNER
	MC_pp_state_msg.header.seq +=1;  // sequence number
	MC_pp_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_pp_state_msg.header.frame_id = "mission_control";  // header frame
	MC_pp_state_msg.sim_mode.data = vrx_mode;
	MC_pp_state_pub.publish(MC_pp_state_msg);  // publish MC_pp_state_msg to "MC_pp_state"

	// SEND STATE TO PROPULSION_SYSTEM
	MC_ps_state_msg.header.seq +=1;  // sequence number
	MC_ps_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_ps_state_msg.header.frame_id = "mission_control";  // header frame
	MC_ps_state_msg.sim_mode.data = vrx_mode;
	MC_ps_state_pub.publish(MC_ps_state_msg);  // publish MC_ps_state_msg to "MC_ps_state"

	// SEND STATE TO PERCEPTION_ARRAY
	MC_pa_state_msg.header.seq +=1;  // sequence number
	MC_pa_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_pa_state_msg.header.frame_id = "mission_control";  // header frame
	MC_pa_state_msg.sim_mode.data = vrx_mode;
	MC_pa_state_pub.publish(MC_pa_state_msg);  // publish MC_pa_state_msg to "MC_pa_state"

	// UPDATE USER OF EACH CODES STATE
	ROS_INFO("MISSION_CONTROL:-----CURRENT STATES-----");
	if (!vrx_mode)  // if in user control mode
	{
		ROS_INFO("MISSION_CONTROL:      MC_state: %i", MC_state);
	}
	ROS_INFO("MISSION_CONTROL:      NA_state: %i", MC_na_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      CC_state: %i", MC_cc_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      PP_state: %i", MC_pp_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      PS_state: %i", MC_ps_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      PA_state: %i", MC_pa_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:-----CURRENT STATES-----\n");
} // END OF publish_states()

// THIS FUNCTION: Updates all subsystem states if in real-world mode
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void user_state_update()
{
	// FIRST: update the user desired mission
	parameters_function();
	
	// NEXT: reset states to be set accordingly to current system statuses
	//	STATES CONCERNED WITH "navigation_array"
	//	0 = On standby
	//	1 = USV NED state converter
	MC_na_state_msg.state.data = 1;

	//	STATES CONCERNED WITH "coordinate_converter"
	//	0 = On standby
	MC_cc_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "path_planner"
	//	0 = On standby
	//  1 = Dynamic navigation demonstration
	//  2 = Entrance and Exit gates
	//  3 = Follow the path
	//  4 = Wildlife Encounter - React and Report
	//  5 = Scan the code
	//  6 = Detect and dock
	//  7 = Find and Fling
	//  8 = UAV replenishment
	MC_pp_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "propulsion_system"
	//	0 = On standby
	//	1 = Propulsion system ON
	MC_ps_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "perception_array"
	//	0 = On standby
	//	1 = General State
	MC_pa_state_msg.state.data = 0;

	if (system_initialized)  // if entire system is initialized
	{
		if (MC_state == 1)  // Dynamic navigation demonstration
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 1;  // 1 = Dynamic navigation demonstration path planner
			if (PP_params_GOTTEN)
			{
				MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			}
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 2)  // Entrance and Exit gates
		{
			if (PP_params_GOTTEN)
			{
				MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			}
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 2;  // 2 = Entrance and Exit gates path planner
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 3)  // Follow the path
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 3;  // 3 = Follow the path path planner
			if (PP_params_GOTTEN)
			{
				MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			}
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 4)  // Wildlife Encounter - React and Report
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 4;  // 4 = Wildlife Encounter - React and Report path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 5)  // Scan the code
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 5;  // 5 = Scan the code path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 6)  // Detect and dock
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 6;  // 6 = Detect and dock path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 7)  // Find and Fling
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 7;  // 7 = Find and Fling path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 8)  // UAV replenishment
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 8;  // 8 = UAV replenishment path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 11)  // Station-keeping
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 11;  // 11 = Station-keeping planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			//MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 12)  // Figure eight wayfinding
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 12;  // 12 = Figure eight wayfinding path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			//MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 0)
		{
			// ALL CODES ON STANDBY
			ROS_INFO("MISSION_CONTROL: ON STANDBY.\n");
		}
		else
		{
			ROS_ERROR("MISSION_CONTROL: CAN'T RETRIEVE MC STATE FROM LAUNCH FILE.\n");
		}

		publish_states();  // PUBLISH EACH SUBSYTEMS STATE
	}  // END OF if (system_initialized)
} // END OF user_state_update()

// THIS FUNCTION: Checks initialization status of entire system and runs function to update states
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
//=============================================================================================================
void MISSION_CONTROL_inspector()
{
	current_time = ros::Time::now();  // sets current_time to the time it is now
	loop_count += 1;  // increment loop counter
	if ((navigation_array_initialized) && (coordinate_converter_initialized) && (path_planner_initialized) && (propulsion_system_initialized) && (perception_array_initialized))
	{
		system_initialized = true;
	}
	else
	{
		system_initialized = false;
		ROS_INFO("MISSION_CONTROL: System is not initialized yet.\n");
	}
	if (!vrx_mode)
	{
		user_state_update();
	}
	// UPDATE USER OF INITIALIZATION STATUSES
	//ROS_DEBUG("MISSION_CONTROL: Loop count = %i", loop_count);
	// if (system_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: mission_control_initialized");
	// }
	// if (navigation_array_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: navigation_array_initialized");
	// }
	// if (coordinate_converter_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: coordinate_converter_initialized");
	// }
	// if (path_planner_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: path_planner_initialized");
	// }
	// if (propulsion_system_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: propulsion_system_initialized");
	// }
	// if (perception_array_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: perception_array_initialized");
	// }
}  // END OF MISSION_CONTROL_inspector()

// THIS FUNCTION: Subscribes to the navigation_array to check initialization status
// ACCEPTS: std_msgs::Bool from "NA_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void NA_initialization_state_update(const std_msgs::Bool status)
{
	navigation_array_initialized = status.data;
}  // END OF NA_initialization_state_update()

// THIS FUNCTION: Subscribes to the coordinate_converter to check initialization status
// ACCEPTS: std_msgs::Bool from "CC_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void CC_initialization_state_update(const std_msgs::Bool status)
{
	coordinate_converter_initialized = status.data;
}  // END OF CC_initialization_state_update()

// THIS FUNCTION: Subscribes to the path_planner to check initialization status
// ACCEPTS: std_msgs::Bool from "PP_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void PP_initialization_state_update(const std_msgs::Bool status)
{
	path_planner_initialized = status.data;
}  // END OF PP_initialization_state_update()

// THIS FUNCTION: Subscribes to the propulsion_system to check initialization status
// ACCEPTS: std_msgs::Bool from "PS_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void PS_initialization_state_update(const std_msgs::Bool status)
{
	propulsion_system_initialized = status.data;
}  // END OF PS_initialization_state_update()

// THIS FUNCTION: Subscribes to the perception_array to check initialization status
// ACCEPTS: std_msgs::Bool from "PA_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void PA_initialization_state_update(const std_msgs::Bool status)
{
	perception_array_initialized = status.data;
}  // END OF PA_initialization_state_update()

// THIS FUNCTION: Updates when VRX Task goal poses have been converted and published
// ACCEPTS: std_msgs::Bool from "CC_goal_poses_publish_state"
// RETURNS: (VOID)
//=============================================================================================================
void CC_goal_poses_publish_state_update(const std_msgs::Bool status)
{
	CC_goal_poses_published = status.data;
	// UPDATE USER OF COORDINATE_CONVERTER STATUS
	// if (CC_goal_poses_published)
	// {
		// ROS_DEBUG("MISSION_CONTROL: COORDINATE CONVERTER FINISHED");
	// }
	// else
	// {
		// ROS_DEBUG("MISSION_CONTROL: COORDINATE CONVERTER NOT FINISHED");
	// }
}  // END OF CC_goal_poses_publish_state_update()

// THIS FUNCTION: Updates when the parameters have been GOT by path_planner
// ACCEPTS: std_msgs::Bool from "PP_params_GOT"
// RETURNS: (VOID)
//=============================================================================================================
void PP_params_GOT_update(const std_msgs::Bool status)
{
	PP_params_GOTTEN = status.data;
}  // END OF PP_params_GOT_update()
//.............................................................................................................END OF Functions...............................................................................................................

//..................................................................................................................Main Program..................................................................................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "mission_control");

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh11;

	// Subscribers
	ros::Subscriber NA_initialization_state_sub = nh1.subscribe("NA_initialization_state", 1, NA_initialization_state_update);  // initialization status of navigation_array
	ros::Subscriber CC_initialization_state_sub = nh2.subscribe("CC_initialization_state", 1, CC_initialization_state_update);  // initialization status of coordinate_converter
	ros::Subscriber PP_initialization_state_sub = nh3.subscribe("PP_initialization_state", 1, PP_initialization_state_update);  // initialization status of path_planner
	ros::Subscriber PS_initialization_state_sub = nh4.subscribe("PS_initialization_state", 1, PS_initialization_state_update);  // initialization status of propulsion_system
	ros::Subscriber PA_initialization_state_sub = nh5.subscribe("PA_initialization_state", 1, PA_initialization_state_update);  // initialization status of perception_array
	// set up vrx subscribers
	ros::Subscriber CC_goal_poses_publish_state_sub = nh8.subscribe("CC_goal_poses_publish_state", 1, CC_goal_poses_publish_state_update);  // whether or not goal waypoints have been converted and published yet
	ros::Subscriber PP_params_GOT_sub = nh10.subscribe("PP_params_GOT", 1, PP_params_GOT_update);  // params_GOT status of path_planner

	// Publishers
	// these publishers publish all the state topics off all the other programs so that mission_control can tell them all how to operate
	MC_na_state_pub = nh11.advertise<jetson::state>("MC_na_state", 1);  // current navigation_array state
	MC_cc_state_pub = nh11.advertise<jetson::state>("MC_cc_state", 1);  // current coordinate_converter state
	MC_pp_state_pub = nh11.advertise<jetson::state>("MC_pp_state", 1);  // current path_planner state
	MC_ps_state_pub = nh11.advertise<jetson::state>("MC_ps_state", 1);  // current propulsion_system state
	MC_pa_state_pub = nh11.advertise<jetson::state>("MC_pa_state", 1);  // current perception_array state

	// initialize header sequences
	MC_na_state_msg.header.seq = 0;
	MC_cc_state_msg.header.seq = 0;
	MC_pp_state_msg.header.seq = 0;
	MC_ps_state_msg.header.seq = 0;
	MC_pa_state_msg.header.seq = 0;

	// Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	ros::Time::init();

	// Initialize global variables
	//sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(10);

	// initial inspection to decide if system is in simulation or real-world mode
	vrx_mode = false;  // false means that the simulation mode is not true, and that the system launch should be set for real-world operation
	while (ros::Time::now().toSec() == 0.0)
	{
		ros::spin();  // spin while the time is not yet non-zero
		ROS_INFO("MISSION_CONTROL:---SPINNING UNTIL TIME IS NON_ZERO---\n");
	}
	ros::Duration(0.5).sleep();  // sleep for half a second
	start_time = ros::Time::now();  // sets start time to the first available non-zero time
	std::cout << "\n";
	ROS_INFO("MISSION_CONTROL: start_time: %.3f", start_time.toSec());
	current_time = ros::Time::now();  // sets current_time to the time it is now
	ROS_INFO("MISSION_CONTROL: current_time: %.3f\n", current_time.toSec());
	duration = current_time - start_time;  // calculates the duration of time since the start_time
	while (duration <= ros::Duration(5.0))
	{
		ros::spinOnce();  // update subscribers
		ros::Duration(1.0).sleep();  // sleep for a second
		std::cout << "\n";
		ROS_INFO("MISSION_CONTROL: Sleeping at 1 Hertz rate");
		ROS_INFO("MISSION_CONTROL: duration: %.3f\n", duration.toSec());
		current_time = ros::Time::now();  // sets current_time to the time it is now
		duration = current_time - start_time;  // calculates the duration of time since the start_time
	}
	if (vrx_mode)  // if vrx_mode is found to be true from initial inspection
	{
		ROS_INFO("MISSION_CONTROL:---VRX MODE ACTIVATED---\n");
	}
	else if (!vrx_mode)  // if vrx_mode is found to be false from initial inspection
	{
		ROS_INFO("MISSION_CONTROL:---USER MODE ACTIVATED---\n");
	}

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		MISSION_CONTROL_inspector();  // check that entire system is initialized before starting calculations
		ros::spinOnce();  // update subscribers
		loop_rate.sleep();  // sleep to accomplish set loop_rate
		last_time = current_time;  // update last_time
	}  // END OF while(ros::ok())

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}  // END OF main()
//.........................................................................................................END OF Main Program...........................................................................................................
