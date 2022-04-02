//  Filename:											mission_control.cpp
//  Creation Date:									03/25/2022
//  Last Revision Date:							03/28/2022
//  Author(s) [email]:								Bradley Hacker [bhacker@lssu.edu]
//  Revisor(s) [email] {Revision Date}:	Bradley Hacker [bhacker@lssu.edu] {03/28/2022}
//  Organization/Institution:					Lake Superior State University - Team AMORE
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
#include "amore/state_msg.h"												// message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"
#include "amore/usv_pose_msg.h"										// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "vrx_gazebo/Task.h"												// message published by VRX detailing current task and state
#include "amore/NED_waypoints.h"										// message that holds array of converted WF goal waypoints w/ headings and number of waypoints
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
int loop_count = 0;                                    			// loop counter, first 10 loops used to intitialize subscribers
int point = 0;                     		    						// number of points on trajectory reached 
int goal_poses;              										// total number of poses to reach 
int loop_goal_recieved;         								// this is kept in order to ensure planner doesn't start controller until the goal is published

float x_goal[100], y_goal[100], psi_goal[100];		// arrays to hold the NED goal poses
float x_usv_NED, y_usv_NED, psi_NED; 				// vehicle position and heading (pose) in NED

float e_x, e_y, e_xy, e_psi;									// current errors between goal pose and usv pose

bool NED_waypoints_published = false;				// NED_waypoints_published = false means the NED poses have not yet been calculated and published
bool NED_waypoints_recieved = false;					// NED_waypoints_recieved = false means goal position has not been acquired from "waypoints_NED"
bool E_reached = false;        								// E_reached = false means the last point has not been reached
//float e_xy_prev, e_psi_prev;								// previous errors

float e_xy_allowed = 0.4;       								// positional error tolerance threshold; NOTE: make as small as possible
float e_psi_allowed = 0.4;      								// heading error tolerance threshold; NOTE: make as small as possible

// STATES CONCERNED WITH "mission_control"
int MC_state = 0;							// 0 = On Standby; 1 = SK Planner; 2 = WF Planner; 4569 = HARD RESET (OR OTHER USE)
// STATES CONCERNED WITH "navigation_array"
int NA_state = 0;							// 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
// STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;      						// 0 = On Standby, 1 = LL controller ON
// STATES CONCERNED WITH "perception_array"
int PA_state = 0;      						// 0 = On Standby, 1 = General State, 2 = Task 3: Perception

// THE FOLLOWING FOUR BOOLS ARE USED TO DETERMINE IF THE SYSTEM HAS BEEN INITIALIZED
bool navigation_array_initialized = false;		// navigation_array_initialized = false means navigation_array is not initialized
bool propulsion_system_initialized = false;	// propulsion_system_initialized = false means propulsion_system is not initialized
bool perception_array_initialized = true;		// perception_array_initialized = false means the perception_array is not initialized !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
bool mission_control_initialized = false;			// mission_control_initialized = false means the mission_control is not initialized
//..................................................................Functions.................................................................
// SYSTEM INITIALIZATION CHECK FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////////////////////////
// THIS FUNCTION SUBSCRIBES TO THE NAVIGATION_ARRAY TO CHECK INITIALIZATION
void NAVIGATION_ARRAY_inspector(const std_msgs::Bool status)
{
	if (status.data)
	{
		navigation_array_initialized = true;
	}
	else
	{
		navigation_array_initialized = false;
	}	
} // END OF NAVIGATION_ARRAY_inspector(const std_msgs::Bool status)

// THIS FUNCTION SUBSCRIBES TO THE PROPULSION_SYSTEM TO CHECK INITIALIZATION
void PROPULSION_SYSTEM_inspector(const std_msgs::Bool status)
{
	if (status.data)
	{
		propulsion_system_initialized = true;
	}
	else
	{
		propulsion_system_initialized = false;
	}	
} // END OF PROPULSION_SYSTEM_inspector(const std_msgs::Bool status)

/* // THIS FUNCTION SUBSCRIBES TO THE PERCEPTION_ARRAY TO CHECK INITIALIZATION
void PERCEPTION_ARRAY_inspector(const std_msgs::Bool status)
{
	if (status.data)
	{
		perception_array_initialized = true;
	}
	else
	{
		perception_array_initialized = false;
	}	
} // END OF PERCEPTION_ARRAY_inspector(const std_msgs::Bool status) */

// THIS FUNCTION CHECKS INITIALIZATION OF ENTIRE SYSTEM
void MISSION_CONTROL_inspector()
{
	if ((loop_count > 10) &&  (navigation_array_initialized) && (propulsion_system_initialized) && (perception_array_initialized))
	{
		mission_control_initialized = true;
		//ROS_INFO("mission_control_initialized");
	}
	else
	{
		mission_control_initialized = false;
		ROS_INFO("!mission_control_initialized");
	}
	// UPDATE USER OF INITIALIZATION STATUSES
	/* if (navigation_array_initialized)
	{
		ROS_INFO("navigation_array_initialized");
	}
	if (propulsion_system_initialized)
	{
		ROS_INFO("propulsion_system_initialized");
	}
	if (perception_array_initialized)
	{
		ROS_INFO("perception_array_initialized");
	} */
} // END OF MISSION_CONTROL_inspector()
// END OF SYSTEM INITIALIZATION CHECK FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////////////////////////

// THIS FUNCTION SUBSCRIBES TO "goal_publish_state" TO KNOW WHEN
// NED WAYPOINTS HAVE BEEN CONVERTED AND PUBLISHED TO "waypoints_NED"
// THIS WAY WE DO NOT TRY TO SUBSCRIBE BEFORE THE TOPIC IS PUBLISHED 
void goal_publish_state_update(const std_msgs::Bool status)
{
	if (status.data)
	{
		NED_waypoints_published = true;
		//ROS_INFO("WF POINT CONVERTER FINISHED");
	}
	else
	{
		NED_waypoints_published = false;
		//ROS_INFO("WF POINT CONVERTER NOT FINISHED");
	} // if (status.data)
} // END OF goal_publish_state_update(const std_msgs::Bool status)

// THIS FUNCTION SUBSCRIBES TO "waypoints_NED" TO GET THE WF NED GOAL POSES
void WF_NED_goal_update(const amore::NED_waypoints::ConstPtr& goal) 
{
	if ((mission_control_initialized) && (MC_state == 0))				// if the system is initialized and task 2: wayfinding
	{
		if ((NED_waypoints_published) && (!NED_waypoints_recieved))	// if the NED goal waypoints have been published but not recieved yet
		{
			goal_poses = goal->quantity;
			for (int i = 0; i < goal_poses; i++)
			{
				x_goal[i] = goal->points[i].x;
				y_goal[i] = goal->points[i].y;
				psi_goal[i] = goal->points[i].z;
			}
			NED_waypoints_recieved = true;
			loop_goal_recieved = loop_count;
			
			// UPDATES STATUSES TO USER ///////////////////////////////////////////////
			ROS_INFO("GOAL POSITIONS HAVE BEEN ACQUIRED BY THE WF PLANNER.");
			ROS_INFO("Size of x_goal array: %i", goal_poses);
		} // if ((NED_waypoints_published) && (!NED_waypoints_recieved))
	}
} // END OF WF_NED_goal_update(const amore::NED_waypoints::ConstPtr& goal) 

// THIS FUNCTION SUBSCRIBES TO "usv_ned" TO GET THE CURRENT USV NED POSE
void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	if (NA_state == 1) // if navigation_array is in standard USV Pose Conversion mode 
	{
		// Update NED USV pose 
		x_usv_NED = odom->pose.pose.position.x;
		y_usv_NED = odom->pose.pose.position.y;
		psi_NED = odom->pose.pose.orientation.z;
	
		// adjust current heading back within -PI and PI
		if (psi_NED < -PI)
		{
			psi_NED = psi_NED + 2.0*PI;
		}
		if (psi_NED > PI)
		{
			psi_NED = psi_NED - 2.0*PI;
		}
	} // if navigation_array is in standard USV Pose Conversion mode
} // END OF pose_update(const nav_msgs::Odometry::ConstPtr& odom)

// THIS FUNCTION SUBSCRIBES "vrx/task/info" TO SEE THE TASK STATE
// IT ALSO TAKES INTO ACCOUNT CURRENT SYSTEM STATE
// IN ORDER TO SET SUBSYSTEM STATES, AS WELL AS "mission_control" state
void state_update(const vrx_gazebo::Task::ConstPtr& msg)
{
	// MUST SET THE FOLLOWING
	NA_state = 0;			// 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
	PS_state = 0;			// 0 = On Standby, 1 = LL controller ON
	MC_state = 0;			// 0 = On Standby; 1 = SK Planner; 2 = WF Planner; 4569 = HARD RESET (OR OTHER USE)
	PA_state = 0;      		// 0 = On Standby, 1 = General State, 2 = Task 3: Perception
	if (mission_control_initialized)
	{
		if (msg->name == "station_keeping")						// NOT DONE YET
		{
			if ((msg->state == "ready") || (msg->state == "running"))
			{
				if (NED_waypoints_recieved)
				{
					NA_state = 1;				// navigation_array is in standard USV NED Pose Conversion mode
					PS_state = 1;				// propulsion_system is turned ON
					MC_state = 1;				// mission_control is in SK Planner mode
				}
				else
				{
					NA_state = 2;				// navigation_array is in SK NED Goal Pose Conversion mode
					PS_state = 0;				// propulsion_system is turned OFF
					MC_state = 0;				// mission_control is On Standby
				}
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
				// reset task statuses as long as task is in "initial" or "finished"
				NED_waypoints_published = false;
				NED_waypoints_recieved = false;
			}
		} // (msg->name == "station_keeping")
		else if (msg->name == "wayfinding")
		{
			if ((msg->state == "ready") || (msg->state == "running"))
			{
				if (NED_waypoints_recieved)
				{
					NA_state = 1;				// navigation_array is in standard USV NED Pose Conversion mode
					PS_state = 1;				// propulsion_system is turned ON
					MC_state = 2;				// mission_control is in WF Planner mode
				}
				else
				{
					NA_state = 3;				// navigation_array is in WF NED Goal Pose Conversion mode
					PS_state = 0;				// propulsion_system is turned OFF
					MC_state = 0;				// mission_control is On Standby
				}
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
				// reset task statuses as long as task is in "initial" or "finished"
				NED_waypoints_published = false;
				NED_waypoints_recieved = false;
			}
		} // (msg->name == "wayfinding")
		else if (msg->name == "perception")										// NOT DONE YET
		{
			NA_state = 1;				// navigation_array is in standard USV NED Pose Conversion mode
			PA_state = 2;				// perception_array is in Task 3: Perception mode
			/* if ((msg->state == "ready") || (msg->state == "running"))
			{
				NA_state = 0;
				MC_state = 0;
			}
			else
			{
				NA_state = 0;
				MC_state = 0;
			} */
		} // (msg->name == "perception")
		
		// 	INTEGRATED TASK CODES FOLLOW
		/* else if (msg->name == "wildlife")
		{
			if ()
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
		} // (msg->name == "wildlife")
		else if (msg->name == "gymkhana")
		{
			if ()
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
		} // (msg->name == "gymkhana")
		else if (msg->name == "scan_dock_deliver")
		{
			if ()
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
		} // (msg->name == "scan_dock_deliver") */
		
		else
		{
			// ALL CODES ON STANDBY
			NA_state = 0;
			PS_state = 0;
			MC_state = 0;
		}
		
		// UPDATE USER OF EACH CODES STATE
		ROS_DEBUG("----------------- CURRENT STATES --------------------");
		ROS_DEBUG("NA_state: %i", NA_state);
		ROS_DEBUG("PS_state: %i", PS_state);
		ROS_DEBUG("MC_state: %i", MC_state);
		ROS_DEBUG("PA_state: %i", PA_state);
	} // if (mission_control_initialized)
} // END OF state_update(const vrx_gazebo::Task::ConstPtr& msg)
//............................................................End of Functions............................................................

int main(int argc, char **argv)
{
  // names the program for visual purposes
  ros::init(argc, argv, "Path Planner");
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  // NodeHandles
  ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh11, nh12;
  
  // Subscribers
  ros::Subscriber task_status = nh1.subscribe("/vrx/task/info", 1, state_update);															// subscriber to VRX task topic
  // INITIALIZATION CHECK SUBSCRIBERS
  ros::Subscriber NA_initialization_state_sub = nh2.subscribe("NA_initialization_state", 1, NAVIGATION_ARRAY_inspector);
  ros::Subscriber PS_initialization_state_sub = nh3.subscribe("PS_initialization_state", 1, PROPULSION_SYSTEM_inspector);
  //ros::Subscriber PA_initialization_state_sub = nh4.subscribe("PA_initialization_state", 1, PERCEPTION_ARRAY_inspector);
  ros::Subscriber ned_sub = nh5.subscribe("usv_ned", 1, pose_update);																		// subscriber for current USV pose converted to NED
  ros::Subscriber WF_waypoints_ned_sub = nh6.subscribe("waypoints_NED", 1, WF_NED_goal_update);					// subscriber for goal waypoints converted to NED
  ros::Subscriber goal_publish_state_sub = nh7.subscribe("goal_publish_state", 1, goal_publish_state_update);			// subscriber for whether or not goal waypoints have been converted yet
  
  // Publishers
  ros::Publisher WF_Planner_status_pub = nh8.advertise<std_msgs::Bool>("WF_Planner_status", 1);						// publisher for whether or not current goal has been attained
  ros::Publisher mpp_goal_pub = nh9.advertise<amore::usv_pose_msg>("mpp_goal", 1);											// publisher for current goal for low level controller (propulsion_system)
  ros::Publisher ps_state_pub = nh10.advertise<amore::state_msg>("ps_state", 1);													// publisher for current propulsion_system status
  ros::Publisher na_state_pub = nh11.advertise<amore::state_msg>("na_state", 1);													// publisher for current navigation_array status
  ros::Publisher pa_state_pub = nh12.advertise<amore::state_msg>("pa_state", 1);													// publisher for current perception_array status
  
  // Local variables
  std_msgs::Bool WF_Planner_recieved_goal;     	// "WF_Planner_status"
  amore::usv_pose_msg usv_goal_pose;				// "mpp_goal"
  amore::state_msg propulsion_system_state;		// "ps_state"
  amore::state_msg navigation_array_state;			// "na_state"
  amore::state_msg perception_array_state;			// "pa_state"
  
  // initialize header sequences
  propulsion_system_state.header.seq = 0;
  navigation_array_state.header.seq = 0;
  perception_array_state.header.seq = 0;
  
  // Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  ros::Time current_time, last_time;	// creates time variables
  current_time = ros::Time::now();						// sets current time to the time it is now
  last_time = current_time;										// sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(100);

  // ros::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  current_time = ros::Time::now();														// update current_time
	  
	  MISSION_CONTROL_inspector();													// check that entire system is initialized before starting calculations
	  
	  WF_Planner_recieved_goal.data = NED_waypoints_recieved;
	  WF_Planner_status_pub.publish(WF_Planner_recieved_goal);			// publish whether or not the WF goal pose has been acquired
	  
	  // SEND STATE TO PROPULSION_SYSTEM
	  propulsion_system_state.header.seq +=1;										// sequence number
	  propulsion_system_state.header.stamp = current_time;					// sets stamp to current time
	  propulsion_system_state.header.frame_id = "mission_control";		// header frame
	  propulsion_system_state.state.data = PS_state;								// sets propulsion system state; 0 = OFF; 1 = ON
	  ps_state_pub.publish(propulsion_system_state);								// publishes propulsion_system_state to "ps_state"
	  
	  // SEND STATE TO NAVIGATION_ARRAY
	  navigation_array_state.header.seq +=1;											// sequence number
	  navigation_array_state.header.stamp = current_time;						// sets stamp to current time
	  navigation_array_state.header.frame_id = "mission_control";			// header frame
	  navigation_array_state.state.data = NA_state;									// sets navigation_array_state; 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
	  na_state_pub.publish(navigation_array_state);									// publishes navigation_array_state to "na_state"
	  
	  // SEND STATE TO PERCEPTION_ARRAY
	  perception_array_state.header.seq +=1;											// sequence number
	  perception_array_state.header.stamp = current_time;						// sets stamp to current time
	  perception_array_state.header.frame_id = "mission_control";			// header frame
	  perception_array_state.state.data = PA_state;									// 0 = On Standby, 1 = General State, 2 = Task 3: Perception
	  pa_state_pub.publish(perception_array_state);								// publishes navigation_array_state to "pa_state"
	  
	  if ((loop_count > loop_goal_recieved) && (MC_state == 1))	// TASK 1: STATION_KEEPING
	  {
		  // PUBLISH THE CURRENT GOAL POSE
		  usv_goal_pose.header.seq = 0;
		  usv_goal_pose.header.seq +=1;										// sequence number
		  usv_goal_pose.header.stamp = current_time;					// sets stamp to current time
		  usv_goal_pose.header.frame_id = "mission_control";		// header frame
		  usv_goal_pose.position.x = x_goal[point];						// sets x-location
		  usv_goal_pose.position.y = y_goal[point];						// sets y-location
		  usv_goal_pose.position.z = 0.0;										// sets z-location
		  usv_goal_pose.psi.data = psi_goal[point];						// sets psi
		  
		  mpp_goal_pub.publish(usv_goal_pose);							// publishes goal usv pose to "mpp_goal"
	  } // if ((loop_count > loop_goal_recieved) && (MC_state == 1))

	  if ((loop_count > loop_goal_recieved) && (MC_state == 2))		// TASK 2: WAYFINDING
	  {
		  // determine error in x and y (position)
		  e_x = x_goal[point] - x_usv_NED;                                       // calculate error in x position
		  e_y = y_goal[point] - y_usv_NED;                                       // calculate error in y position
		  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
		  e_psi = psi_goal[point] - psi_NED;
		  
		  if ((e_xy < e_xy_allowed) && (e_psi < e_psi_allowed) && (!E_reached))
		  {
			  point += 1;
			  ROS_INFO("Point %i reached.", point);
			  if (point==goal_poses)
			  {
				  E_reached = true;
				  ROS_INFO("End point has been reached.\n");
			  }
		  }
	
		  if (E_reached)
		  {
			  PS_state = 0; // propulsion_system is OFF momentarily once exit point is reached
			  // reset and go to points again with a smaller tolerance threshold
			  e_xy_allowed /= 2;
			  e_psi_allowed /= 2;
			  E_reached = false; // reset and go through points again
			  point = 0;
		  }
		  else
		  {
			  PS_state = 1; // propulsion_system is switched back ON until exit point is reached
		  }
	
		  // PUBLISH THE CURRENT GOAL POSE
		  usv_goal_pose.header.seq = 0;
		  usv_goal_pose.header.seq +=1;										// sequence number
		  usv_goal_pose.header.stamp = current_time;					// sets stamp to current time
		  usv_goal_pose.header.frame_id = "mission_control";		// header frame
		  usv_goal_pose.position.x = x_goal[point];						// sets x-location
		  usv_goal_pose.position.y = y_goal[point];						// sets y-location
		  usv_goal_pose.position.z = 0.0;										// sets z-location
		  usv_goal_pose.psi.data = psi_goal[point];						// sets psi
		  
		  mpp_goal_pub.publish(usv_goal_pose);							// publishes goal usv pose to "mpp_goal"
	  } // if ((loop_count > loop_goal_recieved) && (MC_state == 2))
	  
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  // update previous stats
	  /* e_xy_prev = e_xy;
	  e_psi_prev = e_psi; */
	  last_time = current_time;
	  loop_count += 1;
  } // while(ros::ok())

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}