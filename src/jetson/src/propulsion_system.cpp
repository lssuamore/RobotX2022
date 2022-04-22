//  Filename:						        Task1_SK_Controller.cpp
//  Creation Date:						1/31/2022
//  Last Revision Date:                1/31/2022
//  Author(s) [email]:					Brad Hacker [bhacker@lssu.edu]
//                                                  
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................Task1_SK_Controller.cpp.......................
//  This code takes in a position to reach in the global frame as nav_odom msgs. 
//  It gets this position by subscribing to the "mpp_goal" node created by the Task1_SK_Planner.
//  This controller then uses PID control theory to control the heading and position of the USV.
//  This controller uses a dual-azimuthing drive configuration, which makes it over actuated. 
//   Control allocation is used to solve for the thrusts and azimuthing angles of each thruster.
//
//  Inputs and Outputs of the Task1_SK_Controller.cpp file
//				Inputs: Position (x,y) to reach in global NED frame
//				Outputs: Thruster outputs and angles for left and right thrusters


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"											// message type used for receiving NED USV state from navigation_array
#include "jetson/state_msg.h"												// message type used to recieve state of operation from mission_control
#include "std_msgs/Bool.h"
#include "jetson/usv_pose_msg.h"										// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "std_msgs/Int32.h"													// thruster commands
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    				// loop counter, first 10 loops used to intitialize subscribers
int loop_count_ON = 0;                                    		// loop count holder for when the the controller is turned on, used to ensure differential and integral terms are not started until they are calculated
bool PS_state_ON = false;									// used to set and reset the above count
//	STATES CONCERNED WITH "navigation_array"
int NA_state = 0;	//	0 = On standby		//	1 = USV NED pose converter

//	STATES CONCERNED WITH "path_planner"
int PP_state = 0;		//	0 = On standby		//	1 = Task 1: Station-Keeping			//	2 = Task 2: Wayfinding
		
//	STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;		//	0 = On standby		//	1 = Propulsion system ON

int LL_state = 1;		//	1 = PID HP Dual-azimuthing station keeping controller	//	2 = PID HP Differential wayfinding controller

float dt = 0.25;														// [s] used for differential term

float x_goal, y_goal, psi_goal;							// [m, m, radians] desired position and heading
float x_usv_NED, y_usv_NED, psi_NED; 		// vehicle position and heading (pose) in NED
float e_x, e_y, e_xy, e_psi;								// current errors between goal pose and usv pose

// initialize accumulated total errors for integral term
float e_x_total = 0;
float e_y_total = 0;
float e_xy_total = 0;
float e_psi_total = 0;
// initialize previous errors for calculating differential term
float e_x_prev = 0;
float e_y_prev = 0;
float e_xy_prev = 0;
float e_psi_prev = 0;

float T_x;																// thrust to set in x-direction in earth-fixed frame
float T_y;																// thrust to set in y-direction in earth-fixed frame
float T_x_bf;														// thrust in x-direction in body-fixed frame
float T_y_bf;														// thrust in y-direction in body-fixed frame
float M_z;															// desired moment around the z-axis

// matrix to hold outputs of controlller
//float tau[3][1];

float T_p;																// used to set the port (left) thruster output
float T_s;																// used to set the starboard (right) thruster output
float A_p;															// used to set the port (left) thruster angle
float A_s;															// used to set the starboard (right) thruster angle

// Used for correction of saturated thrust values 
float T_p_C;														// Corrected port (left) thrust output
float T_s_C;														// Corrected starboard (right) thrust output

float Kp_x, Kp_y, Kp_psi;									// Proportional gains
float Kd_x, Kd_y, Kd_psi;									// Differential gains
float Ki_x, Ki_y, Ki_psi;										// Integration gains

float B = 2.0;                  // [m] FAU FOUND IT TO BE 2.0
/* 
float L = 4.6;														// [m] length of USV, 16 ft or 4.88 m

// Transformation Matrix to convert from external dynamics to internal dynamics
float Transform[3][4] = {
   {1.0, 0.0, 1.0, 0.0} ,
   {0.0, 1.0, 0.0, 1.0} ,
   {1.0, -2.3, -1.0, -2.3}
};

float Transform_transpose[4][3] = {
   {1.0, 0.0, 1.0} ,
   {0.0, 1.0, -2.3} ,
   {1.0,0.0, -1.0} ,
   {0.0,1.0, -2.3}
};
}; */

// this is the pseudoinverse of the transform matrix from
// forces experienced directly at the thrusters to forces experienced at the USV COG
// Transform_pseudoinverse = Transform_transpose * inv(Transform*Transform_transpose)
float Transform_pseudoinverse[4][3] = {
   {0.5, 1.15, 0.5} ,
   {0.0, 0.5, 0.0} ,
   {0.5, -1.15, -0.5} ,
   {0.0, 0.5, 0.0}
};

// Control Allocation output
float F_xp;																			// force in the x-direction on the port side
float F_yp;																			// force in the y-direction on the port side
float F_xs;																			// force in the x-direction on the starboard side
float F_ys;																			// force in the y-direction on the starboard side

std_msgs::Bool ps_initialization_status;							// "ps_initialization_state" message
ros::Publisher ps_initialization_state_pub;						// "ps_initialization_state" publisher

ros::Time current_time, last_time;									// creates time variables
//..............................................................End of Global Variables............................................................


//..................................................................Functions.................................................................
// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "ps_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void PROPULSION_SYSTEM_inspector()
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;									// increment loop counter
	if (loop_count > 3)
	{
		ps_initialization_status.data = true;
		//ROS_INFO("propulsion_system_initialized -- PS");
	}
	else
	{
		ps_initialization_status.data = false;
		ROS_INFO("!propulsion_system_initialized -- PS");
	}
	ps_initialization_state_pub.publish(ps_initialization_status);						// publish the initialization status of the propulsion_system to "ps_initialization_state"
} // END OF PROPULSION_SYSTEM_inspector()

/////////////////////////////////////////////////////////////////		STATE UPDATERS		///////////////////////////////////////////////////////////////////
// THIS FUNCTION: Updates the state of "navigation_array" given by "mission_control"
// ACCEPTS: navigation_array state_msg from "na_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void na_state_update(const jetson::state_msg::ConstPtr& msg)
{
	if (ps_initialization_status.data)
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
	if (ps_initialization_status.data)
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
	if (ps_initialization_status.data)
	{
		PS_state = msg->state.data;
	}
	if ((!PS_state_ON) && (PP_state == 1))
	{
		PS_state_ON = true;
		loop_count_ON = loop_count;
	}
	else
	{
		PS_state_ON = false;
	}
} // END OF ps_state_update()

// THIS FUNCTION: Updates the current NED USV pose converted through the navigation_array
// ACCEPTS: Current NED USV pose and velocities from "nav_ned"
// RETURNS: (VOID)
// =============================================================================
void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	if (NA_state == 1)		// if navigation_array is in standard USV NED pose converter mode 
	{
		// Update NED USV pose 
		x_usv_NED = odom->pose.pose.position.x;
		y_usv_NED = odom->pose.pose.position.y;
		psi_NED = odom->pose.pose.orientation.z;
	}
} // END OF pose_update()

// THIS FUNCTION: Updates the goal pose for the propulsion_system given by path_planner
// ACCEPTS: usv_pose_msg from "current_goal_pose"
// RETURNS: (VOID)
// =============================================================================
void goal_pose_update(const jetson::usv_pose_msg::ConstPtr& goal) 
{
	if ((PS_state == 1) && (PP_state != 0))
	{
		x_goal = goal->position.x;		
		y_goal = goal->position.y;
		psi_goal = goal->psi.data;
	}
} // END OF goal_pose_update()

// THIS FUNCTION UPDATES THE GAINS AND WHICH LL CONTROLLER SHOULD BE USED 
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void update_gains_LL_controller()
{
	// Get the LL_state which decides which controller to use
	ros::param::get("/LL_state_G", LL_state);
	ROS_DEBUG("LL_state: %6i ~ 0 = Dual-azimuthing station keeping controller, 1 = Differential wayfinding controller ~ --PS", LL_state);
	// Get the PID gains
	ros::param::get("/Kp_xy_G", Kp_x);
	Kp_y = Kp_x; //ros::param::get("/Kp_y_G", Kp_y);
	ros::param::get("/Kp_psi_G", Kp_psi);
	ros::param::get("/Kd_xy_G", Kd_x);
	Kd_y = Kd_x; //ros::param::get("/Kd_y_G", Kd_y);
	ros::param::get("/Kd_psi_G", Kd_psi);
	ros::param::get("/Ki_xy_G", Ki_x);
	Ki_y = Ki_x; //::param::get("/Ki_y_G", Ki_y);
	ros::param::get("/Ki_psi_G", Ki_psi);
} // END OF update_gains_LL_controller()

// THIS FUNCTION DISPLAYS THE UPDATED GAINS
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void display_gains()
{
	// PROPORTIONAL GAINS
	ROS_INFO("Kp_xy is: %.2f --PS", Kp_x);
	ROS_INFO("Kp_psi is: %.2f --PS\n", Kp_psi);
	// DERIVATIVE GAINS
	ROS_INFO("Kd_xy is: %.2f --PS", Kd_x);
	ROS_INFO("Kd_psi is: %.2f --PS\n", Kd_psi);
	// INTEGRAL GAINS
	ROS_INFO("Ki_xy is: %.2f --PS", Ki_x);
	ROS_INFO("Ki_psi is: %.2f --PS\n", Ki_psi);
} // END OF display_gains()

// THIS FUNCTION: Resets the integral term once the errors become minimal 
//									to avoid overshooting the goal if a huge effort's built up
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void Integral_reset()
{
	if ((float)abs(e_x) < 0.1)		// NOTE: this value may need adjusting
	{
	  e_x_total = 0.0;
	}
	if ((float)abs(e_y) < 0.1)		// NOTE: this value may need adjusting
	{
	  e_y_total = 0.0;
	}
	if ((float)abs(e_xy) < 1.0)
	{
	  e_xy_total = 0.0;
	}
	if ((float)abs(e_psi) < 0.5)	// NOTE: this value may need adjusting
	{
	  e_psi_total = 0.0;
	}
} // END OF Integral_reset()

// THIS FUNCTION: Checks for negative thrust values and corrects if so
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void negative_thrust_correction_check()
{
	// check for negative thrust values
	if (T_p < 0.0)
	{
		T_p = -1.0 * T_p;
		A_p = A_p + PI;
	}
	if (T_s < 0.0)
	{
		T_s = -1.0 * T_s;
		A_s = A_s + PI;
	}
} // END OF negative_thrust_correction_check()

// THIS FUNCTION: Checks that angles are between 0 and 2PI and corrects if so, then converts radians to degrees 
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void angle_correction_check()
{
	while ((A_p > 2.0*PI) || (A_p < 0.0) || (A_s > 2.0*PI) || (A_s < 0.0))
	{
		// Angles to thrusters can only be set between 0 and 2PI
		if (A_p > 2.0*PI)
		{
			A_p = A_p - 2.0*PI;
		}
		else if (A_p < 0.0)
		{
			A_p = A_p + 2.0*PI;
		}
		if (A_s > 2.0*PI)
		{
			A_s = A_s - 2.0*PI;
		}
		else if (A_s < 0.0)
		{
			A_s = A_s + 2.0*PI;
		}
	}
	// convert to degrees
	A_p = A_p * (180/PI);
	A_s = A_s * (180/PI);
} // END OF angle_correction_check()

// THIS FUNCTION: Checks for saturation of thrust outputs and corrects if so
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void thrust_saturation_check()
{
	// check for saturation
	if (((float)abs(T_p) > 1.0) || ((float)abs(T_s) > 1.0))
	{
		// correct for saturation by normalizing thrust data
		if ((float)abs(T_p) > (float)abs(T_s))
		{
		  //ROS_DEBUG("LT IS ISSUE!");
		  T_p_C= T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		else if ((float)abs(T_p) < (float)abs(T_s))
		{
		  //ROS_DEBUG("RT IS ISSUE!");
		  T_p_C = T_p / (float)abs(T_s);
		  T_s_C = T_s / (float)abs(T_s);
		}
		else 
		{
		  //ROS_DEBUG("Equal");
		  //ROS_DEBUG("Divide by : %f\n", (float)abs(T_p));      // displays right thrust value
		  T_p_C = T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		T_p = T_p_C;
		T_s = T_s_C;
		ROS_DEBUG("Port Thrust corrected: %.2f --PS", T_p);
		ROS_DEBUG("Stbd Thrust corrected: %.2f --PS\n", T_s);
	}
	
	/* // make less aggressive by only allowing half the output possible
	if (((float)abs(T_p) > 0.5) || ((float)abs(T_s) > 0.5))
	{
		// correct for saturation by normalizing thrust data
		if ((float)abs(T_p) > (float)abs(T_s))
		{
		  //ROS_DEBUG("LT IS ISSUE!");
		  T_p_C= T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		else if ((float)abs(T_p) < (float)abs(T_s))
		{
		  //ROS_DEBUG("RT IS ISSUE!");
		  T_p_C = T_p / (float)abs(T_s);
		  T_s_C = T_s / (float)abs(T_s);
		}
		else 
		{
		  //ROS_DEBUG("Equal");
		  //ROS_DEBUG("Divide by : %f\n", (float)abs(T_p));      // displays right thrust value
		  T_p_C = T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		T_p = T_p_C*0.5;
		T_s = T_s_C*0.5;
		ROS_DEBUG("LT after extra correction is: %f\n", T_p);            // displays left thrust value
		ROS_DEBUG("RT after extra correction is: %f\n", T_s);          // displays right thrust value
	} */
	
	// Scale the thrust value from between 0.0 - 1.0 to 0.0 - 55.0
	T_p = T_p * 55.0;
	T_s = T_s * 55.0;
	
	// Use bollard pull test data to output the thrust value from 140 - 255
	T_p = 144 + (2.5)*T_p + (-0.02)*pow(T_p,2.0) + (-6.59e-04)*pow(T_p,3.0) + (1.47e-05)*pow(T_p,4.0);
	T_s = 144 + (2.5)*T_s + (-0.02)*pow(T_s,2.0) + (-6.59e-04)*pow(T_s,3.0) + (1.47e-05)*pow(T_s,4.0);
	
} // END OF thrust_saturation_check()
//............................................................End of Functions............................................................


int main(int argc, char **argv)
{
	//names the program for visual purposes
	ros::init(argc, argv, "propulsion_system");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh11;

	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, na_state_update);
	ros::Subscriber pp_state_sub = nh2.subscribe("pp_state", 1, pp_state_update);
	ros::Subscriber ps_state_sub = nh3.subscribe("ps_state", 1, ps_state_update);
	ros::Subscriber nav_NED_sub = nh4.subscribe("nav_ned", 1, pose_update);														// Obtains the USV pose in global NED from mission_control
	ros::Subscriber current_goal_pose_sub = nh5.subscribe("current_goal_pose", 1, goal_pose_update);				// goal pose given by path planners

	// Publishers
	ps_initialization_state_pub = nh7.advertise<std_msgs::Bool>("ps_initialization_state", 1);		// state of initialization
	ros::Publisher stbd_T_pub = nh7.advertise<std_msgs::Int32>("thruster_int_right", 10);			// float value between -1.0 and 1.0, speed to right thruster
	ros::Publisher port_T_pub = nh8.advertise<std_msgs::Int32>("thruster_int_left", 10);				// float value between -1.0 and 1.0, speed to left thruster
	ros::Publisher stbd_A_pub = nh9.advertise<std_msgs::Int32>("angle_int_right", 10);				// float value between -PI to PI, angle to right thruster
	ros::Publisher port_A_pub = nh10.advertise<std_msgs::Int32>("angle_int_left", 10);				// float value between -PI to PI, angle to left thruster

	// Local variables
	std_msgs::Int32 LT, RT, LA, RA;									// LT is left thrust, RT is right thrust, LA is left thruster angle, RA is right thruster angle

	// Initialize global variables
	ps_initialization_status.data = false;
	current_time = ros::Time::now();										// sets current time to the time it is now
	last_time = current_time;												// sets last time to the time it is now

	//sets the frequency for which the program sleeps at 100 = 1/100 second
	ros::Rate loop_rate(3);

	//rosk::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		PROPULSION_SYSTEM_inspector();											// check initialization status and update ps_initialization_status

		if (PS_state == 1)
		{
			update_gains_LL_controller();										// Update all gain parameters in launch file 
			//display_gains();									// UNCOMMENT IF YOU WANT TO PRINT GAINS TO USER 

			Integral_reset();										// Reset integral term once the errors become minimal 

			// determine error in x and y (position)
			e_x = x_goal - x_usv_NED;
			e_y = y_goal - y_usv_NED;
			e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));		// calculate magnitude of positional error

			/* // if within error, have selective gains
			if (e_xy < 1.0) 
			{
				Kp_x = 0.7;
				Kp_y = 0.7;
				Kd_x = 0.2;
				Kd_y = 0.2;
				Ki_x = 0.0;
				Ki_y = 0.0;
			} */

			// position control law
			if (loop_count > (loop_count_ON + 2)) // don't include differential term or integration term first 2 loops after being turned ON	
			{
				if (LL_state == 1)			//	1 = PID HP Dual-azimuthing station keeping controller
				{
					T_x = Kp_x*e_x;
					T_y = Kp_y*e_y;
				}
				else if (LL_state == 2)	//	2 = PID HP Differential wayfinding controller
				{
					T_x = Kp_x*e_xy;
				}
			}
			else
			{
				if (LL_state == 1)			//	1 = PID HP Dual-azimuthing station keeping controller
				{
					// trapezoidal integration of errors for integral term
					e_x_total = e_x_total + ((e_x_prev + e_x)/2.0)*dt; 
					e_y_total = e_y_total + ((e_y_prev + e_y)/2.0)*dt;
					T_x = Kp_x*e_x + Kd_x*((e_x - e_x_prev)/dt) + Ki_x*e_x_total;
					T_y = Kp_y*e_y + Kd_y*((e_y - e_y_prev)/dt) + Ki_y*e_y_total;
				}
				else if (LL_state == 2)	//	2 = PID HP Differential wayfinding controller
				{
					e_xy_total = e_xy_total + ((e_xy_prev + e_xy)/2.0)*dt;	// trapezoidal integration of errors for integral term
					T_x = Kp_x*e_xy+ Kd_x*((e_xy - e_xy_prev)/dt) + Ki_x*e_xy_total;
				}
			}

			// Do not give desired heading at goal until within 10.0 meters of goal position
			// Until then, give the desired heading to travel to goal location
			if ((e_xy > 10.0) || (LL_state == 2))
			{
				psi_goal = atan2(e_y,e_x);       // [radians]
			}

			e_psi = psi_goal - psi_NED;
			
			// correct discontinuity in heading error
			if (e_psi < ((-2.0*PI) + (0.05*2.0*PI)))
			{
				e_psi = e_psi + 2.0*PI;
			}
			if (e_psi > ((2.0*PI) - (0.05*2.0*PI)))
			{
				e_psi = e_psi - 2.0*PI;
			}

			// ensure shortest turn is used
			if (e_psi < -PI)
			{
				e_psi = e_psi + 2.0*PI;
			}
			if (e_psi > PI)
			{
				e_psi = e_psi - 2.0*PI;
			}

			/* // if within error, have selective gains
			if ((float)abs(e_psi) < 0.1)
			{
				Kp_psi = 0.0;
				Kd_psi = 0.0;
				Ki_psi = 0.0;
			} */

			// orientation control law
			if (loop_count > (loop_count_ON + 2)) // don't include differential term or integration term first time through
			{
				M_z = Kp_psi*e_psi;
			}
			else
			{
				e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt;		// trapezoidal integration of error for integral term
				M_z = Kp_psi*e_psi + Kd_psi*((e_psi - e_psi_prev)/dt) + Ki_psi*e_psi_total;
			}

			// UPDATES STATUSES TO USER ///////////////////////////////////////////////
			ROS_DEBUG("x_goal: %.2f --PS", x_goal);
			ROS_DEBUG("y_goal: %.2f --PS", y_goal);
			ROS_DEBUG("des_psi: %.2f --PS\n", psi_goal);

			ROS_DEBUG("x_usv: %.2f --PS", x_usv_NED);
			ROS_DEBUG("y_usv: %.2f --PS", y_usv_NED);
			ROS_DEBUG("psi_NED: %.2f --PS\n", psi_NED);

			ROS_DEBUG("e_x: %.2f --PS", e_x);                             // x posn. error
			ROS_DEBUG("e_y: %.2f --PS", e_y);                             // y posn. error
			ROS_DEBUG("e_xy: %.2f --PS", e_xy);                             // magnitude of posn. error
			ROS_DEBUG("e_psi: %.2f --PS\n", e_psi);                        // heading error	  

			/* ROS_DEBUG("Control Efforts --PS");
			ROS_DEBUG("T_x_G: %.3f --PS", T_x);
			ROS_DEBUG("T_y_G: %.3f --PS", T_y);
			ROS_DEBUG("M_z: %.3f --PS\n", M_z); */

			/* // if within a meter only control heading
			if (e_xy < 1.0) 
			{
				T_x = 0.0;
				T_y = 0.0;
			} */

			// only control heading if heading is off by more than 45 degree
			if ((float)abs(e_psi) > 0.785)
			{
				T_x = 0.0;
				T_y = 0.0;
			}

			/* // if errors are small enough, do not try to correct for them
			if ((float)abs(e_x) < 0.1)
			{
				T_x = 0.0;
			}
			if ((float)abs(e_y) < 0.1)
			{
				T_y = 0.0;
			}
			if ((float)abs(e_psi) < 0.1)
			{
				M_z = 0.0;
			} */
			
			if (LL_state == 1)				//	1 = PID HP Dual-azimuthing station keeping controller
			{
				// Convert to USV body-fixed frame from global frame
				T_x_bf = T_x*cos(psi_NED) + T_y*sin(psi_NED);
				T_y_bf = T_y*cos(psi_NED) - T_x*sin(psi_NED);
				/* T_x = (float)abs(T_x_bf);
				T_y = (float)abs(T_y_bf); */

				T_x = T_x_bf;
				T_y = T_y_bf;

				// calculate the control allocation outputs
				// f = Transform_pseudoinverse * tau;
				F_xp = Transform_pseudoinverse[0][0]*T_x + Transform_pseudoinverse[0][1]*T_y + Transform_pseudoinverse[0][2]*M_z;
				F_yp = Transform_pseudoinverse[1][0]*T_x + Transform_pseudoinverse[1][1]*T_y + Transform_pseudoinverse[1][2]*M_z;
				F_xs = Transform_pseudoinverse[2][0]*T_x + Transform_pseudoinverse[2][1]*T_y + Transform_pseudoinverse[2][2]*M_z;
				F_ys = Transform_pseudoinverse[3][0]*T_x + Transform_pseudoinverse[3][1]*T_y + Transform_pseudoinverse[3][2]*M_z;

				T_p = sqrt(pow(F_xp,2.0)+pow(F_yp,2.0));		// calculate magnitude of port thrust
				T_s = sqrt(pow(F_xs,2.0)+pow(F_ys,2.0));		// calculate magnitude of starboard thrust
				A_p = -atan2(F_yp,F_xp);									// calculate angle of port thrust
				A_s = -atan2(F_ys,F_xs);									// calculate angle of starboard thrust

				// DEBUG INFORMATION ////////////////////////////////////////////////////////////
				/* // Proportional, Derivative, and Integral amounts of control effort
				ROS_DEBUG("Control Effort Information");
				ROS_DEBUG("T_x_P: %f", Kp_x*e_x);
				ROS_DEBUG("T_x_D: %f", Kd_x*((e_x - e_x_prev)/dt));
				ROS_DEBUG("T_x_I: %f", Ki_x*e_x_total);
				ROS_DEBUG("T_y_P: %f", Kp_y*e_y);
				ROS_DEBUG("T_y_D: %f", Kd_y*((e_y - e_y_prev)/dt));
				ROS_DEBUG("T_y_I: %f", Ki_y*e_y_total);
				ROS_DEBUG("M_z_P: %f", Kp_psi*e_psi);
				ROS_DEBUG("M_z_D: %f", Kd_psi*((e_psi - e_psi_prev)/dt));
				ROS_DEBUG("M_z_I: %f\n", Ki_psi*e_psi_total);

				ROS_DEBUG("BEFORE SWAP TO BODY-FIXED FRAME");
				ROS_DEBUG("T_x_G: %f", T_x);
				ROS_DEBUG("T_y_G: %f", T_y);
				ROS_DEBUG("M_z: %f", M_z);
				ROS_DEBUG("AFTER SWAP");
				ROS_DEBUG("T_x_bf: %f", T_x_bf);
				ROS_DEBUG("T_y_bf: %f\n", T_y_bf);

				// Print f values
				ROS_DEBUG("F_xp: %f", F_xp);
				ROS_DEBUG("F_yp: %f", F_yp);
				ROS_DEBUG("F_xs: %f", F_xs);
				ROS_DEBUG("F_ys: %f\n", F_ys); */
			}
			else if (LL_state == 2)		//	2 = PID HP Differential wayfinding controller
			{
				// Calculate torque to thrusters
				T_p = T_x/2.0 + M_z/B;
				T_s = T_x/2.0 - M_z/B;

				// Set thruster angles to zero since it is differential drive
				A_p = 0.0;
				A_s = 0.0;

				/* // display contributions of output 
				ROS_DEBUG("LT_T is: %f\n", T_x/2.0);			// displays left thrust value
				ROS_DEBUG("LT_M is: %f\n", M_z/B);			// displays right thrust value
				ROS_DEBUG("RT_T is: %f\n", T_x/2.0);			// displays left thrust value
				ROS_DEBUG("RT_M is: %f\n", -M_z/B);			// displays right thrust value

				ROS_DEBUG("LT is: %f\n", T_p);						// displays left thrust value
				ROS_DEBUG("RT is: %f\n", T_s);					// displays right thrust value */
			}
			
			ROS_DEBUG("Before corrections --------------");
			ROS_DEBUG("Port Thrust: %.2f", T_p);
			ROS_DEBUG("Stbd Thrust: %.2f", T_s);
			ROS_DEBUG("Port Angle: %.2f", A_p);
			ROS_DEBUG("Stbd Angle: %.2f", A_s);
			
			negative_thrust_correction_check();
			angle_correction_check();
			
			ROS_DEBUG("After corrections before saturation check ---------------");
			ROS_DEBUG("Port Thrust: %.2f", T_p);
			ROS_DEBUG("Stbd Thrust: %.2f", T_s);
			ROS_DEBUG("Port Angle: %.2f", A_p);
			ROS_DEBUG("Stbd Angle: %.2f\n", A_s);

			thrust_saturation_check();
			
			ROS_DEBUG("After saturation check ---------------");
			ROS_DEBUG("Port Thrust: %.2f", T_p);
			ROS_DEBUG("Stbd Thrust: %.2f", T_s);

			// only print to thrusters if far enough into loop to have correct calculations
			// for some reason the first 4 times through loop the current pose variables do not update
			// give time to let path planners and pose converters stabilize
			if ((loop_count>30) && (PS_state == 1))                                                 
			{ 
				LA.data = (int)A_p;
				LT.data = (int)T_p;
				RA.data = (int)A_s;
				RT.data = (int)T_s;
				port_A_pub.publish(LA);
				port_T_pub.publish(LT);
				stbd_A_pub.publish(RA);
				stbd_T_pub.publish(RT);
			}

			// update previous errors
			e_x_prev = e_x;
			e_y_prev = e_y;
			e_xy_prev = e_xy;
			e_psi_prev = e_psi;
		} // if (PS_state == 1)
		else // (PS_state == 0), Controller is turned off by command from high level
		{
			LA.data = 0;
			LT.data = 0;
			RA.data = 0;
			RT.data = 0;
			port_A_pub.publish(LA);
			port_T_pub.publish(LT);
			stbd_A_pub.publish(RA);
			stbd_T_pub.publish(RT);
		}

		ros::spinOnce();										// update subscribers
		loop_rate.sleep();									// sleep for set loop_rate
		last_time = current_time;						// update last_time
	} // while(ros::ok())
	LT.data = 0;
	port_T_pub.publish(LT);
	RT.data = 0;	
	stbd_T_pub.publish(RT);

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}
