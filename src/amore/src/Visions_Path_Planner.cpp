//  Filename:						        Visions_Path_Planner.cpp
//  Creation Date:						12/4/2021
//  Last Revision Date:                
//  Author(s) [email]:					Brad Hacker [bhacker@lssu.edu]
//                                                  Shaede Perzanowksi [sperzanowski1@lssu.edu]
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................About Visions_Path_Planner.cpp.......................
//  This code takes in the position of two buoys wrt the USV, which are converted to global positions,
//  which are then used along with the current location of the USV to calculate the
//  midpoint between the buoys in the global frame, along with an approach point to reach before going to the midpoint,
//  as well as an exit point to reach after reaching the midpoint.
//  This code then sends these points to the node "mpp_goal" so they can be accessed by the low-level controllers.
//
//  Inputs and Outputs of the Visions_Path_Planner.cpp file
//				Inputs: Left and right buoy locations (x,y) wrt USV
//				Outputs: Locations for intermediate approach point, midpoint between buoys, and exit point (x,y)

// Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "time.h"

// include necessary message libraries
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

// DEFINING GLOBAL VARIABLES
#define PI 3.14159265

// Global Variables ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int loop_count=0;            // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
float x_goal;
float y_goal;
// for getting vehicle position and heading
float x_usv_ENU;           // this variable is updated as the WAM-V x-position in ENU in real time by transfering lat and long
float y_usv_ENU;           // this variable is updated as the WAM-V y-position in ENU in real time by transfering lat and long
float x_usv_NED;           // this variable is updated as the WAM-V x-position in NED
float y_usv_NED;           // this variable is updated as the WAM-V y-position in NED
// q0 - q3 are current orientation in quaternion form
float q1;
float q2;
float q3;
float q0;
float psi_ENU;                // this variable is updated as the WAM-V z-orientation in ENU in real time through the compass
float psi_NED;                // this variable is updated as the WAM-V z-orientation in NED

float ON_OFF = 0.0;         // if this variable is a 0.0, the controller is OFF, if it is a 1.0, the controller is ON

// for calculating desired poses
float CL_x; // x-location of left buoy wrt USV
float CL_y; // y-location of left buoy wrt USV
float CR_x; // x-location of right buoy wrt USV
float CR_y; // y-location of right buoy wrt USV

float CL_x_NED; // x-location of left buoy centroid in global frame
float CL_y_NED; // y-location of left buoy centroid in global frame
float CR_x_NED; // x-location of right buoy centroid in global frame
float CR_y_NED; // y-location of right buoy centroid in global frame

float M_x; // x-location of midpoint
float M_y; // y-location of midpoint

float d_L; // distance from USV to left buoy
float d_M; // distance from USV to midpoint
float d_R; // distance from USV to right buoy
float d_I; // distance from midpoint to approach point

float d_LM; // half distance between left and right buoys
float a_L; // distance between left buoy and the approach point
float theta; // angle created by d_I and d_LM

float x_I_CL; // x-coord. of intermediate point wrt left buoy 
float y_I_CL; // y-coord. of intermediate point wrt left buoy 

float x_E_CL; // x-coord. of exit point wrt left buoy 
float y_E_CL; // y-coord. of exit point wrt left buoy 

float s_M; // slope of of line from CL to CR 
float alpha; // angle of frame CL wrt global frame 

float I_x; // x-coord. of intermediate point wrt global 
float I_y; // y-coord. of intermediate point wrt global

float E_x; // x-coord. of exit point wrt global 
float E_y; // y-coord. of exit point wrt global

float e_x;                        // this is calculated as the error between the desired x-pos. and the actual x-pos.
float e_y;                        // this is calculated as the error between the desired y-pos. and the actual y-pos.
float e_xy; // this is calculated as the error between the desired xy-pos. and the actual xy-pos.

bool calculations_done = false; // if calculations_done is true, this means the intermediate approach point and the midpoint have been calculated
bool I_reached = false; // if I_reached is false this means the intermediate approach point has not been reached
int I_reached_loop = -1; // keeps track of which loop the intermediate approach point is reached to avoid midpoint reach flip on the same loop
bool M_reached = false; // if M_reached is false this means the midpoint has not been reached
int M_reached_loop = -1; // keeps track of which loop the midpoint is reached to avoid E_reached to flip on the same loop
bool E_reached = false; // if E_reached is false this means the exit point has not been reached

void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	x_usv_ENU = odom->pose.pose.position.x;
	y_usv_ENU = odom->pose.pose.position.y;
	q1 = odom->pose.pose.orientation.x;
	q2 = odom->pose.pose.orientation.y;
	q3 = odom->pose.pose.orientation.z;
	q0 = odom->pose.pose.orientation.w;
	
	// convert current heading from quaternion into radians
	psi_ENU = atan2((2.0*(q3*q0 + q1*q2)) , (1 - 2*(pow(q2,2.0) + pow(q3,2.0)))); // orientation off x-axis
	
	// convert from ENU to NED
    x_usv_NED = y_usv_ENU;
	y_usv_NED = x_usv_ENU;
	psi_NED = PI/2.0 - psi_ENU;
	// adjust current heading back within -PI and PI
	if (psi_NED < -PI)
	{
   	  psi_NED = psi_NED + 2*PI;
	}
	if (psi_NED > PI)
	{
	  psi_NED = psi_NED - 2*PI;
	} 
}

// update left buoy centroid location (x,y) in global frame
void left_buoy_x_update (const std_msgs::Float32 lb_x_loc)
{
	CL_x = lb_x_loc.data;
}
void left_buoy_y_update (const std_msgs::Float32 lb_y_loc)
{
	CL_y = lb_y_loc.data;
}

// update right buoy centroid location (x,y) in global frame
void right_buoy_x_update (const std_msgs::Float32 rb_x_loc)
{
	CR_x = rb_x_loc.data;
}
void right_buoy_y_update (const std_msgs::Float32 rb_y_loc)
{
	CR_y = rb_y_loc.data;
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "Visions Path Planner");
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  // set up NodeHandles
  ros::NodeHandle nh1;  // this node handle is used for subscribing to geonav_odom which provides position in x, y
  ros::NodeHandle nh2;  // subscriber to left_buoy_x_loc
  ros::NodeHandle nh3;  // subscriber to left_buoy_y_loc
  ros::NodeHandle nh4;  // subscriber to right_buoy_x_loc
  ros::NodeHandle nh5;  // subscriber to right_buoy_y_loc
  ros::NodeHandle nh6;  // publisher to mpp_goal
  
  // start publishers and subscribers
  ros::Subscriber nav_sub = nh1.subscribe("geonav_odom", 1, pose_update);  // subscribes to geonav_odom to get USV pose in global frame
  ros::Subscriber left_buoy_x_loc_sub = nh2.subscribe("left_x", 1, left_buoy_x_update);  // subscribes to left_buoy_x_loc in global frame
  ros::Subscriber left_buoy_y_loc_sub = nh3.subscribe("left_y", 1, left_buoy_y_update);  // subscribes to left_buoy_y_loc in global frame
  ros::Subscriber right_buoy_x_loc_sub = nh4.subscribe("right_x", 1, right_buoy_x_update);  // subscribes to right_buoy_x_loc in global frame
  ros::Subscriber right_buoy_y_loc_sub = nh5.subscribe("right_y", 1, right_buoy_y_update);  // subscribes to right_buoy_y_loc in global frame
  ros::Publisher mpp_goal_pub = nh6.advertise<nav_msgs::Odometry>("mpp_goal", 10); //publishes poses of midpoint and approach points for low level controller
  
  // local variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  nav_msgs::Odometry nav_odom; // mpp_goal position
  ros::Time current_time, last_time;  // creates time variables
  current_time = ros::Time::now();   // sets current time to the time it is now
  last_time = ros::Time::now();        // sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(4);
  ros::spinOnce();                             // initialize position
  loop_rate.sleep();  

  // ros::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	current_time = ros::Time::now(); // update time
	
	if (loop_count > 20)
	{
	  // only calculate the midpoint and the intermediate approach point once
	  if (!calculations_done)
	  {
		/* // hardcode values
		CL_x = 37.85;
		CL_y = 2.2;
		CR_x = 37.85;
		CR_y = 15.75; */
		/* x_usv_NED = 24.24;
		y_usv_NED = 11.88;
		psi_NED = 0.0; */
		ROS_DEBUG("________vvvvvvvvvvv  {PP}  vvvvvvvvvvv___________________\n");
		ROS_DEBUG("~~~USV POSE~~~");
		ROS_DEBUG("Psi_NED: %f", psi_NED);
		ROS_DEBUG("x_NED: %f", x_usv_NED);
		ROS_DEBUG("y_NED: %f\n", y_usv_NED);
		ROS_DEBUG("~~~Buoy locations wrt USV~~~");
		ROS_DEBUG("LB_x: %f", CL_x);
		ROS_DEBUG("LB_y: %f", CL_y);
		ROS_DEBUG("RB_x: %f", CR_x);
		ROS_DEBUG("RB_y: %f\n", CR_y);
		CL_x_NED = cos(psi_NED)*CL_x - sin(psi_NED)*CL_y + x_usv_NED;
		CL_y_NED = sin(psi_NED)*CL_x + cos(psi_NED)*CL_y + y_usv_NED;
		CR_x_NED = cos(psi_NED)*CR_x - sin(psi_NED)*CR_y + x_usv_NED;
		CR_y_NED = sin(psi_NED)*CR_x + cos(psi_NED)*CR_y + y_usv_NED;
		ROS_DEBUG("~~~Buoy locations in NED frame~~~");
		ROS_DEBUG("LB_x: %f", CL_x_NED);
		ROS_DEBUG("LB_y: %f", CL_y_NED);
		ROS_DEBUG("RB_x: %f", CR_x_NED);
		ROS_DEBUG("RB_y: %f\n", CR_y_NED);
		
		M_x = (CL_x_NED+CR_x_NED)/2.0; // x-location of midpoint
		M_y = (CL_y_NED+CR_y_NED)/2.0; // y-location of midpoint
		
		/* ROS_DEBUG("x_usv_NED: %f\n", x_usv_NED);
		ROS_DEBUG("y_usv_NED: %f\n", y_usv_NED); */
		d_L = sqrt(pow((CL_x_NED-x_usv_NED),2.0)+pow((CL_y_NED-y_usv_NED),2.0));    // distance from USV to left buoy
		d_M = sqrt(pow((M_x-x_usv_NED),2.0)+pow((M_y-y_usv_NED),2.0));      // distance from USV to midpoint
		d_R = sqrt(pow((CR_x_NED-x_usv_NED),2.0)+pow((CR_y_NED-y_usv_NED),2.0));  // distance from USV to right buoy
		d_I = (d_L+d_M+d_R)/3.0; // distance from midpoint to approach point
		
		// intermediate approach point doesn't need to be more than 8 meters back from midpoint
		if (d_I > 4.0)
		{
		  d_I = 4.0;
		}
		/* ROS_DEBUG("d_L: %f\n", d_L);
		ROS_DEBUG("d_M: %f\n", d_M);
		ROS_DEBUG("d_R: %f\n", d_R);
		ROS_DEBUG("d_I: %f\n", d_I); */
		
		d_LM = 0.5*sqrt(pow((CR_x_NED-CL_x_NED),2.0)+pow((CR_y_NED-CL_y_NED),2.0));                // half distance between left and right buoys
		a_L = sqrt(pow(d_LM,2.0)+pow(d_I,2.0));                                                       // distance between left buoy and the approach point
		theta = atan(d_I/d_LM);                                                                                    // angle created by d_I and d_LM
		/* ROS_DEBUG("d_LM: %f\n", d_LM);
		ROS_DEBUG("a_L: %f\n", a_L);
		ROS_DEBUG("theta: %f\n", theta); */
		
		x_I_CL = a_L*cos(theta);                                                                                 // x-coord. of intermediate point wrt left buoy 
		y_I_CL = a_L*sin(theta);                                                                                  // y-coord. of intermediate point wrt left buoy
		/* ROS_DEBUG("x_I_CL: %f\n", x_I_CL);
		ROS_DEBUG("y_I_CL: %f\n", y_I_CL); */
		
		x_E_CL = a_L*cos(theta);                                                                               // x-coord. of exit point wrt left buoy 
		y_E_CL = -a_L*sin(theta);                                                                               // y-coord. of exit point wrt left buoy
		/* ROS_DEBUG("x_E_CL: %f\n", x_E_CL);
		ROS_DEBUG("y_E_CL: %f\n", y_E_CL); */
		
		// calculate intermediate position wrt global
		s_M = (CR_y_NED-CL_y_NED)/(CR_x_NED-CL_x_NED);                                                                // slope of of line from CL to CR 
		alpha = atan2((CR_y_NED-CL_y_NED),(CR_x_NED-CL_x_NED));                                                                                           // angle of frame CL wrt global frame
		/* ROS_DEBUG("s_M: %f\n", s_M);
		ROS_DEBUG("alpha: %f\n", alpha); */
		
		I_x = cos(alpha)*x_I_CL - sin(alpha)*y_I_CL + CL_x_NED;                                            // x-coord. of intermediate point wrt global 
		I_y = sin(alpha)*x_I_CL + cos(alpha)*y_I_CL + CL_y_NED;                                           // y-coord. of intermediate point wrt global
		
		E_x = cos(alpha)*x_E_CL - sin(alpha)*y_E_CL + CL_x_NED;                                            // x-coord. of intermediate point wrt global 
		E_y = sin(alpha)*x_E_CL + cos(alpha)*y_E_CL + CL_y_NED;                                           // y-coord. of intermediate point wrt global
		
		// display calculated goal points to reach
		ROS_DEBUG("~~~Desired points~~~");
		ROS_DEBUG("I_x: %f", I_x);
		ROS_DEBUG("I_y: %f", I_y);
		ROS_DEBUG("M_x: %f", M_x);
		ROS_DEBUG("M_y: %f", M_y);
		ROS_DEBUG("E_x: %f", E_x);
		ROS_DEBUG("E_y: %f\n", E_y);
		ROS_DEBUG("________^^^^^^^^^^^  {PP}  ^^^^^^^^^^^___________________|\n");
		
		calculations_done = true;
	  }
	  // determine error in x and y (position)
	  e_x = x_goal - x_usv_NED;                                                // calculate error in x position
	  e_y = y_goal - y_usv_NED;                                                // calculate error in y position
	  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                           // calculate magnitude of positional error
	  
	  if ((!I_reached) && (e_xy < 4.0))
	  {
		I_reached = true;
		I_reached_loop = loop_count;
		ROS_DEBUG("Intermediate approach point has been reached. {PP}\n");
	  }
	  if ((!M_reached) && (I_reached) && (e_xy < (0.6*d_LM)) && (loop_count != I_reached_loop))
	  {
		M_reached = true;
	    M_reached_loop = loop_count;
		ROS_DEBUG("Midpoint has been reached. {PP}\n");
	  }
	  if ((!E_reached) && (M_reached) && (e_xy < 4.0) && (loop_count != M_reached_loop) && (loop_count != I_reached_loop))
	  {
		E_reached = true;
		ROS_DEBUG("Exit point has been reached. {PP}\n");
	  }
	
	  // publish the desired pose
	  if ((!M_reached) && (!I_reached))
	  {
		// until intermediate approach point is reached, it is fed
		x_goal = I_x;
	    y_goal = I_y;
	  }
	  if ((I_reached) && (!M_reached))
	  {
		// once intermediate point is reached, the midpoint is fed
	    x_goal = M_x;
	    y_goal = M_y;
	  }
	  if ((M_reached) && (!E_reached))
	  {
	    // once midpoint is reached, the exit point is fed
	    x_goal = E_x;
	    y_goal = E_y;
	  }
	  if (E_reached)
	  {
	    // once exit point is reached, the current position is fed
	    x_goal = x_usv_NED;
	    y_goal = y_usv_NED;
		ON_OFF = 0.0; // controller is OFF once exit point is reached
	  }
	  else
	  {
		ON_OFF = 1.0; // controller is ON until exit point is reached
	  }
	  
	
   	  // fill in message with values that are unnecesary
	  nav_odom.header.seq +=1;                                          // sequence number
	  nav_odom.header.stamp = current_time;                    // sets stamp to current time
	  nav_odom.header.frame_id = "odom";                         // header frame
	  nav_odom.child_frame_id = "base_link";                      // child frame
	  nav_odom.pose.pose.position.x = x_goal;                    // sets x-location
	  nav_odom.pose.pose.position.y = y_goal;                    // sets y-location
	  nav_odom.pose.pose.position.z = ON_OFF; // use this to shut off low-level controller by sending a 0.0
	  nav_odom.pose.pose.orientation.x = 0.0;
	  nav_odom.pose.pose.orientation.y = 0.0;
	  nav_odom.pose.pose.orientation.z = 0.0;
	  nav_odom.pose.pose.orientation.w = 0.0;
	  nav_odom.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	  nav_odom.twist.twist.linear.x = 0.0; //sets velocity values all to zero
	  nav_odom.twist.twist.linear.y = 0.0;
	  nav_odom.twist.twist.linear.z = 0.0;
	  nav_odom.twist.twist.angular.x = 0.0;
	  nav_odom.twist.twist.angular.y = 0.0;
	  nav_odom.twist.twist.angular.z = 0.0;
	  nav_odom.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	  mpp_goal_pub.publish(nav_odom); // publishes desired pose to mpp_goal node
	} // if (loop_count>10)
	
	last_time = current_time; 
	ros::spinOnce();
	loop_rate.sleep();
	loop_count = loop_count + 1;
  } // while(ros::ok() && !goal)

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}