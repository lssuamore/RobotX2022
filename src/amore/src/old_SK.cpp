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
// ---------------------------------------------------------------------------------------------------------------------
int k = 0;                          // used to stop the differential and integration term from being  processed first time through the loop
float duration = 1000;      // amount of time to finish the path 
float dt = 0.25;                   // [s] used for differential term

float x_usv;                     // this variable is updated as the WAM-V x position in real time by transfering lat and long
float y_usv;                     // this variable is updated as the WAM-V y position in real time by transfering lat and long
float x_usv_NED;           // this variable is updated as the WAM-V x position in NED
float y_usv_NED;           // this variable is updated as the WAM-V y position in NED

// q0 - q3 are current orientation in quaternion form
float q1;            
float q2;
float q3;
float q0;
float psi;                          // this variable is updated as the WAM-V z-orientation in real time through the compass
float psi_NED;                // this variable is updated as the WAM-V z-orientation in NED

// close range goal on red buoy
float x_goal = 38.12;
float y_goal = 15.46;

float psi_desired = 0.56;       // [radians] desired heading to hold at goal location, refer to line 230

float e_x;                        // error between the desired x-pos. and the actual x-pos.
float e_y;                        // error between the desired y-pos. and the actual y-pos.
float e_xy;                      // magnitude of position error
float e_psi;                     // error between the desired z-orientation and the actual z-orientation

// initialize accumulated total errors for integral term
float e_x_total = 0;
float e_y_total = 0;
float e_psi_total = 0;
// initialize previous errors for calculating differential term
float e_x_prev = 0;
float e_y_prev = 0;
float e_xy_prev = 0;
float e_psi_prev = 0;

float T_x;              // thrust to set in x-direction in earth-fixed frame
float T_y;              // thrust to set in y-direction in earth-fixed frame
float T_x_bf;         // thrust in x-direction in body-fixed frame
float T_y_bf;         // thrust in y-direction in body-fixed frame
float M_z;             // desired moment around the z-axis

// matrix to hold outputs of controlller
float tau[3][1];

float T_p;         // used to set the port (left) thruster output
float T_s;         // used to set the starboard (right) thruster output
float A_p;         // used to set the port (left) thruster angle
float A_s;         // used to set the starboard (right) thruster angle

// Used for correction of saturated thrust values 
float T_p_C;         // Corrected port (left) thrust output
float T_s_C;         // Corrected starboard (right) thrust output

// Proportional gains
float Kp_x = 0.05;               // Position proportional gain in x-direction
float Kp_y = 0.05;               // Position proportional gain in y-direction
float Kp_psi = 0.3;              // z-orientation proportional gain
// Differential gains
float Kd_x = 0.1;               // Position differential gain in x-direction
float Kd_y = 0.1;               // Position differential gain in y-direction
float Kd_psi = 5.0;              // z-orientation differential gain
// Integration gains
float Ki_x = 0.0;                // Position integration gain in x-direction
float Ki_y = 0.0;                // Position integration gain in x-direction
float Ki_psi = 0.0; //0.06;               // z-orientation integration gain

/* 
float B = 2.0;                  // [m] beam of USV, 8 ft or 2.44 m
float L = 4.6;                  // [m] length of USV, 16 ft or 4.88 m

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
float F_xp; // force in the x-direction on the port side
float F_yp; // force in the y-direction on the port side
float F_xs; // force in the x-direction on the starboard side
float F_ys; // force in the y-direction on the starboard side

// matrix to hold outputs of control allocation
float f[4][1];

void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	x_usv = odom->pose.pose.position.x;
	y_usv = odom->pose.pose.position.y;
	/* printf("the x is: %f\n", x_usv); //displays x coord.
	printf("the y is: %f\n", y_usv); //displays y coord. */
	q1 = odom->pose.pose.orientation.x;
	q2 = odom->pose.pose.orientation.y;
	q3 = odom->pose.pose.orientation.z;
	q0 = odom->pose.pose.orientation.w;
	
	// convert current heading from quaternion into radians
	psi = atan2((2.0*(q1*q2 + q3*q0)) , (pow(q1,2.0) + pow(q0,2.0) - pow(q2,2.0) - pow(q3,2.0))); // orientation off x-axis, old 2nd term: 1 - 2*(pow(q2,2.0) + pow(q3,2.0))
	
	// convert from ENU to NED
    x_usv_NED = y_usv;
	y_usv_NED = x_usv;
	psi_NED = PI/2.0 - psi;	
}

// THIS FUNCTION UPDATES THE GAINS -------
// ACCEPTS NOTHING (VOID) ------------------------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void parameters_function()
{
	// GET ALL PARAMETERS FROM LAUNCH FILE
	ros::param::get("/Kp_xy_G", Kp_x);
	Kp_y = Kp_x; //ros::param::get("/Kp_y_G", Kp_y);
	ros::param::get("/Kp_psi_G", Kp_psi);
	ros::param::get("/Kd_xy_G", Kd_x);
	Kd_y = Kd_x; //ros::param::get("/Kd_y_G", Kd_y);
	ros::param::get("/Kd_psi_G", Kd_psi);
	ros::param::get("/Ki_xy_G", Ki_x);
	Ki_y = Ki_x; //::param::get("/Ki_y_G", Ki_y);
	ros::param::get("/Ki_psi_G", Ki_psi);
	ROS_INFO("Kp_xy is: %f", Kp_x);
	//ROS_INFO("Kp_y is: %f", Kp_y);
	ROS_INFO("Kp_psi is: %f\n", Kp_psi);
	ROS_INFO("Kd_xy is: %f", Kd_x);
	//ROS_INFO("Kd_y is: %f", Kd_y);
	ROS_INFO("Kd_psi is: %f\n", Kd_psi);
	ROS_INFO("Ki_xy is: %f", Ki_x);
	//ROS_INFO("Ki_y is: %f", Ki_y);
	ROS_INFO("Ki_psi is: %f\n", Ki_psi);
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "Station_keeping");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  std_msgs::Float32 LT, RT, LA, RA;            // LT is left thrust, RT is right thrust, LA is left thruster angle, RA is right thruster angle
  
  // set up NodeHandles
  ros::NodeHandle nh1;  // this node handle is used for subscribing to geonav_odom which provides position in x, y
  ros::NodeHandle nh5;  // this node handle is used for publishing thrust speed to starboard side 
  ros::NodeHandle nh6;  // this node handle is used for publishing thrust speed to port side
  ros::NodeHandle nh7;  // this node handle is used for publishing thrust angle to starboard side 
  ros::NodeHandle nh8;  // this node handle is used for publishing thrust angle to port side
  
  // start publishers and subscribers
  ros::Subscriber nav_sub = nh1.subscribe("geonav_odom", 1, pose_update);                                                      // subscribes to geonav_odom
  ros::Publisher stbd_T_pub = nh5.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);      // publishes float value between -1.0 and 1.0, speed to right thruster
  ros::Publisher port_T_pub = nh6.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);         // publishes float value between -1.0 and 1.0, speed to left thruster
  ros::Publisher stbd_A_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 10);     // publishes float value between -PI to PI, angle to right thruster
  ros::Publisher port_A_pub = nh8.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 10);        // publishes float value between -PI to PI, angle to left thruster
  
  ros::Time current_time, last_time;  // creates time variables
  current_time = ros::Time::now();   // sets current time to the time it is now
  last_time = ros::Time::now();        // sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(4);
  ros::spinOnce();                             // initialize position

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  // Update gain values
	  parameters_function();
	  
	  // Integral Reset
	  if ((float)abs(e_x) < 0.1) 
	  {
		  e_x_total=0.0;
	  }
	  if ((float)abs(e_y) < 0.1)
	  {
		  e_y_total=0.0;
	  }
	  if ((float)abs(e_psi) < 0.5)
	  {
		  e_psi_total=0.0;
	  }
	  
	  // determine error in x and y (position)
	  e_x = x_goal - x_usv_NED;
	  e_y = y_goal - y_usv_NED;
	  
	  /* // if errors are small enough, do not try to correct for them
      if ((float)abs(e_x) < 0.1)
	  {
		  e_x=0.0;
	  }
	  if ((float)abs(e_y) < 0.1)
	  {
		  e_y=0.0;
	  } */
	  
	  // calculate magnitude of positional error
	  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                           
	  
	  /* // if within error only have P-controller
	  if (e_xy < 2.0) 
	  {
		  Kd_x = 0.0;
		  Kd_y = 0.0;
          Ki_x = 0.0;
		  Ki_y = 0.0;
	  } */
	  
	  // position control law
	  if (k<6) // don't include differential term or integration term first time through
	  {
		  T_x = Kp_x*e_x;
		  T_y = Kp_y*e_y;
	  }
	  else
	  {
		  // trapezoidal integration of errors for integral term
		  e_x_total = e_x_total + ((e_x_prev + e_x)/2.0)*dt; 
		  e_y_total = e_y_total + ((e_y_prev + e_y)/2.0)*dt;
		  /* ROS_DEBUG("E_X_Total = %f\n", e_x_total);
	      ROS_DEBUG("E_Y_Total = %f\n", e_y_total); */
		  
		  T_x = Kp_x*e_x + Kd_x*((e_x - e_x_prev)/dt) + Ki_x*e_x_total;
		  T_y = Kp_y*e_y + Kd_y*((e_y - e_y_prev)/dt) + Ki_y*e_y_total; 
	  }
	  
	  // Give desired heading at goal once within 5.0 meters
	  if (e_xy < 10.0)
	  {
		  psi_desired = 3.0;       // [radians] desired heading to hold at goal location
	  }
	  else
	  {
		  psi_desired = atan2(e_y,e_x);       // [radians] desired heading to travel to goal location
	  }
	  
	  // adjust current heading back within -PI and PI
	  if (psi_NED < -PI)
	  {
		  psi_NED = psi_NED + 2.0*PI;
	  }
	  if (psi_NED > PI)
	  {
		  psi_NED = psi_NED - 2.0*PI;
	  } 
	  
	  e_psi = psi_desired - psi_NED;
	  
	  /* // if error is small enough, do not try to correct
	  if ((float)abs(e_psi) < 0.1)
	  {
		  e_psi=0.0;
	  } */
	  
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
	  
	  /* // if within error only have P-controller
	  if ((float)abs(e_psi) < 0.1)
	  {
		  Kd_psi = 0.0;
		  Ki_psi = 0.0;
	  } */
	  
	  // orientation control law
	  if (k<6) // don't include differential term or integration term first time through
	  {
		  M_z = Kp_psi*e_psi;
	  }
	  else
	  {
		  // trapezoidal integration of error for integral term
		  e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt; 
		  //ROS_DEBUG("E_PSI_Total = %f\n", e_psi_total);
		  
		  M_z = Kp_psi*e_psi + Kd_psi*((e_psi - e_psi_prev)/dt) + Ki_psi*e_psi_total;
	  }
	  
	  // UPDATES STATUSES TO USER ///////////////////////////////////////////////
	  ROS_DEBUG("x_goal: %f", x_goal);
	  ROS_DEBUG("x_usv: %f", x_usv_NED);
	  ROS_DEBUG("e_x is: %f\n", e_x);                             // x posn. error
	  ROS_DEBUG("y_goal: %f", y_goal);
      ROS_DEBUG("y_usv: %f", y_usv_NED);
      ROS_DEBUG("e_y is: %f\n", e_y);                             // y posn. error
	  ROS_DEBUG("e_xy: %f\n", e_xy);                             // magnitude of posn. error
	  ROS_DEBUG("desired psi is: %f", psi_desired);
	  ROS_DEBUG("psi_NED is: %f", psi_NED);
	  ROS_DEBUG("e_psi is: %f\n", e_psi);                        // heading error
	  
	  ROS_DEBUG("Control Efforts");
	  ROS_DEBUG("T_x_G: %f", T_x);
	  ROS_DEBUG("T_y_G: %f", T_y);
	  ROS_DEBUG("M_z: %f\n", M_z);
	  
	  /* // if within a meter only control heading
	  if (e_xy < 1.0) 
	  {
		  T_x = 0.0;
		  T_y = 0.0;
	  } */
	  
	  /* // only control heading if heading is off by more than 45 degree
	  if ((float)abs(e_psi) > 0.785)
	  {
		  T_x = 0.0;
		  T_y = 0.0;
	  } */
	  
	  /* // if error is small enough, do not try to correct
	  if ((float)abs(e_psi) < 0.1)
	  {
		  M_z = 0.0;
	  } */
	  
	  /* tau [1][1] = T_x;
	  tau [2][1] = T_y;
	  tau [3][1] = M_z; */
	  
	  // Convert to USV body-fixed frame from global frame
	  T_x_bf = T_x*cos(psi_NED) + T_y*sin(psi_NED);
	  T_y_bf = T_y*cos(psi_NED) - T_x*sin(psi_NED);
	  /* T_x = (float)abs(T_x_bf);
	  T_y = (float)abs(T_y_bf); */
	  
	  T_x = T_x_bf;
	  T_y = T_y_bf;
	   
	  // calculate the control allocation outputs
	  // f = Transform_pseudoinverse * tau;
      f[0][0] = Transform_pseudoinverse[0][0]*T_x + Transform_pseudoinverse[0][1]*T_y + Transform_pseudoinverse[0][2]*M_z;
	  f[1][0] = Transform_pseudoinverse[1][0]*T_x + Transform_pseudoinverse[1][1]*T_y + Transform_pseudoinverse[1][2]*M_z;
	  f[2][0] = Transform_pseudoinverse[2][0]*T_x + Transform_pseudoinverse[2][1]*T_y + Transform_pseudoinverse[2][2]*M_z;
	  f[3][0] = Transform_pseudoinverse[3][0]*T_x + Transform_pseudoinverse[3][1]*T_y + Transform_pseudoinverse[3][2]*M_z;
	  
	  // Extract values from f
	  F_xp = f[0][0]; // force in x-direction on the port side
	  F_yp = f[1][0]; // force in y-direction on the port side
	  F_xs = f[2][0]; // force in x-direction on the starboard side
	  F_ys = f[3][0]; // force in y-direction on the starboard side
	  
	  T_p = sqrt(pow(F_xp,2.0)+pow(F_yp,2.0));                 // calculate magnitude of port thrust
	  T_s = sqrt(pow(F_xs,2.0)+pow(F_ys,2.0));                 // calculate magnitude of starboard thrust
	  A_p = -atan2(F_yp,F_xp);                                              // calculate angle of port thrust
	  A_s = -atan2(F_ys,F_xs);                                              // calculate angle of starboard thrust
	  
	  ROS_DEBUG("Before------------------------");
	  ROS_DEBUG("Port Thrust: %f", T_p);
	  ROS_DEBUG("Starboard Thrust: %f", T_s);
	  ROS_DEBUG("Port Angle: %f", A_p);
	  ROS_DEBUG("Starboard Angle: %f", A_s);
	  
	  // Angles to thrusters can only be set between -PI/2 and PI/2
	  if (A_p > PI/2.0)
	  {
		  A_p = A_p - PI;
		  T_p = -1.0 * T_p;
	  }
	  else if (A_p < -PI/2.0)
	  {
		  A_p = A_p + PI;
		  T_p = -1.0 * T_p;
	  }
	  if (A_s > PI/2.0)
	  {
		  A_s = A_s - PI;
		  T_s = -1.0 * T_s;
	  }
	  else if (A_s < -PI/2.0)
	  {
		  A_s = A_s + PI;
		  T_s = -1.0 * T_s;
	  }
	  
	  if (A_p > 1.4)
	  {
		  A_p = PI/2.0;
	  }
	  else if (A_p < -1.4)
	  {
		  A_p = -PI/2.0;
	  }
	  if (A_s > 1.4)
	  {
		  A_s = PI/2.0;
	  }
	  else if (A_s < -1.4)
	  {
		  A_s = -PI/2.0;
	  }
	  
	  /* //-----------------------------------------------FAU control allocation conversion---------------------------------------------------
	  // check if in azimuth deadzone (25-155 degrees)
	  if ((A_p < 2.7) && (A_p > 0.44))
	  {
		  if ((float)abs(A_p - 2.7) < (float)abs(A_p - 0.44))
		  {
			  A_p = 2.7;
		  }
		  else 
		  {
			  A_p = 0.44;
		  }
	  }
	  else if ((A_p > -2.7) && (A_p < -0.44))
	  {
		  if ((float)abs(A_p + 2.7) < (float)abs(A_p + 0.44))
		  {
			  A_p = -2.7;
		  }
		  else 
		  {
			  A_p = -0.44;
		  }
	  }
	  if ((A_s < 2.7) && (A_s > 0.44))
	  {
		  if ((float)abs(A_s - 2.7) < (float)abs(A_s - 0.44))
		  {
			  A_s = 2.7;
		  }
		  else 
		  {
			  A_s = 0.44;
		  }
	  }
	  else if ((A_s > -2.7) && (A_s < -0.44))
	  {
		  if ((float)abs(A_s + 2.7) < (float)abs(A_s + 0.44))
		  {
			  A_s = -2.7;
		  }
		  else 
		  {
			  A_s = -0.44;
		  }
	  }
	  
	  // angular restriction
      if ((A_p >= 2.7) && (A_p <= PI))
	  {
		  T_p *= -1;
		  A_p -= PI;
	  }
	  else if ((A_p >= -PI) && (A_p <= -2.7))
	  {
		  T_p *= -1;
		  A_p += PI;
	  }
	  if ((A_s >= 2.7) && (A_s <= PI))
	  {
		  T_s *= -1;
		  A_s -= PI;
	  }
	  else if ((A_s >= -PI) && (A_s <= -2.7))
	  {
		  T_s *= -1;
		  A_s += PI;
	  }
	  //-----------------------------------------------FAU control allocation conversion--------------------------------------------------- */
	  
	  ROS_DEBUG("After------------------------");
	  ROS_DEBUG("Port Thrust: %f", T_p);
	  ROS_DEBUG("Starboard Thrust: %f", T_s);
	  ROS_DEBUG("Port Angle: %f", A_p);
	  ROS_DEBUG("Starboard Angle: %f\n", A_s);
	  
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
		  ROS_DEBUG("Port Thrust after correction: %f\n", T_p);
	      ROS_DEBUG("Starboard Thrust after correction: %f\n", T_s);
	  }
	  
	  /* // check for reverse force not being equivalent to forward force
	  if ((T_p < 0.0) && (T_s > 0.0))
	  {
		  T_s = T_s/1.5;
	  }
	  else if ((T_p > 0.0) && (T_s < 0.0))
	  {
		  T_p = T_p/1.5;
	  } */
	  
	  /* if ((e_xy - e_xy_prev) > 0.0) // positional error is growing
	  {
		  T_p = -T_p;
		  T_s = -T_s;
	  } */
	  
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
	  ROS_DEBUG("M_z_I: %f\n", Ki_psi*e_psi_total); */
	  
	  /* ROS_DEBUG("BEFORE SWAP TO BODY-FIXED FRAME");
	  ROS_DEBUG("T_x_G: %f", T_x);
	  ROS_DEBUG("T_y_G: %f", T_y);
	  ROS_DEBUG("M_z: %f", M_z);
	  ROS_DEBUG("AFTER SWAP");
	  ROS_DEBUG("T_x_bf: %f", T_x_bf);
	  ROS_DEBUG("T_y_bf: %f\n", T_y_bf); */
	  
	  /* // Print f values
	  ROS_DEBUG("F_xp: %f", F_xp);
	  ROS_DEBUG("F_yp: %f", F_yp);
	  ROS_DEBUG("F_xs: %f", F_xs);
	  ROS_DEBUG("F_ys: %f\n", F_ys); */
	  
	  // only print to thrusters if far enough into loop to have correct calculations
	  // for some reason the first 4 times through loop the current pose variables do not update
	  if (k>4)                                                 
	  { 
          LA.data = A_p;
		  port_A_pub.publish(LA);
          LT.data = T_p;
		  port_T_pub.publish(LT);
		  RA.data = A_s;
		  stbd_A_pub.publish(RA);
	      RT.data = T_s;
	      stbd_T_pub.publish(RT);
	  }
	  
	  // update previous errors
	  e_x_prev = e_x;
	  e_y_prev = e_y;
	  e_xy_prev = e_xy;
	  e_psi_prev = e_psi;
	  
	  k = k+1;
	  ros::spinOnce();
	  loop_rate.sleep();
  }
  LT.data = 0.0;
  port_T_pub.publish(LT);
  RT.data = 0.0;	
  stbd_T_pub.publish(RT);

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}


