//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "time.h"

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/Float64.h"                             // maybe 64
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

// DEFINING GLOBAL VARIABLES
// ---------------------------------------------------------------------------------------------------------------------
int k =1;                           // used to stop the differential and integration term from being  processed first time through the loop
float duration = 100;      // amount of time to finish the path 
float dt = 0.1;                   // [s] used for differential term
float B;                            // [] 8 ft or 2.44 m

float x_usv;                     // this variable is updated as the WAM-V x position in real time by transfering lat and long 
float y_usv;                     // this variable is updated as the WAM-V y position in real time by transfering lat and long 
// q0 - q3 are current orientation in quaternion form
float q1;            
float q2;
float q3;
float q0;
float psi;                          // this variable is updated as the WAM-V z-orientation in real time through the compass

float x_goal = 0;                    // this variable is updated as the WAM-V x goal
float y_goal = 20;                  // this variable is updated as the WAM-V y goal

float x_velD;                   // this variable is calculated as the desired velocity in x-direction
float y_velD;                   // this variable is calculated as the desired velocity in y-direction
float velD;                       // this is calculated as the magnitude of the desired velocity
float psiD;                       // this is calculated as the desired z-orientation

float vel_USV;               // this is updated in real time through the IMU

float e_xVel;                   // this is calculated as the error between the desired velocity and the actual velocity
float e_psi;                     // this is calculated as the error between the desired z-orientation and the actual z-orientation

float e_xVel_total = 0;   // used for I term
float e_psi_total = 0;     // used for I term
float e_xVel_prev = 0;  // holds previous xVel error
float e_psi_prev = 0;    // holds previous heading error

float T_x;              // this is continuously calculated as the thrust to set in x-direction
float M_z;             // this is calculated as the desired moment around the z-axis

float T_port;          // this is used to set the port (left) thruster
float T_stbd;         // this is used to set the starboard (right) thruster

// Proportional gains
float Kp_x = 0.00105;              // Position proportional gain in x-direction
float Kp_y = 0.00105;             // Position proportional gain in y-direction
float Kp_psi = 0.0000148;       // z-orientation proportional gain
// Differential gains
float Kd_x = 0;                       // Position differential gain in x-direction
float Kd_y = 0;                       // Position differential gain in y-direction
float Kd_psi = 0;                    // z-orientation differential gain
// Integration gains
float Ki_x = 0;                        // Position integration gain in x-direction
float Ki_y = 0;                        // Position integration gain in y-direction
float Ki_psi = 0;                     // z-orientation integration gain

void status_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	x_usv = odom->pose.pose.position.x;
	y_usv = odom->pose.pose.position.y;
	q1 = odom->pose.pose.orientation.x;
	q2 = odom->pose.pose.orientation.y;
	q3 = odom->pose.pose.orientation.z;
	q0 = odom->pose.pose.orientation.w;
	vel_USV = odom->twist.twist.linear.x;
}

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "PID_HS_controller");
  
  std_msgs::Float32 LT, RT;
  
  // set up NodeHandles
  ros::NodeHandle nh3;  // this node handle is used for subscribing to geonav_odom which provides position in x, y
  ros::NodeHandle nh5;  // this node handle is used for publishing thrust information to starboard side 
  ros::NodeHandle nh6;  // this node handle is used for publishing thrust information to port side
  
  // start publishers and subscribers
  ros::Subscriber nav_sub = nh3.subscribe("geonav_odom", 1, status_update);
  ros::Publisher stbd_T_pub = nh5.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1000);
  ros::Publisher port_T_pub = nh6.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1000);

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);
  ros::spinOnce(); // initialize position

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
//	  std_msgs::Float32 LT, RT;
	  
	  psi = atan2((2.0*(q3*q0 + q1*q2)) , (pow(q0,2.0) + pow(q1,2.0) + pow(q2,2.0) + pow(q3,2.0)));
	  printf("psi is: %f\n", psi);  
	  printf("velocity is: %f\n", vel_USV);    
	  // determine error in xVel
	  x_velD = (x_goal - x_usv) / duration;                         // calculate desired velocity in x-direction
	  y_velD = (y_goal - y_usv) / duration;                         // calculate desired velocity in y-direction
	  velD = sqrt(pow(x_velD,2)+pow(y_velD,2));            // calculate magnitude of desired velocity
	  e_xVel = velD - vel_USV;                                         // calculate error in x velocity
	  
	  // position control law
	  if (k==1) // don't include differential term or integration term
	  {
		  T_x = Kp_x*e_xVel;
	  }
	  else
	  {
		  // trapezoidal integration of errors for integral term
		  e_xVel_total = e_xVel_total + ((e_xVel_prev + e_xVel)/2)*dt;
		  T_x = Kp_x*e_xVel + Kd_x*((e_xVel - e_xVel_prev)/dt) + Ki_x*e_xVel_total;
	  }
	  
	  // determine error in heading (psi)
	  psiD = atan2(y_velD, x_velD);
	  e_psi = psiD - psi;
	  
	  // orientation control law
	  if (k==1)
	  {
		  // don't include differential term or integration term
		  M_z = Kp_psi*e_psi;
	  }
	  else
	  {
		  // trapezoidal integration of errors for integral term
		  e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2)*dt;
		  M_z = Kp_psi*e_psi + Kd_psi*((e_psi - e_psi_prev)/dt) + Ki_psi*e_psi_total;
	  }
	  
	  // update previous errors
	  e_xVel_prev = e_xVel; 
	  e_psi_prev = e_psi;
	  
	  T_port = T_x/2 + M_z/B;
	  T_stbd = T_x/2 - M_z/B;
	  
	  // ensure thrusters are within bounds
	  if (T_port < -1.0)
	  {
		  T_port = -1.0;
	  }
	  if (T_port > 1.0)
	  {
		  T_port = 1.0;
	  }
	  if (T_stbd < -1.0)
	  {
		  T_stbd = -1.0;
	  }
	  if (T_stbd > 1.0)
	  {
		  T_stbd = 1.0;
	  }
	  
	  LT.data = T_port;
	  port_T_pub.publish(LT);
	  RT.data = T_stbd;
	  stbd_T_pub.publish(RT);
	  
	  ros::spinOnce();
	  loop_rate.sleep();
	  k = k+1;
  }

  return 0;
}


