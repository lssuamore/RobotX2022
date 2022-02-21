//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
//#include "gps_common/GPSFix.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <iostream>
#include "stdio.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

// DEFINING GLOBAL VARIABLES
// ---------------------------------------------------------------------------------------------------------------------
int k =1;                           // used to stop the differential and integration term from being  processed first time through the loop
float duration = 1000;      // amount of time to finish the path 
float dt = 0.1;                   // [s] used for differential term
float B = 2.44;                  // [m] 8 ft or 2.44 m

float x_usv;                     // this variable is updated as the WAM-V x position in real time by transfering lat and long 
float y_usv;                     // this variable is updated as the WAM-V y position in real time by transfering lat and long 
float psi;                          // this variable is updated as the WAM-V z-orientation in real time through the compass

float x_goal = 150.674282;                    // this variable is updated as the WAM-V x goal position through the PID_pub
float y_goal = -34.422402;                    // this variable is updated as the WAM-V y goal position through the PID_pub

float psiD;                       // this is calculated as the desired z-orientation

float e_x;                        // this is calculated as the error between the desired x-pos. and the actual x-pos.
float e_y;                        // this is calculated as the error between the desired y-pos. and the actual y-pos.
float e_xy;                      // this is calculated as the error between the desired xy-pos. and the actual xy-pos.
float e_psi;                     // this is calculated as the error between the desired z-orientation and the actual z-orientation

float e_xy_total = 0;      // used for I term
float e_psi_total = 0;     // used for I term
float e_xy_prev = 0;     // holds previous xy error
float e_psi_prev = 0;    // holds previous heading error

float T_x;              // this is continuously calculated as the thrust to set in x-direction
float M_z;             // this is calculated as the desired moment around the z-axis

float T_port;          // this is used to set the port (left) thruster
float T_stbd;         // this is used to set the starboard (right) thruster

// Proportional gains
float Kp_x = 0.01005;              // Position proportional gain in x-direction
float Kp_y = 0.00105;              // Position proportional gain in y-direction
float Kp_psi = 0.0000008;       // z-orientation proportional gain
// Differential gains
float Kd_x = 0.05;                        // Position differential gain in x-direction
float Kd_y = 0;                        // Position differential gain in y-direction
float Kd_psi = 0;                     // z-orientation differential gain
// Integration gains
float Ki_x = 0.000005;                        // Position integration gain in x-direction
float Ki_y = 0;                        // Position integration gain in y-direction
float Ki_psi = 0;                     // z-orientation integration gain

void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) 
{
	x_usv = gps_msg->longitude;
	y_usv = gps_msg->latitude;
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "PID_controller");
  //ros::NodeHandle nh1;  // this node handle is used for subscribing to user desired position
  ros::NodeHandle nh2;  // this node handle is used for subscribing to gps
  ros::NodeHandle nh4;  // this node handle is used for subscribing to compass
  ros::NodeHandle nh5;  // this node handle is used for publishing thrust information to starboard side 
  ros::NodeHandle nh6;  // this node handle is used for publishing thrust information to port side
  
  
  //declare subsrcriber "squarenum" is the name of the node
  //1 is how many to save in the buffer
  //uint_function is the function called when a value is recieved
  //ros::Subscriber square = nh1.subscribe("squarenum", 1, uint_function);
  
  ros::Subscriber gps_sub = nh2.subscribe("/wamv/sensors/gps/gps/fix", 1, gpsCallBack);    // subscribe to gps to attain Latitude and longitude
  ros::Publisher stbd_T_pub = nh5.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1000);
  ros::Publisher port_T_pub = nh6.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1000);
  
  //ros::Subscriber topic_sub1 = nh3.subscribe("/wamv/sensors/gps/gps/fix_velocity", 1, VelCallBack);    // subscribe to gps to attain Latitude and Longitude
  
  //ros::Subscriber topic_sub = nh4.subscribe("/wamv/sensors/gps/gps/fix", 1, gpsCallBack);    // subscribe to gps to attain Latitude and longitude

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);
  ros::spinOnce(); // initialize position

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  std_msgs::Float32 LT, RT;
	  
	  // determine error in x and y (position)
	  e_x = x_goal - x_usv;                                                // calculate error in x position
	  e_y = y_goal - x_usv;                                                // calculate error in x position
	  e_xy = sqrt(pow(e_x,2)+pow(e_y,2));                      // calculate magnitude of desired velocity
	  
	  // position control law
	  if (k==1) // don't include differential term or integration term
	  {
		  T_x = Kp_x*e_xy;
	  }
	  else
	  {
		  // trapezoidal integration of errors for integral term
		  e_xy_total = e_xy_total + ((e_xy_prev + e_xy)/2)*dt;
		  T_x = Kp_x*e_xy+ Kd_x*((e_xy - e_xy_prev)/dt) + Ki_x*e_xy_total;
	  }
	  
	  // determine error in heading (psi)
	  psiD = atan2(y_goal-y_usv, x_goal-x_usv);
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
	  e_xy_prev = e_xy; 
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
