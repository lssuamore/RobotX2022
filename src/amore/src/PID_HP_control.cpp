// Includes all of the ROS libraries needed
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

// include necessary message libraries
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

// DEFINING GLOBAL VARIABLES
#define PI 3.14159265
// ---------------------------------------------------------------------------------------------------------------------
bool goal = false;            // goal = false means goal has not been reached, goal = true means reached, starts out as false
int k =1;                           // used to stop the differential and integration term from being  processed first time through the loop
float duration = 1000;      // amount of time to finish the path 
float dt = 0.1;                   // [s] used for differential term
float B = 2.44;                  // [m] 8 ft or 2.44 m

float x_usv;                     // this variable is updated as the WAM-V x position in real time by transfering lat and long 
float y_usv;                     // this variable is updated as the WAM-V y position in real time by transfering lat and long

// q0 - q3 are current orientation in quaternion form
float q1;            
float q2;
float q3;
float q0;
float psi;                          // this variable is updated as the WAM-V z-orientation in real time through the compass

float x_goal = 30;                    // this variable is updated as the WAM-V x goal position through the PID_pub
float y_goal = 30;                  // this variable is updated as the WAM-V y goal position through the PID_pub

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
float Kp_xy = 0.05;              // Position proportional gain in x-direction
float Kp_psi = 0.5;                // z-orientation proportional gain
// Differential gains
float Kd_xy = 0.0;                      // Position differential gain in x-direction
float Kd_psi = 0.0;                     // z-orientation differential gain
// Integration gains
float Ki_xy = 0.001;                       // Position integration gain in x-direction
float Ki_psi = 0.01;                      // z-orientation integration gain

/* // used for using geonav transform package to get lat and long to x and y
double latitude;
double longitude;
double th;

float orientationx;
float orientationy;
float orientationz;
float orientationw;

float velx;
float vely;
float velz;

void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) 
{
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	th = gps_msg->altitude; //sets altitude rom gps
}

 void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg) 
 {
	orientationx = imu_msg->orientation.x; //sets orientationx
	orientationy = imu_msg->orientation.y; //sets orientationy
	orientationz = imu_msg->orientation.z; //sets orientationz
	orientationw = imu_msg->orientation.w; //sets orientationw
}

void gps_velCallBack(const geometry_msgs::Vector3Stamped::ConstPtr& vel_msg) 
{
	velx = vel_msg->vector.x; //sets orientationx
	vely = vel_msg->vector.y; //sets orientationy
	velz = vel_msg->vector.z; //sets orientationz
}

// THIS FUNCTION GETS THE POSITION OF THE USV IN LOCAL FRAME -------
// ACCEPTS NOTHING (VOID) ------------------------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void gps_subscriber()
{
	ros::NodeHandle nh4;  // this node handle is used for publishing position and velocity to nav_odom
	ros::Publisher position_pub = nh4.advertise<nav_msgs::Odometry>("nav_odom", 10);                                    // publishes geographic position and velocity
	
	ros::Time current_time;  // creates time variables
	current_time = ros::Time::now(); //time
	
	nav_msgs::Odometry nav_odom; //new message
	nav_odom.header.seq +=1; //sequence number
	nav_odom.header.stamp = current_time; //sets stamp to current time
	nav_odom.header.frame_id = "odom"; //header frame
	nav_odom.child_frame_id = "base_link"; //child frame
	nav_odom.pose.pose.position.x = longitude; //sets long
	nav_odom.pose.pose.position.y = latitude; //sets lat
	nav_odom.pose.pose.position.z = th; //sets altitude
//	nav_odom.pose.pose.orientation = odom_quat; //sets orientation
	nav_odom.pose.pose.orientation.x = orientationx;
	nav_odom.pose.pose.orientation.y = orientationy;
	nav_odom.pose.pose.orientation.z = orientationz;
	nav_odom.pose.pose.orientation.w = orientationw;
	nav_odom.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	nav_odom.twist.twist.linear.x = velx; //sets velocity values all to zero
	nav_odom.twist.twist.linear.y = vely;
	nav_odom.twist.twist.linear.z = velz;
	nav_odom.twist.twist.angular.x = 0.0;
	nav_odom.twist.twist.angular.y = 0.0;
	nav_odom.twist.twist.angular.z = 0.0;
	nav_odom.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	position_pub.publish(nav_odom); //publishes above to position_pub
	//sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(100);
	ros::spinOnce();
	loop_rate.sleep();
} */

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
}

// THIS FUNCTION UPDATES THE GAINS -------
// ACCEPTS NOTHING (VOID) ------------------------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void parameters_function()
{
	// GET ALL PARAMETERS FROM LAUNCH FILE
	ros::param::get("/Kp_xy_G", Kp_xy);
	ros::param::get("/Kp_psi_G", Kp_psi);
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "PID_HP_controller");
  
  std_msgs::Float32 LT, RT;
  
  /* // set up NodeHandles
  ros::NodeHandle nh0;  // this node handle is used for subscribing to gps/gps/fix
  ros::NodeHandle nh1;  // this node handle is used for subscribing to gps/gps/fix_velocity
  ros::NodeHandle nh2;  // this node handle is used for subscribing to imu/imu/data
  ros::NodeHandle nh4;  // this node handle is used for publishing position and velocity to nav_odom
  // start publishers and subscribers
  ros::Subscriber topic_sub = nh0.subscribe("/wamv/sensors/gps/gps/fix", 1, gpsCallBack);                              // subscribes to gps fix
  ros::Subscriber sub_gpsvel = nh1.subscribe("/wamv/sensors/gps/gps/fix_velocity", 1, gps_velCallBack);        // subscribes to gps fix_velocity
  ros::Subscriber sub_imu = nh2.subscribe("/wamv/sensors/imu/imu/data", 1, imuCallBack);                               // subscribes to the IMU
  ros::Publisher position_pub = nh4.advertise<nav_msgs::Odometry>("nav_odom", 10);                                      // publishes geographic position and velocity */
  
  // set up NodeHandles
  ros::NodeHandle nh3;  // this node handle is used for subscribing to geonav_odom which provides position in x, y
  ros::NodeHandle nh5;  // this node handle is used for publishing thrust information to starboard side 
  ros::NodeHandle nh6;  // this node handle is used for publishing thrust information to port side
  
  // start publishers and subscribers
  ros::Subscriber nav_sub = nh3.subscribe("geonav_odom", 1, pose_update);                                                     // subscribes to geonav_odom
  ros::Publisher stbd_T_pub = nh5.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);     // publishes float value between -1.0 and 1.0 to right thruster
  ros::Publisher port_T_pub = nh6.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);        // publishes float value between -1.0 and 1.0 to left thruster
  
  ros::Time current_time, last_time;  // creates time variables
  current_time = ros::Time::now();   // sets current time to the time it is now
  last_time = ros::Time::now();        // sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);
  ros::spinOnce();                             // initialize position

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok() && !goal)
  {
	  //while(goal==0 && ros::ok())
	  //{
		  //parameters_function(); // get gains from launch file
		  
		  // convert current heading from quaternion into radians
		  psi = atan2((2.0*(q3*q0 + q1*q2)) , (1 - 2*(pow(q2,2.0) + pow(q3,2.0)))); // offset so that 0 is straight ahead at start 
		  
		  // adjust current heading back within -PI and PI
		  if (psi < -PI)
		  {
			  psi = psi + 2*PI;
		  }
		  if (psi > PI)
		  {
			  psi = psi - 2*PI;
		  } 
		  
		  // determine error in x and y (position)
		  e_x = x_goal - x_usv;                                                // calculate error in x position
		  e_y = y_goal - y_usv;                                                // calculate error in y position
		  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                 // calculate magnitude of positional error
		  
		  /* printf("the x is: %f\n", x_usv);       // displays current x posn.
		  printf("the e_x is: %f\n", e_x);       // displays current x posn. error
	      printf("the y is: %f\n", y_usv);       // displays current y posn.
	      printf("the e_y is: %f\n", e_y);       // displays current y posn. error
		  printf("the e_xy is: %f\n", e_xy);   // displays magnitude of positional error */
		  
		  if (e_xy < 20)                                                          // once within desired error, quit I term of PID controller
		  {
			  // reset accumulated error
			  e_xy_total = 0; 
			  e_psi_total = 0;
		  }

		  if (e_xy < 1)                                                          // once within desired error, quit PH controller
		  {
			  printf("Goal Reached!");                                   // inform user goal has been reached
			  goal =true;
		  }

		  // position control law
		  if (k==1) // don't include differential term or integration term first time through
		  {
			  T_x = Kp_xy*e_xy;
		  }
		  else
		  {
			  e_xy_total = e_xy_total + ((e_xy_prev + e_xy)/2.0)*dt; // trapezoidal integration of errors for integral term
			  T_x = Kp_xy*e_xy+ Kd_xy*((e_xy - e_xy_prev)/dt) + Ki_xy*e_xy_total;
		  }

		  // determine error in heading (psi)
		  /* if ((e_y <= 0.25) && (e_y >= -0.25))
		  {
			  psiD = psi;
		  }
		  else
		  {
			  psiD = atan2(e_y, e_x) - 1.5708;
		  } */
		  
		  // heading is based off positional error and is given with respect to the sway (x direction)
		  // CCW is positive
		  psiD = atan2(e_y, e_x); 
		  
		  // adjust desired heading back within -PI and PI
		  if (psiD < -PI)
		  {
			  psiD = psiD + 2*PI;
		  }
		  if (psiD > PI)
		  {
			  psiD = psiD - 2*PI;
		  }
		  
		  e_psi = psiD - psi;
		  printf("psi is: %f\n", psi);                                      // displays current heading
		  printf("psiD is: %f\n", psiD);                                // displays desired heading
		  printf("the e_psi is: %f\n", e_psi);                       // displays heading error

		  // orientation control law
		  if (k==1) // don't include differential term or integration term first time through
		  {
			  M_z = Kp_psi*e_psi;
		  }
		  else
		  {
			  e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt; // trapezoidal integration of errors for integral term
			  M_z = Kp_psi*e_psi + Kd_psi*((e_psi - e_psi_prev)/dt) + Ki_psi*e_psi_total;
		  }

		  // update previous errors
		  e_xy_prev = e_xy; 
		  e_psi_prev = e_psi;
		  
		  //M_z = 0;                                                  // debugggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg
		  // Calculate torque to thrusters
		  T_port = T_x/2.0 - M_z/B;
		  T_stbd = T_x/2.0 + M_z/B;

		  // ensure thrusters are within bounds [-1.0 - 1.0]
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
		  
		  /* printf("LT is: %f\n", T_port);            //displays left thrust value
		  printf("RT is: %f\n", T_stbd);          //displays right thrust value */

		  ros::spinOnce();
		  loop_rate.sleep();
		  k = k+1;
	  //}
	  /* LT.data = 0.0;
	  port_T_pub.publish(LT);
	  RT.data = 0.0;
	  stbd_T_pub.publish(RT);

	  ros::spinOnce();
	  loop_rate.sleep(); */
  }
  LT.data = 0.0;
  port_T_pub.publish(LT);
  RT.data = 0.0;	
  stbd_T_pub.publish(RT);

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}


