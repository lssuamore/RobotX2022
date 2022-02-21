// Filename:								traj_pub.cpp
// Creation Date:						10/11/2021
// Last Revision Date:
// Author(s) [email]:					Shaede Perzanowski [sperzanowski1@lssu.edu]
// Revisor(s) [Revision Date]:
// Organization/Institution:			Lake Superior State University

//...................................................About traj_pub.cpp.....................................................................
// The traj_pub.cpp file is used to publish different goal trajectories to a node for the WAM-V to
// subscribe to. Different goal positions will be published to the goal_pos node. To change trajectories,
// comment and uncomment the appropriate sections. 

// Inputs and Outputs of the traj_pub.cpp file
//				Inputs: none (it uses the ROS simulation time)
//				Outputs: q (nav_msgs::Odometry type that contains pose information) to goal_pos node


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
//...........................................End of Included Libraries and Message Types....................................

//.................................................................Constants..................................................................
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
nav_msgs::Odometry q;						// create an odometry message structure to share goal position to goal_pos node
int loop_count=0;            // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
// THIS FUNCTION: sets the header time stamp for q
// ACCEPTS: nothing (uses the global variable q and the ROS simulation time)
// RETURNS: nothing
// =============================================================================
void HeadTime()
{
	// Set the header stamp time of q
	ros::Time current_time;  // create ROS time variable
	
	if(!ros::Time::isValid())	// ensures the simulation time has been initialized in some manner
	{
		ros::Time::init();			// if simulation time has not been initialized, initialize it
	}
	
	current_time = ros::Time::now();		// capture the current simulation time
	q.header.stamp = current_time; //sets header time stamp to current simulation time  
} // end of HeadTime()
//............................................................End of Functions............................................................

//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{

  // names the program for visual purposes
  ros::init(argc, argv, "traj_pub");
  
  // Variables
  int i;					// for indexing for-loops
  float y = 0.0;		// holds desired y-value
  float x = 0.0;		// holds desired x-value
  
  // Initialize nav_msgs::Odometry message q
  q.header.seq = 0; //sequence number
  q.header.frame_id = "odom"; //header frame
  q.child_frame_id = "base_link"; //child frame
  // Set some pose information
  q.pose.pose.position.z = 0.0;
  q.pose.pose.orientation.x = 0.0;
  q.pose.pose.orientation.y = 0.0;
  q.pose.pose.orientation.z = 0.0;
  q.pose.pose.orientation.w = 0.0;
  q.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Set all velocity information to zero
  q.twist.twist.linear.x = 0.0;
  q.twist.twist.linear.y = 0.0;
  q.twist.twist.linear.z = 0.0;
  q.twist.twist.angular.x = 0.0;
  q.twist.twist.angular.y = 0.0;
  q.twist.twist.angular.z = 0.0;
  q.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Node handles
  ros::NodeHandle n1; // for the goal position node

  // Publishers
  ros::Publisher trajectory_pub = n1.advertise<nav_msgs::Odometry>("goal_pos", 1000);
  
  // Subscribers
  
  // Initialize simulation time
  ros::Time::init();
  
  // Set the loop sleep rate
  ros::Rate loop_rate(4); // 10 Hz
  
  // DIRECTIONS FOR CHANGING TRAJECTORY: Only have one curve (for-loop) uncommented at a time
  //			uncomment the std::cout commands to show the data points of a trajectory in command window
  while(ros::ok())		//rosk::ok() will stop when the user inputs Ctrl+C
  {
	if (loop_count>10)
	{
		// .....Straight lines.....
		// y = 0..........................................
		for(i=0; i<500; i++)
		{
			x = 0.0;
			y = y + (float)i;
			//std::cout << "x = " << x << "\n";
			//std::cout << "y = " << y << "\n";
			q.header.seq +=1; //sequence number
			q.pose.pose.position.x = x;				// set goal x-coordinate
			q.pose.pose.position.y = y;				// set goal y-coordinate
			HeadTime();										// call publisher function
			trajectory_pub.publish(q);					// Publish the goal position data q to the goal_pos node
		} // end of for-loop for y=0
		// y = x..........................................
		/* for(i=0; i<500; i++)
		{
			x = (float)i;
			y = x;
			//std::cout << "x = " << x << "\n";
			//std::cout << "y = " << y << "\n";
			q.header.seq +=1; //sequence number
			q.pose.pose.position.x = x;				// set goal x-coordinate
			q.pose.pose.position.y = y;				// set goal y-coordinate
			HeadTime();										// call publisher function
			trajectory_pub.publish(q);					// Publish the goal position data q to the goal_pos node
		} // end of for-loop for y=x */
		// y = (-3/13)x.................................
		/* for(i=0; i<500; i++)
		{
			x = (float)i;
			y = -(3.0/13.0)*x;
			//std::cout << "x = " << x << "\n";
			//std::cout << "y = " << y << "\n";
			q.header.seq +=1; //sequence number
			q.pose.pose.position.x = x;				// set goal x-coordinate
			q.pose.pose.position.y = y;				// set goal y-coordinate
			HeadTime();										// call publisher function
			trajectory_pub.publish(q);					// Publish the goal position data q to the goal_pos node
		} // end of for-loop for y=(-3/13)x */
	
		// .....Sinusoids.....
		// y = sin(x/2)..................................
		/* for(i=0; i<500; i++)
		{
			x = (float)i;
			y = sin(x/2.0);
			//std::cout << "x = " << x << "\n";
			//std::cout << "y = " << y << "\n";
			q.header.seq +=1; //sequence number
			q.pose.pose.position.x = x;				// set goal x-coordinate
			q.pose.pose.position.y = y;				// set goal y-coordinate
			HeadTime();										// call publisher function
			trajectory_pub.publish(q);					// Publish the goal position data q to the goal_pos node
		} // end of for-loop for y=sin(x/2) */
	
		// .....Power Functions.....
		// y = 2^x - 1...................................
		/* for(i=0; i<500; i++)
		{
			x = (float)i;
			y = pow(2.0,x) - 1.0;
			//std::cout << "x = " << x << "\n";
			//std::cout << "y = " << y << "\n";
			q.header.seq +=1; //sequence number
			q.pose.pose.position.x = x;				// set goal x-coordinate
			q.pose.pose.position.y = y;				// set goal y-coordinate
			HeadTime();										// call publisher function
			trajectory_pub.publish(q);					// Publish the goal position data q to the goal_pos node
		} // end of for-loop for y=2^x - 1 */
		// y = 2^x.......................................
		/* for(i=0; i<500; i++)
		{
			x = (float)i;
			y = pow(2.0,x);
			//std::cout << "x = " << x << "\n";
			//std::cout << "y = " << y << "\n";
			q.header.seq +=1; //sequence number
			q.pose.pose.position.x = x;				// set goal x-coordinate
			q.pose.pose.position.y = y;				// set goal y-coordinate
			HeadTime();										// call publisher function
			trajectory_pub.publish(q);					// Publish the goal position data q to the goal_pos node
		} // end of for-loop for y=2^x */
	
		// .....Other Curves.....
		// y = x^(-x) + (0.7)x.......................
		/* for(i=0; i<500; i++)
		{
			x = (float)i;
			y = pow(x,-x) + (0.7)*x;
			//std::cout << "x = " << x << "\n";
			//std::cout << "y = " << y << "\n";
			q.header.seq +=1; //sequence number
			q.pose.pose.position.x = x;				// set goal x-coordinate
			q.pose.pose.position.y = y;				// set goal y-coordinate
			HeadTime();										// call publisher function
			trajectory_pub.publish(q);					// Publish the goal position data q to the goal_pos node
		} // end of for-loop for y=x^(-x) + (0.7)x */
	
	}
	//sends out any data necessary then waits based on the loop rate
	ros::spinOnce();
	loop_rate.sleep();
	loop_count = loop_count + 1;

  } // end of main program loop
  return 0;
} // end of main()
//............................................................End of Main Program...........................................................
