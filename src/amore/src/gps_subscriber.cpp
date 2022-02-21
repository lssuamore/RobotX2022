//  Filename:						        gps_subscriber.cpp
//  Creation Date:						10/15/2021
//  Last Revision Date:                
//  Author(s) [email]:					Taylor Lamorie [tlamorie@lssu.edu]
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................gps_subscriber.cpp.......................
//  This code publishes the USV lat and long coordinates to nav_odom.
//  This allows the geonav_transform package to subscribe to the nav_odom node.
//  Geonav_transform allows USV coordinates to be read off in NED coordinates. 
//
//  Inputs and Outputs of the gps_subscriber.cpp file
//				Inputs: USV lat, long, amplitude, orientation, and speed
//				Outputs: nav_msgs::Odometry

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "iostream"
#include "stdio.h"
#include "time.h"
//#include "std_msgs/Float32.h"

// q0 - q3 are current orientation in quaternion form
float q1;            
float q2;
float q3;
float q0;
float psi;                          // this variable is updated as the WAM-V z-orientation in real time through the compass

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

void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	th = gps_msg->altitude; //sets altitude rom gps
	//printf("the latitude is: %f\n", latitude); //prints
	//printf("the longitude is: %f\n", longitude);
	//printf("the altitude is: %f\n", th);
}

// nav_msgs odometry
void callback(const nav_msgs::Odometry::ConstPtr& odom) {
	// ROS_FATAL("USV x is: %f\n", odom->pose.pose.position.y); //extracts x coor from nav_odometery
	// ROS_FATAL("USV y is: %f\n", odom->pose.pose.position.x); //extracts y coor from nav_odometery
	//printf("the x orientation is: %f\n", odom->pose.pose.orientation.x); //extracts x orientation
	//printf("the y orientation is: %f\n", odom->pose.pose.orientation.y); //extracts y orientation
	//printf("the z orientation is: %f\n", odom->pose.pose.orientation.z); //extracts z orientation
	//printf("the w orientation is: %f\n", odom->pose.pose.orientation.w); //extracts w orientation
	//printf("the velocity x is: %f\n", odom-> twist.twist.linear.x);//prints velocity x
	//printf("the velocity y is: %f\n", odom-> twist.twist.linear.y);//prints velocity y
	//printf("the velocity z is: %f\n", odom-> twist.twist.linear.z);//prints velocity z
}

 void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg) {
	orientationx = imu_msg->orientation.x; //sets orientationx
	orientationy = imu_msg->orientation.y; //sets orientationy
	orientationz = imu_msg->orientation.z; //sets orientationz
	orientationw = imu_msg->orientation.w; //sets orientationw
//	q1 = orientationx;
//	q2 = orientationy;
//	q3 = orientationz;
//	q0 = orientationw;
	// calculate current heading in radians
//	psi = atan2((2.0*(q3*q0 + q1*q2)) , (1 - 2*(pow(q2,2.0) + pow(q3,2.0))));
//	printf("the orientation is: %f\n", psi);                  // prints current heading
//	printf("the orientation x is: %f\n", orientationx); //prints
//	printf("the orientation y is: %f\n", orientationy); //prints
//	printf("the orientation z is: %f\n", orientationz); //prints
//	printf("the orientation w is: %f\n", orientationw); //prints
}

void gps_velCallBack(const geometry_msgs::Vector3Stamped::ConstPtr& vel_msg) {
	velx = vel_msg->vector.x; //sets velx
	vely = vel_msg->vector.y; //sets vely
	velz = vel_msg->vector.z; //sets velz
//	printf("the velocity x is: %f\n", velx);//prints velocity x
//	printf("the velocity y is: %f\n", vely); //prints velocity y
//	printf("the velocity z is: %f\n", velz); //prints velocity z
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Geoposition");

	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
	ros::NodeHandle nh3;
	ros::NodeHandle nh4;

	ros::Subscriber topic_sub = nh.subscribe("/wamv/sensors/gps/gps/fix", 10, gpsCallBack);
	ros::Publisher position_pub = nh1.advertise<nav_msgs::Odometry>("nav_odom", 1000); //publishes geographic position and velocity
	ros::Subscriber sub = nh2.subscribe("geonav_odom", 10, callback);  //subscribes to local odom frame
	ros::Subscriber sub_imu = nh3.subscribe("/wamv/sensors/imu/imu/data", 1000, imuCallBack);  //subscribes to the IMU
	ros::Subscriber sub_gpsvel = nh4.subscribe("/wamv/sensors/gps/gps/fix_velocity", 1000, gps_velCallBack);  //subscribes to gps velocity

	ros::Time current_time, last_time; //takes current time
	current_time = ros::Time::now(); //sets current time to the time it is now
	last_time = ros::Time::now(); //sets last time to the time it is now
	ros::Rate loop_rate(10);

	while(ros::ok()) {

	current_time = ros::Time::now(); //time

//	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); //creates orientation

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

//	last_time = current_time
	ros::spinOnce();
	loop_rate.sleep();
	}

}