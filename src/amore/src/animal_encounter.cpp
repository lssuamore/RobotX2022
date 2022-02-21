#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "iostream"
#include "stdio.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <string>
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include "geographic_msgs/GeoPath.h"
#include "iostream"
#include "stdio.h"
#include "time.h"

using namespace cv;
using namespace std;

cv::Mat org_img;
cv::Mat org_img1;
cv::Mat org_img2;


double latitude_turtle;
double longitude_turtle;
double altitude_turtle;

double latitude_platypus;
double longitude_platypus;
double altitude_platypus;

double latitude_crocodile;
double longitude_crocodile;
double altitude_crocodile;

int i = 0;
/* double orientationx;
double orientationy; 
double orientationz;
double orientationw;  */


void AnimalCallBack(const geographic_msgs::GeoPath::ConstPtr& animal_msg) {

	for (unsigned i=1;i<animal_msg->poses.size();i++) {
		//printf(%c, animal_msg->poses[i].header.frame_id);
	if (animal_msg->poses[i].header.frame_id == "turtle") {
		latitude_turtle = animal_msg->poses[i].pose.position.latitude; //sets latitude from gps
		longitude_turtle = animal_msg->poses[i].pose.position.longitude; //sets longitude from gps
		altitude_turtle = animal_msg->poses[i].pose.position.altitude; //sets altitude rom gps
		/* orientationx = animal_msg->orientation.x; //sets orientationx
		orientationy = animal_msg->orientation.y; //sets orientationy
		orientationz = animal_msg->orientation.z; //sets orientationz
		orientationw = animal_msg->orientation.w; //sets orientationw */
		printf("the latitude of turtle is: %f\n", latitude_turtle); //prints
		printf("the longitude of turtle: %f\n", longitude_turtle);
		printf("the altitude of turtle: %f\n", altitude_turtle);
		i = 1;
	}

	else if (animal_msg->poses[i].header.frame_id == "platypus") {
		latitude_platypus = animal_msg->poses[i].pose.position.latitude; //sets latitude from gps
		longitude_platypus = animal_msg->poses[i].pose.position.longitude; //sets longitude from gps
		altitude_platypus = animal_msg->poses[i].pose.position.altitude; //sets altitude rom gps
		printf("the latitude of platypus is: %f\n", latitude_platypus); //prints
		printf("the longitude of platypus: %f\n", longitude_platypus);
		printf("the altitude of platypus: %f\n", altitude_platypus);
		i = 2;
	}
	
	else if (animal_msg->poses[i].header.frame_id  == "crocodile") {
		latitude_crocodile = animal_msg->poses[i].pose.position.latitude; //sets latitude from gps
		longitude_crocodile = animal_msg->poses[i].pose.position.longitude; //sets longitude from gps
		altitude_crocodile = animal_msg->poses[i].pose.position.altitude; //sets altitude rom gps
		printf("the latitude of crocodile is: %f\n", latitude_crocodile); //prints
		printf("the longitude of crocodile: %f\n", longitude_crocodile);
		printf("the altitude of crocodile: %f\n", altitude_crocodile);
		i = 3;
	}
	
	else {
		printf("No animal detected\n");
	}
	}

}


void callback(const nav_msgs::Odometry::ConstPtr& odom) {
	printf("the x is: %f\n", odom->pose.pose.position.x); //extracts x coor from nav_odometery
	printf("the y is: %f\n", odom->pose.pose.position.y); //extracts y coor from nav_odometery
}


			

void cameraCallBack(const sensor_msgs::ImageConstPtr& camera_msg) {
	try 
	{
		org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("front_left", org_img); //shows the image in front left window
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera_msg -> encoding.c_str()); //prints out the encoding string
	}
	
}

void cameraCallBack1(const sensor_msgs::ImageConstPtr& camera_msg) {

	try 
	{
		org_img1 = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("front_right", org_img1); //converts the ros image to bgr type
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera_msg -> encoding.c_str()); //prints out the encoding string
	}
	
}

void cameraCallBack2(const sensor_msgs::ImageConstPtr& camera_msg) {

	try 
	{
		org_img2 = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("middle_right", org_img2); //converts the ros image to bgr type
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera_msg -> encoding.c_str()); //prints out the encoding string
	}
	
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "Camera");
	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
	ros::NodeHandle nh3;
	ros::NodeHandle nh4;
	ros::NodeHandle nh5;
	
	
	cv::namedWindow("front_left"); //creates new windows for each camera
	cv::namedWindow("front_right");
	cv::namedWindow("middle_right");
	
	image_transport::ImageTransport it(nh); //transports the images from the subscriber
	image_transport::ImageTransport it1(nh1);
	image_transport::ImageTransport it2(nh2);
	image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, cameraCallBack); //front left camera
	image_transport::Subscriber camera_sub1 = it1.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 1, cameraCallBack1); //front rightcamera
	image_transport::Subscriber camera_sub2 = it2.subscribe("/wamv/sensors/cameras/middle_right_camera/image_raw", 1, cameraCallBack2); //middle_right camera
	
	ros::Subscriber animal_pose = nh3.subscribe("/vrx/wildlife/animals/poses", 10, AnimalCallBack); //subscribes to animal topic
	
	ros::Publisher position_animal = nh4.advertise<nav_msgs::Odometry>("nav_odom", 1000); //publishes geographic position and velocity
	ros::Subscriber sub_odom = nh5.subscribe("geonav_odom", 10, callback);  //subscribes to local odom frame
	
	//subscribe to ros topic and go to gazebo and figure out the message type rosmsg

	ros::spin();
	cv::destroyWindow("front_left"); //destroys the new windows
	cv::destroyWindow("front_right");
	cv::destroyWindow("middle_right");
	
	ros::Time current_time, last_time; //takes current time
	current_time = ros::Time::now(); //sets current time to the time it is now
	last_time = ros::Time::now(); //sets last time to the time it is now
	
	
	
	current_time = ros::Time::now(); //time
	nav_msgs::Odometry nav_odom; //new message
	switch (i) {
		case 1:  
			nav_odom.header.seq +=1; //sequence number
			nav_odom.header.stamp = current_time; //sets stamp to current time
			nav_odom.header.frame_id = "odom"; //header frame
			nav_odom.child_frame_id = "base_link"; //child frame
			nav_odom.pose.pose.position.x = longitude_turtle; //sets long
			nav_odom.pose.pose.position.y = latitude_turtle; //sets lat
			nav_odom.pose.pose.position.z = altitude_turtle; //sets altitude
		//	nav_odom.pose.pose.orientation = odom_quat; //sets orientation
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
			position_animal.publish(nav_odom); //publishes above to position_pub
			i = 0;
			break;
			
		case 2:  
			nav_odom.header.seq +=1; //sequence number
			nav_odom.header.stamp = current_time; //sets stamp to current time
			nav_odom.header.frame_id = "odom"; //header frame
			nav_odom.child_frame_id = "base_link"; //child frame
			nav_odom.pose.pose.position.x = longitude_platypus; //sets long
			nav_odom.pose.pose.position.y = latitude_platypus; //sets lat
			nav_odom.pose.pose.position.z = altitude_platypus; //sets altitude
		//	nav_odom.pose.pose.orientation = odom_quat; //sets orientation
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
			position_animal.publish(nav_odom); //publishes above to position_pub
			i = 0;
			break;
			
		case 3:  
			nav_odom.header.seq +=1; //sequence number
			nav_odom.header.stamp = current_time; //sets stamp to current time
			nav_odom.header.frame_id = "odom"; //header frame
			nav_odom.child_frame_id = "base_link"; //child frame
			nav_odom.pose.pose.position.x = longitude_crocodile; //sets long
			nav_odom.pose.pose.position.y = latitude_crocodile; //sets lat
			nav_odom.pose.pose.position.z = altitude_crocodile; //sets altitude
		//	nav_odom.pose.pose.orientation = odom_quat; //sets orientation
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
			position_animal.publish(nav_odom); //publishes above to position_pub
			i = 0;
			break;
	}
	
	
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		
		/* current_time = ros::Time::now(); //time
		nav_msgs::Odometry nav_odom; //new message
	switch (i) {
		case 1:  
			nav_odom.header.seq +=1; //sequence number
			nav_odom.header.stamp = current_time; //sets stamp to current time
			nav_odom.header.frame_id = "odom"; //header frame
			nav_odom.child_frame_id = "base_link"; //child frame
			nav_odom.pose.pose.position.x = longitude_turtle; //sets long
			nav_odom.pose.pose.position.y = latitude_turtle; //sets lat
			nav_odom.pose.pose.position.z = altitude_turtle; //sets altitude
		//	nav_odom.pose.pose.orientation = odom_quat; //sets orientation
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
			position_animal.publish(nav_odom); //publishes above to position_pub
			i = 0;
			break;
			
		case 2:  
			nav_odom.header.seq +=1; //sequence number
			nav_odom.header.stamp = current_time; //sets stamp to current time
			nav_odom.header.frame_id = "odom"; //header frame
			nav_odom.child_frame_id = "base_link"; //child frame
			nav_odom.pose.pose.position.x = longitude_platypus; //sets long
			nav_odom.pose.pose.position.y = latitude_platypus; //sets lat
			nav_odom.pose.pose.position.z = altitude_platypus; //sets altitude
		//	nav_odom.pose.pose.orientation = odom_quat; //sets orientation
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
			position_animal.publish(nav_odom); //publishes above to position_pub
			i = 0;
			break;
			
		case 3:  
			nav_odom.header.seq +=1; //sequence number
			nav_odom.header.stamp = current_time; //sets stamp to current time
			nav_odom.header.frame_id = "odom"; //header frame
			nav_odom.child_frame_id = "base_link"; //child frame
			nav_odom.pose.pose.position.x = longitude_crocodile; //sets long
			nav_odom.pose.pose.position.y = latitude_crocodile; //sets lat
			nav_odom.pose.pose.position.z = altitude_crocodile; //sets altitude
		//	nav_odom.pose.pose.orientation = odom_quat; //sets orientation
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
			position_animal.publish(nav_odom); //publishes above to position_pub
			i = 0;
			break;
			
	} */
		
		
		
		
	
	ros::spinOnce();
	loop_rate.sleep();
	
	}
}