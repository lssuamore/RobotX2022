#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "ros/ros.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"
#include "stdio.h"
#include "cv_bridge/cv_bridge.h"
#include <vector>
#include <string>

int hue_low = 0;
int hue_high = 179;

int sat_low = 0;
int sat_high = 255;

int value_low = 0;
int value_high = 255; 

cv::Mat org_img;
cv::Mat imgHSV;

cv::Mat org_img1;
cv::Mat imgHSV1;

cv::Mat org_img2;
cv::Mat imgHSV2;

float height; //height and width of image 
float width; 
float area; //area of contour

void cameraCallBack(const sensor_msgs::ImageConstPtr& camera_msg) {
	try 
	{
		org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("front_left", org_img); //converts the ros image to bgr type
		height = camera_msg->height; 
		width = camera_msg->width; 
		printf("the height is: %f\n", height); 
		printf("the width is: %f\n", width); 
		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV); //converts BGR to HSV image
		
		cv::Mat threshold_test;
		cv::inRange(imgHSV, cv::Scalar(hue_low, sat_low, value_low), cv::Scalar(hue_high, sat_high, value_high), threshold_test); 
		cv::imshow("updated", threshold_test); 
			
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
		cv::Mat background1;
		org_img1.copyTo(background1);
		cv::cvtColor(org_img1, imgHSV1, cv::COLOR_BGR2HSV);
		
		cv::Mat threshold_test1;
		cv::inRange(imgHSV1, cv::Scalar(hue_low, sat_low, value_low), cv::Scalar(hue_high, sat_high, value_high), threshold_test1); 
		cv::imshow("updated1", threshold_test1); 
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
		cv::Mat background2;
		org_img2.copyTo(background2);
		cv::cvtColor(org_img2, imgHSV2, cv::COLOR_BGR2HSV);

		cv::Mat threshold_test2;
		cv::inRange(imgHSV2, cv::Scalar(hue_low, sat_low, value_low), cv::Scalar(hue_high, sat_high, value_high), threshold_test2); 
		cv::imshow("updated2", threshold_test2); 
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
	
	cv::namedWindow("front_left"); //creates new windows for each camera
	cv::namedWindow("front_right");
	cv::namedWindow("middle_right");
	cv::namedWindow("updated", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("updated1", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("updated2", cv::WINDOW_AUTOSIZE);
	
	cv::createTrackbar("LowH", "front_left", &hue_low, 179); //trackbar for hue
	cv::createTrackbar("HighH", "front_left", &hue_high, 179); 
		
	cv::createTrackbar("LowS", "front_left", &sat_low, 255); //trackbar for saturation
	cv::createTrackbar("HighS", "front_left", &sat_high, 255);
		
	cv::createTrackbar("LowV", "front_left", &value_low, 255); //trackbar for value
	cv::createTrackbar("HighV", "front_left", &value_high, 255);
	
	cv::createTrackbar("LowH", "front_right", &hue_low, 179); //trackbar for hue
	cv::createTrackbar("HighH", "front_right", &hue_high, 179); 
		
	cv::createTrackbar("LowS", "front_right", &sat_low, 255); //trackbar for saturation
	cv::createTrackbar("HighS", "front_right", &sat_high, 255);
		
	cv::createTrackbar("LowV", "front_right", &value_low, 255); //trackbar for value
	cv::createTrackbar("HighV", "front_right", &value_high, 255);
	
	cv::createTrackbar("LowH", "middle_right", &hue_low, 179); //trackbar for hue
	cv::createTrackbar("HighH", "middle_right", &hue_high, 179); 
		
	cv::createTrackbar("LowS", "middle_right", &sat_low, 255); //trackbar for saturation
	cv::createTrackbar("HighS", "middle_right", &sat_high, 255);
		
	cv::createTrackbar("LowV", "middle_right", &value_low, 255); //trackbar for value
	cv::createTrackbar("HighV", "middle_right", &value_high, 255);
	
	image_transport::ImageTransport it(nh); //transports the images from the subscriber
	image_transport::ImageTransport it1(nh1);
	image_transport::ImageTransport it2(nh2);
	image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, cameraCallBack); //front left camera
	image_transport::Subscriber camera_sub1 = it1.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 1, cameraCallBack1); //front rightcamera
	image_transport::Subscriber camera_sub2 = it2.subscribe("/wamv/sensors/cameras/middle_right_camera/image_raw", 1, cameraCallBack2); //middle_right camera

	ros::spin();
	cv::destroyWindow("front_left"); //destroys the new windows
	cv::destroyWindow("front_right");
	cv::destroyWindow("middle_right");
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
	
	ros::spinOnce();
	loop_rate.sleep();
	
	}
}