// Filename:                     	camera_capture.cpp
// Creation Date:					04/20/2022
// Last Revision Date:			04/20/2022
// Author(s) [email]:				Taylor Lamorie [tlamorie@lssu.edu]
// Revisor(s) {Date}:        
// Organization/Institution:	Lake Superior State University

//...................................................About camera_capture.cpp.....................................................................
//subscribes to both the right and left camera on the Zed2i and uses opencv to convert into a showable image


//................................................Included Libraries and Message Types..........................................
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "iostream"
#include "stdio.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <string>

using namespace cv;
using namespace std;


// THIS FUNCTION: Subscribed to the Right side of the camera and uses opencv to show in a window 
// ACCEPTS: (sensor_msgs/Image pointer)
// RETURNS: (VOID)
// =============================================================================
void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try 
	{
		ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
		cv::Mat org_img;
		org_img = cv_bridge::toCvShare(msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("Right_side", org_img);                                     //displays the image in a window of default size
		
		ros::NodeHandle nh;
		int s;
		if (nh.getParam("/right_camera", s))                                   //set param to 1 to take a picture 
		{
			ROS_INFO("Got param for right");
        }
        else
        {
			ROS_ERROR("Failed to get param 'right_camera");
        }
		
		if (s == 1);                                                                 //if param is 1 
		{
		static int image_count_right;                                //static varaible to keep count of picture 
		std::stringstream sstream;                               //string object converted to stream for saving 
		sstream << "my_image_right" << image_count_right << ".png" ;                  //name of image file 
		ROS_ASSERT(cv::imwrite(sstream.str(),  org_img));      //runs the imwrite command 
		image_count_right++;                                      //counts image
		 nh.setParam("/right_camera", 0);                          //should set param back to 0 so it doesn't take more then 1 picture
		}
	
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg -> encoding.c_str()); //prints out the encoding string
	}
} //END

// THIS FUNCTION: Subscribed to the Left side of the camera and uses opencv to show in a window 
// ACCEPTS: (sensor_msgs/Image pointer)
// RETURNS: (VOID)
// =============================================================================
void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try 
	{
		ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
		cv::Mat org_img1;
		org_img1 = cv_bridge::toCvShare(msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("Left_side", org_img1);                                     //displays the image in a window of default size
		
		ros::NodeHandle nh1;
		int s1;
		if (nh1.getParam("/left_camera", s1))                                  //left camera param set to 1
		{
			ROS_INFO("Got param for left");
        }
        else
        {
			ROS_ERROR("Failed to get param 'left_camera");
        }
		
		if (s1 == 1);                                                                 //s1 different then other side camera
		{
			static int image_count_left;                                //static varaible to keep count of picture 
			std::stringstream sstream;                                  //string object converted to stream for saving 
			sstream << "my_image_left" << image_count_left << ".png" ;                    //name of image file 
			ROS_ASSERT(cv::imwrite(sstream.str(),  org_img1));       //runs the imwrite command 
			image_count_left++;                                         //counts image
			nh1.setParam("/left_camera", 0);                           //should set param back to 0 so it doesn't take more then 1 picture
		}
		
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg -> encoding.c_str()); //prints out the encoding string
	}
} //END

//...............................................................Main Program..............................................................
int main(int argc, char** argv)
{
	ros::init(argc, argv, "zed_video_subscriber");
  
  	cv::namedWindow("Right_side", cv::WINDOW_AUTOSIZE);					
	cv::namedWindow("Left_side", cv::WINDOW_AUTOSIZE);
  
	ros::NodeHandle n, n1;
  
  	image_transport::ImageTransport it1(n);		// transports the images from published node to subscriber
	image_transport::ImageTransport it2(n1);

	image_transport::Subscriber subRightRectified = it1.subscribe("/zed2i/zed_node/right/image_rect_color", 1, imageRightRectifiedCallback); //subscribes to right camera rectified 
	image_transport::Subscriber subLeftRectified = it2.subscribe("/zed2i/zed_node/left/image_rect_color", 1, imageLeftRectifiedCallback);  //subscribes to left camera rectified 

	ros::spin();

	cv::destroyWindow("Right_side"); //destroys the new windows
	cv::destroyWindow("Left_side");
	
	ros::Rate loop_rate(1);
	
	while(ros::ok()) {
	
	ros::spinOnce();
	loop_rate.sleep();
	
	}
	
}

