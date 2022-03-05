#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
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

std::vector <std::vector <cv::Point>> contours;
std::vector <std::vector <cv::Point>> contours1;
std::vector <std::vector <cv::Point>> contours2;

cv::Scalar yellow_low = cv::Scalar(16, 154, 82); //180
cv::Scalar yellow_high = cv::Scalar(54, 255, 255); 
cv::Scalar green_low = cv::Scalar(59, 180, 25);  //70 //59 now includes the totem //30
cv::Scalar green_high = cv::Scalar(100, 255, 255); 
cv::Scalar blue_low = cv::Scalar(112, 170, 0);   
cv::Scalar blue_high = cv::Scalar(125, 255, 255);

cv::Scalar blue_water_low = cv::Scalar(92, 0, 0);   
cv::Scalar blue_water_high = cv::Scalar(148, 255, 255);
//cv::Scalar purple_low = cv::Scalar(130, 117, 89);
//cv::Scalar purple_high = cv::Scalar(160, 255, 255);
cv::Scalar red_low = cv::Scalar(130, 200, 60);  //bitwise add //200 //60
cv::Scalar red_high = cv::Scalar(179, 255, 255); //sometimes detects red mutliple times
cv::Scalar red_low1 = cv::Scalar(0, 80, 82);  //bitwise add
cv::Scalar red_high1 = cv::Scalar(1, 255, 255); //sometimes detects red mutliple times
cv::Scalar orange_low = cv::Scalar(2, 170, 50); //detects orange but also red sometimes
cv::Scalar orange_high = cv::Scalar(15, 255, 255); //belive this works now
cv::Scalar white_low = cv::Scalar(0, 0, 80); 
cv::Scalar white_high = cv::Scalar(0, 0, 255); 
cv::Scalar black_low = cv::Scalar(0, 0, 0); //black works but detects background black in the tree line
cv::Scalar black_high = cv::Scalar(179, 0, 3); //black works

cv::Mat org_img;
cv::Mat imgHSV;
cv::Mat mask;
cv::Mat red_mask; //mask images for each colour
cv::Mat red_mask1;
cv::Mat orange_mask;
cv::Mat yellow_mask;
cv::Mat green_mask;
cv::Mat blue_mask;
cv::Mat black_mask;
cv::Mat white_mask;
cv::Mat red_res;
cv::Mat orange_res;
cv::Mat green_res;
cv::Mat blue_res;
cv::Mat black_res;
cv::Mat white_res;
cv::Mat yellow_res;


cv::Mat org_img1;
cv::Mat imgHSV1;
cv::Mat mask1;

cv::Mat org_img2;
cv::Mat imgHSV2;
cv::Mat mask2;

float height; //height and width of image 
float width; 
float area; //area of contour

/* void LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
	sensor_msgs::PointCloud2 in_cloud; //creates pointcloud2 message
	sensor_msgs::PointCloud out_cloud; //creates pointclud message for ease of printing and reading
	in_cloud = *lidar_msg; //makes in_cloud equal to the pointer lidar_msg
	sensor_msgs::convertPointCloud2ToPointCloud(in_cloud, out_cloud); //function to convert pointcloud2 to pointcloud
//	printf("the height is: %d\n", lidar_msg->height); //extracts x orientation
//	printf("the width is: %d\n", lidar_msg->width); //extracts x orientation
//	for (int i=0; i < lidar_msg->fields.size(); i++)
//	{
//		printf("the fields is: %d\n", lidar_msg->fields); //extracts x orientation
//	}

 	for (int i=0; i < out_cloud.points.size(); i++) //for loop for iteration through coordinates
	{
		geometry_msgs::Point32 point;
		printf("#####:   LiDAR Point %i   :#####\n", i); 
		point.x = out_cloud.points[i].x;  //goes through x coordinates
		printf("         x:  %f\n", point.x); 
		point.y = out_cloud.points[i].y; //goes through y coordinates
		printf("         y:  %f\n", point.y); 
		point.z = out_cloud.points[i].z; //goes through z coordinates
		printf("         z:  %f\n\n\n\n", point.z);
	} 

} */

void cameraCallBack(const sensor_msgs::ImageConstPtr& camera_msg) {
	try 
	{
		org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("front_left", org_img); //converts the ros image to bgr type
		height = camera_msg->height; 
		width = camera_msg->width; 
		//printf("the height is: %f\n", height); 
		//printf("the width is: %f\n", width); 
	 	cv::Mat base_mask;
		cv::Mat black_mask;
		cv::Mat main_mask;
		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV); //converts BGR to HSV image
		
/* 		cv::Scalar color_low_a = cv::Scalar(0, 150, 0);   
		cv::Scalar color_high_a = cv::Scalar(179, 255, 255);
		cv::inRange(imgHSV, color_low_a, color_high_a, base_mask);
		cv::Scalar color_low_b = cv::Scalar(0, 0, 0);   
		cv::Scalar color_high_b = cv::Scalar(179, 100, 30);
		cv::inRange(imgHSV, color_low_b, color_high_b, black_mask);
		cv::bitwise_or(base_mask, black_mask, main_mask); //check order is correct
		
//		cv::imshow("updated", main_mask);
		
		cv::Mat anti_blue;
		cv::Mat anti_blue1;
		cv::Mat anti_blue2;
		cv::Mat all_mask;
		cv::Mat img_i;
		cv::Scalar color_low_c = cv::Scalar(0, 0, 0);   
		cv::Scalar color_high_c = cv::Scalar(90, 255, 255);
		cv::inRange(imgHSV, color_low_c, color_high_c, anti_blue1);
		cv::Scalar color_low_d = cv::Scalar(150, 0, 0);   //150
		cv::Scalar color_high_d = cv::Scalar(179, 255, 255);
		cv::inRange(imgHSV, color_low_d, color_high_d, anti_blue2);
		cv::bitwise_or(anti_blue1, anti_blue2, anti_blue); //check order is correct
		
//		cv::imshow("updated", anti_blue);
		
		cv::bitwise_or(anti_blue, main_mask, all_mask); //check order is correct
//		cv::imshow("updated", all_mask);
		cv::bitwise_not(all_mask, img_i); //check order is correct
		cv::imshow("updated", img_i); */
		
//		cv::imshow("updated", imgHSV); 
//		cv::Mat threshold_test;
//		cv::inRange(imgHSV, cv::Scalar(hue_low, sat_low, value_low), cv::Scalar(hue_high, sat_high, value_high), threshold_test); 
//		cv::imshow("updated", threshold_test); 
		
		cv::inRange(imgHSV, red_low, red_high, red_mask); 
		cv::inRange(imgHSV, red_low1, red_high1, red_mask1); 
		red_mask = red_mask + red_mask1;
		
		cv::Mat background;
		cv::Mat grey_frame;
		cv::Mat threshold;
		//720
		//1280
//		cv::Mat cropped_image = org_img(cv::Range (155, 520), cv::Range(100, 1280));
		org_img.copyTo(background);
//		cv::cvtColor(org_img, grey_frame, cv::COLOR_BGR2GRAY); //converts RGB to GRAY format for ease of detecting objects
//		cv::threshold(grey_frame, threshold ,40, 255, cv::THRESH_BINARY_INV); //threshold image conversion
//		cv::imshow("updated", cropped_image); 
		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV);
		cv::inRange(imgHSV, blue_low, blue_high, blue_mask); 
		cv::inRange(imgHSV, white_low, white_high, white_mask);  //detects white, orange, black, red, green and blue
		cv::inRange(imgHSV, orange_low, orange_high, orange_mask); 
		cv::inRange(imgHSV, black_low, black_high, black_mask); 
		cv::inRange(imgHSV, red_low, red_high, red_mask); 
		cv::inRange(imgHSV, green_low, green_high, green_mask);  
		cv::inRange(imgHSV, yellow_low, yellow_high, yellow_mask);

		/* float kdata[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
		cv::Mat kernel (3, 3, CV_8U, kdata);
		cv::filter2D(black_mask, black_mask, CV_8U, kernel); */

		cv::medianBlur(black_mask, black_mask, 3);
		cv::medianBlur(blue_mask, blue_mask, 3);
		cv::medianBlur(white_mask, white_mask, 3);
		cv::medianBlur(orange_mask, orange_mask, 3);
		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		cv::medianBlur(yellow_mask, yellow_mask, 3);
		
	
//		cv::imshow("updated", black_mask); 

		cv::bitwise_and(org_img, org_img, mask=red_mask, red_res);
		cv::bitwise_and(org_img, org_img, mask=blue_mask, blue_res);
		cv::bitwise_and(org_img, org_img, mask=white_mask, white_res); //bitwise and to detect white only, orange only, green only and red only
		cv::bitwise_and(org_img, org_img, mask=orange_mask, orange_res);
		cv::bitwise_and(org_img, org_img, mask=green_mask, green_res);
		cv::bitwise_and(org_img, org_img, mask=black_mask, black_res);
		cv::bitwise_and(org_img, org_img, mask=yellow_mask, yellow_res);

 	 /*  cv::findContours(threshold, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			area = cv::contourArea(contours[i]);
			std::string str1 = std::to_string(area);
			cv::putText(cropped_image, str1, (boundRect.tl(), boundRect.br()), 1, 1, (255, 0, 0));
			cv::rectangle(cropped_image, boundRect.tl(), boundRect.br(), (0, 255, 0), 3);
		}    */
		
 		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
//			cv::putText(background, "red",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
//			std::cout << "Red Detected";
		}
//		cv::imshow("updated", red_mask);  	
		
 		cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
//			cv::putText(background, "green",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
//		cv::imshow("updated", green_mask);  	
		
		cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(255, 0, 0), 3);
//			cv::putText(background, "blue",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
//		cv::imshow("updated", blue_mask);  	

		cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(255, 255, 255), 3);
//			cv::putText(background, "white",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
//		cv::imshow("updated", white_mask);  	
		
		cv::findContours(orange_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 179, 255), 3);
//			cv::putText(background, "orange",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
		
	 	cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 0), 3); 
//			cv::putText(background, "black",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		} 
//		cv::imshow("updated", black_mask);  	
		
		cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		/* for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			//cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 255), 3); 
//			cv::putText(background, "yellow",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
		}   */  
		
//		cv::imshow("updated", background);  		
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
//		cv::imshow("yellow", imgHSV); 
		cv::inRange(imgHSV1, red_low, red_high, mask1); 
//		cv::imshow("yellow", mask); 
		cv::findContours(mask1, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//		cv::drawContours(background, contours, -1, (0, 255, 0), 3);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), (0, 0, 0), 3);
		}
		//cv::imshow("updated1", background1); 
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
//		cv::imshow("yellow", imgHSV); 
		cv::inRange(imgHSV2, red_low, red_high, mask2); 
//		cv::imshow("yellow", mask); 
		cv::findContours(mask2, contours2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//		cv::drawContours(background, contours, -1, (0, 255, 0), 3);
		for (size_t i=0; i < contours2.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours2[i]);
			cv::rectangle(background2, boundRect.tl(), boundRect.br(), (0, 0, 0), 3);
		}
		//cv::imshow("updated2", background2); 
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
	//cv::namedWindow("updated1", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("updated2", cv::WINDOW_AUTOSIZE);
	
	cv::createTrackbar("LowH", "front_left", &hue_low, 179); //trackbar for hue
	cv::createTrackbar("HighH", "front_left", &hue_high, 179); 
		
	cv::createTrackbar("LowS", "front_left", &sat_low, 255); //trackbar for saturation
	cv::createTrackbar("HighS", "front_left", &sat_high, 255);
		
	cv::createTrackbar("LowV", "front_left", &value_low, 255); //trackbar for value
	cv::createTrackbar("HighV", "front_left", &value_high, 255);
	
	image_transport::ImageTransport it(nh); //transports the images from the subscriber
	image_transport::ImageTransport it1(nh1);
	image_transport::ImageTransport it2(nh2);
	image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/left_camera/image_raw", 1, cameraCallBack); //front left camera
	image_transport::Subscriber camera_sub1 = it1.subscribe("/wamv/sensors/cameras/right_camera/image_raw", 1, cameraCallBack1); //front rightcamera
	image_transport::Subscriber camera_sub2 = it2.subscribe("/wamv/sensors/cameras/middle_camera/image_raw", 1, cameraCallBack2); //middle_right camera
	//ros::Subscriber lidar_sub = nh3.subscribe("/wamv/sensors/lidars/lidar_wamv/points", 10, LidarCallBack); //subscribes to Lidar

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
