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

std::vector <std::vector <cv::Point>> contours;
std::vector <std::vector <cv::Point>> c;
std::vector<cv::Point> approx; //array 

//colors for the scan the code
cv::Scalar yellow_low = cv::Scalar(16, 154, 82); 
cv::Scalar yellow_high = cv::Scalar(54, 255, 255); 
cv::Scalar green_low = cv::Scalar(60, 220, 0);  
cv::Scalar green_high = cv::Scalar(100, 255, 255); 
cv::Scalar blue_low = cv::Scalar(112, 170, 0);   
cv::Scalar blue_high = cv::Scalar(125, 255, 255);
cv::Scalar red_low = cv::Scalar(0, 1, 0);  
cv::Scalar red_high = cv::Scalar(1, 255, 255); 

cv::Scalar buoy_low = cv::Scalar(0, 0, 0); 
cv::Scalar buoy_high = cv::Scalar(179, 5, 179);  // cant do by color because the floaty part of the dock is the same hsv colors

cv::Mat org_img;
cv::Mat imgHSV;
cv::Mat mask;
cv::Mat red_mask; //mask images for each colour
cv::Mat yellow_mask;
cv::Mat green_mask;
cv::Mat blue_mask;

cv::Mat red_res;
cv::Mat green_res;
cv::Mat blue_res;
cv::Mat yellow_res;

cv::Mat mask_all;

//used to display all colors
cv::Mat background;

cv::Mat light_buoy;

int center_x;

char res [3];
char yell = 'Y';
char re = 'R';
char gree = 'G';
char blu = 'B';
std::string color;
std::string docking_symbol;
std::string docking_color;

//tis function looks for the light buoy and returns true or false
bool detectLightBuoy(cv::Mat scanner) { //bool //cant do by color see above
	cv::Mat imgHSV1;
	cv::cvtColor(org_img, imgHSV1, cv::COLOR_BGR2HSV); //converts BGR to HSV image
		
	cv::inRange(imgHSV, buoy_low, buoy_high, scanner); //detect buoy scanner
	
	cv::medianBlur(scanner, scanner, 3);
	
	cv::findContours(scanner, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
	for (size_t i=0; i < contours.size(); ++i) {
		cv::Rect boundRect = cv::boundingRect(contours[i]);
		cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
	} 	
	return true; //return a bool either true is scan is detected or false if it isn't
}


//takes an image as an input and will approximate shape
/* void detectShape(cv::Mat self) {
	std::vector <std::vector <cv::Point>> c; //contours
	std::vector<cv::Point> approx; //array 
	cv::findContours(self, c, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contours of input image 
	//contours is c and peri is perimeter
	float peri = cv::arcLength(c, true);
	//uses opencv function to calculate the approximate sides
	cv::approxPolyDP(c, approx, 0.04 * peri, true);
	
	if (sizeof(approx) == 3) { //if 3 sides then it is a triangle 
		printf("Triangle\n");
	}
	
	else if (sizeof(approx) == 4) { //if 4 sides and ar is between 0.95 and 1.05 then it is a square else it is a rectangle
		float x, y;
        cv::Rect rect = cv::boundingRect(approx);
		x = rect.width;
		y = rect.height;
        float ar = x / y;
		if (ar>= 0.95 & ar <= 1.05) {
			printf("Square\n");
		}
		else {
			printf("Rectangle\n");
		}
	}
	
	else if (sizeof(approx) == 12) { //12 sides it is the cross
		printf("Cross\n");
	}
	
	else { //if no sides then a circle
		printf("Circle\n");
	}
} */

void cameraCallBack(const sensor_msgs::ImageConstPtr& camera_msg) {
	try 
	{
		org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("front_left", org_img); //converts the ros image to bgr type
		cv::Mat main_mask;
		cv::Mat main_mask_f;
		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV); //converts BGR to HSV image
	
		cv::Mat grey_frame;
		cv::Mat threshold;
	
		org_img.copyTo(background);
		org_img.copyTo(light_buoy);

		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV);
		
		cv::inRange(imgHSV, red_low, red_high, red_mask); //detect red
		cv::inRange(imgHSV, blue_low, blue_high, blue_mask);  //detects blue
		cv::inRange(imgHSV, green_low, green_high, green_mask);  //detects green
		cv::inRange(imgHSV, yellow_low, yellow_high, yellow_mask); //detects yellow
		
		cv::medianBlur(blue_mask, blue_mask, 3);
		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		cv::medianBlur(yellow_mask, yellow_mask, 3);

/* 		//adds all masks together using bitwise_and 
		cv::bitwise_and(org_img, org_img, mask=red_mask, red_res);
		cv::bitwise_and(org_img, org_img, mask=blue_mask, blue_res);
		cv::bitwise_and(org_img, org_img, mask=green_mask, green_res);
		cv::bitwise_and(org_img, org_img, mask=yellow_mask, yellow_res); */
		
 		/* cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			//cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
			cv::drawContours(background, contours, 0, 255, 1);
			color = "red"; // color equal to the contour color
		} 	 */
		
 		/* cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			//cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
			cv::drawContours(background, contours, 0, 255, 1);
			color = "green";
		} */
		
		cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(255, 0, 0), 3);
			//cv::drawContours(background, contours, 0, 255, 1);
			color = "blue";
		}
		
		/* cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			//cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 255), 3); 
			cv::drawContours(background, contours, 0, 255, 1);
			color = "yellow";
		}     */
		
	//mask_all = green_mask + blue_mask + red_mask + yellow_mask;
	
	for(int i = 0; i<contours.size(); i++ ) {
	//contours is c and peri is perimeter
	//uses opencv function to calculate the approximate sides
	cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.01, true);	//curve, approxCurve, epsilon, closed //for epsilon need to try different values
	printf("%li\n", sizeof(approx));
	if (sizeof(approx) == 3) { //if 3 sides then it is a triangle 
		printf("Triangle\n");
	}
	
	else if (sizeof(approx) == 4) { //if 4 sides and ar is between 0.95 and 1.05 then it is a square else it is a rectangle
		float x, y;
        cv::Rect rect = cv::boundingRect(approx);
		x = rect.width;
		y = rect.height;
        float ar = x / y;
		if (ar>= 0.95 & ar <= 1.05) {
			printf("Square\n");
		}
		else {
			printf("Rectangle\n");
		}
	}
	
	else if (sizeof(approx) == 12) { //12 sides it is the cross
		printf("Cross\n");
	}
	
	else { //if no sides then a circle
		printf("Circle\n");
	}
	}
	
	
/* 	std::vector< cv::Point2f > corners;
		int maxCorners = 15;
		double qualityLevel = 0.5;
		double minDistance = 10.0;
		int blockSize = 3;
		bool useHarrisDetector = false;
		double k = 0.04;
		corners.reserve(maxCorners);
 		cv::goodFeaturesToTrack(blue_mask, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );
		 for( size_t i = 0; i < corners.size(); i++ ) {
			cv::circle(background, corners[i], 7, cv::Scalar(255, 0, 0), -1 );
		} 
	if (sizeof(corners) == 3) { //if 3 sides then it is a triangle 
		printf("Triangle\n");
	}
	
	else if (sizeof(corners) == 8) { //if 4 sides and ar is between 0.95 and 1.05 then it is a square else it is a rectangle
		printf("Cross\n");
	}
	
	else if (sizeof(corners) == 4)  { //12 sides it is the cross
		printf("Rectangle\n");
	}
	
	else { //if no sides then a circle
		printf("Circle\n");
	} */
		
		
		
/* 		std::vector<cv::Moments> mu(contours.size());
		std::vector<cv::Point> pts;
		std::vector<cv::Point2f> mc(contours.size());
		for( int i = 0; i < contours.size(); i++ ) {
			pts = contours[i];
	
			for( int i = 0; i<contours.size(); i++) {
				mc[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y); 				
			}
			for( int i = 0; i<contours.size(); i++ ) {
				cv::Scalar color = cv::Scalar(128,0,0);
				cv::circle(background, mc[i], 4, color, -1, 8, 0 );
			} 
		}
			
		for( int i = 0; i<contours.size(); i++ ) {
			//const char *res [4];
			if (cv::contourArea(contours[i]) > 100) { //so it doesn't pick up small dots of color value will need to be adjusted
				cv::Rect rect = cv::boundingRect(contours[i]);
				float x, y, width, height, cx, cy;
				width = rect.width;
				height = rect.height;
				x = rect.x;
				y = rect.y;
				cx = (x + width) / 2;
				cy = (y + height) / 2;
				//needs to go through 3 times because needs to pick up 3 colors in order
				for( int i = 0; i<3; i++ ) {
					if (color == "yellow") { //if color is equal to yellow append in array so array would look something like ['Y', 'B', 'Y]
						strncat(res, &yell, 1);
						//strcat("Y", res);
					}
					else if (color == "red") {
						strncat(res, &re, 1);
						//strcat("R", res);
					}
					else if (color == "green") {
						strncat(res, &gree, 1);
						//strcat("G", res);
					}
					else if (color == "blue") {
						strncat(res, &blu, 1);
						//strcat("B", res);
					}
				}
			}
			
		}
		//when array is fully apended if res[0] is whatever then do something if res[1] is whatever then do something 
		//need to somehow make a list of cx of each contour/shape to determine which color is more left which will be a higher x value to order the list by location on the image....arrange colors in order 	
		
		
		if (res[0] == yell) { //if the first color is yellow then docking color is yellow
			docking_color = "Yellow";
			printf("docking color is Yellow\n");
		}
		else if (res[0] == re) { //if the first color is red then docking color is red
			docking_color = "Red";
			printf("docking color is Red\n");	
		}
		else if (res[0] = gree) { //if the first color is green then docking color is green
			docking_color = "Green";
			printf("docking color is Green\n");
		}
		else if (res[0] = blu) { //if the first color is blue then docking color is blue
			docking_color = "Blue";
			printf("docking color is Blue\n");			
		}	
		
		if (res[2] == yell) { //if the third color is yellow then the docking symbol is a rectangle
			docking_symbol = "Rectangle";
			printf("docking symbol is Rectangle\n");
		}
		else if (res[2] == re) { //if the third color is red then the docking symbol is a circle
			docking_symbol = "Circle";
			printf("docking symbol is Circle\n");	
		}
		else if (res[2] = gree) { //if the third color is green then the docking symbol is a triangle
			docking_symbol = "Triangle";
			printf("docking symbol is Triangle\n");
		}
		else if (res[2] = blu) { //if the third color is blue then the docking symbol is a cross
			docking_symbol = "Cross";
			printf("docking symbol is Cross\n");			
		}	 */
		
		cv::imshow("updated", background);  		
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
	
	cv::namedWindow("front_left"); //creates new windows for each camera
	cv::namedWindow("updated", cv::WINDOW_AUTOSIZE);
	
	image_transport::ImageTransport it(nh); //transports the images from the subscriber
	image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, cameraCallBack); //front left camera

	ros::spin();
	cv::destroyWindow("front_left"); //destroys the new windows
	cv::destroyWindow("updated");
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
	
//	detectShape(mask_all); //function created to detect shapes
//	detectLightBuoy(light_buoy); //function detects the light buoy
	
	ros::spinOnce();
	loop_rate.sleep();
	
	}
}
