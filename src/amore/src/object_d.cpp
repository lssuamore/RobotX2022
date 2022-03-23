#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "iostream"
#include "stdio.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/Point.h"
#include "amore/NED_waypoints.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "time.h"
#include <vector>
#include <string>

using namespace cv;
using namespace std;

geographic_msgs::GeoPoseStamped task3_message; //publisher message type
ros::Publisher task3_pub; //publisher for judges topic
ros::Time current_time, last_time;  // creates time variables

//below is the hsv ranges for each color default lighting
cv::Scalar green_low; 
cv::Scalar green_high; 
cv::Scalar red_low;  
cv::Scalar red_high; 
cv::Scalar red_low1 = cv::Scalar(0, 185, 54); 
cv::Scalar red_high1 = cv::Scalar(1, 255, 255); 
cv::Scalar orange_low; 
cv::Scalar orange_high; 
cv::Scalar white_low;
cv::Scalar white_high;
cv::Scalar black_low; 
cv::Scalar black_high;

cv::Scalar fog_background_low;
cv::Scalar fog_background_high;
cv::Scalar fog_background1_low; 
cv::Scalar fog_background1_high;
cv::Scalar am_background_low;
cv::Scalar am_background_high;

cv::Mat mask;
cv::Mat mask_new;

//mask images for each colour
cv::Mat red_mask; 
cv::Mat red_mask1;
cv::Mat orange_mask;
cv::Mat green_mask;
cv::Mat black_mask;
cv::Mat white_mask;

cv::Mat fog_background_mask;
cv::Mat fog_background1_mask;
cv::Mat am_background_mask;

//used to apply the colored mask to the original image
cv::Mat red_res; 
cv::Mat orange_res;
cv::Mat green_res;
cv::Mat black_res;
cv::Mat white_res;

//for drawing contrours around the objects
std::vector <std::vector <cv::Point>> contours;
std::vector <std::vector <cv::Point>> contours1;
std::vector <std::vector <cv::Point>> contours2;

//arrays for x and y pixel centriod
int color_x[100];
int color_y[100];

//original images from cameras
cv::Mat org_img;
cv::Mat org_img1;
cv::Mat org_img2;

//height and width of image 
float height;
float width; 

//buoy classification
float compactness;
float area;
float perimeter;
float Ry;
float Rx;
			
			
int reg_buoy;
int circle_buoy; 

int color_red = 0;
int color_green = 0;
int color_orange = 0;
int color_black = 0;
int color_white = 0;

int red_x;
int red_y;
int green_x;
int green_y;
int black_x;
int black_y;
int white_x;
int white_y;
int orange_x;
int orange_y;

string color_type_buoy = " ";
string color_type = " ";

int size_indicies;
float centriodx;
float centriody;

//lidar data array
float array_lidar_x [100];
float array_lidar_y [100];
int track;
int camera_track;

//distances in x and y for each color
float xdistance_red;
float xdistance_green;
float ydistance_red;
float ydistance_green;
float ydistance_orange;
float ydistance_black;
float xdistance_orange;
float xdistance_black;
float xdistance_white;
float ydistance_white;

int size_red = 0;
int size_green = 0;
int size_orange = 0;
int size_black = 0;
int size_white = 0;
int size_mask_t = 0;



void HSV_change(cv::Mat imgHSV){
	
		green_low = cv::Scalar(59, 115, 25);  
		green_high = cv::Scalar(100, 255, 255); 
		red_low = cv::Scalar(130, 170, 60);  
		red_high = cv::Scalar(179, 255, 255); 
		red_low1 = cv::Scalar(0, 185, 54); 
		red_high1 = cv::Scalar(1, 255, 255); 
		orange_low = cv::Scalar(2, 170, 50); 
		orange_high = cv::Scalar(15, 255, 255); 
		white_low = cv::Scalar(0, 0, 80); 
		white_high = cv::Scalar(0, 0, 255); 
		black_low = cv::Scalar(0, 0, 0); 
		black_high = cv::Scalar(179, 0, 35); 
	
	
		fog_background_low = cv::Scalar(0, 0, 200); 
		fog_background_high = cv::Scalar(0, 0, 240); 
		fog_background1_low = cv::Scalar(0, 0, 160); 
		fog_background1_high = cv::Scalar(0, 0, 240); 
		/* am_background_low = cv::Scalar(32, 130, 0); 
		am_background_high = cv::Scalar(33, 255, 255);  */
		
		cv::inRange(imgHSV, fog_background_low, fog_background_high, fog_background_mask); 
		cv::findContours(fog_background_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		if (contours.size() >0) { //change ranges
			green_low = cv::Scalar(59, 25, 0);  
			green_high = cv::Scalar(100, 255, 255); 
			red_low = cv::Scalar(130, 65, 0);  
			red_high = cv::Scalar(179, 255, 255);  
			orange_low = cv::Scalar(2, 0, 0); 
			orange_high = cv::Scalar(15, 255, 255); 
			white_low = cv::Scalar(0, 0, 95); 
			white_high = cv::Scalar(0, 0, 202); 
			black_low = cv::Scalar(0, 0, 0); 
			black_high = cv::Scalar(179, 0, 125);
		}
		
		cv::inRange(imgHSV, fog_background1_low, fog_background1_high, fog_background1_mask); 
		cv::findContours(fog_background1_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		if (contours.size() >0) { //change ranges
			green_low = cv::Scalar(59, 50, 0);  
			green_high = cv::Scalar(100, 255, 255); 
			red_low = cv::Scalar(130, 60, 0);  
			red_high = cv::Scalar(179, 255, 255); 
			orange_low = cv::Scalar(2, 0, 0); 
			orange_high = cv::Scalar(15, 255, 255); 
			white_low = cv::Scalar(0, 0, 100); 
			white_high = cv::Scalar(0, 0, 150); 
			black_low = cv::Scalar(0, 0, 0); 
			black_high = cv::Scalar(179, 0, 100);
		}
		
	/* 	cv::inRange(imgHSV, am_background_low, am_background_high, am_background_mask); 
		cv::findContours(am_background_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		if (contours.size() >0) { //change ranges
			green_low = cv::Scalar(59, 178, 75);  
			green_high = cv::Scalar(100, 255, 255); 
			red_low = cv::Scalar(130, 130, 75);  
			red_high = cv::Scalar(179, 255, 255);  
			orange_low = cv::Scalar(2, 40, 200); 
			orange_high = cv::Scalar(15, 255, 255); 
			white_low = cv::Scalar(0, 0, 200); 
			white_high = cv::Scalar(0, 0, 255); 
			black_low = cv::Scalar(0, 0, 0); 
			black_high = cv::Scalar(179, 0, 170);
		} */
		
			
	
/* 		cv::inRange(imgHSV, red_low, red_high, red_mask); 
		cv::inRange(imgHSV, red_low1, red_high1, red_mask1); 
		
		//add first red mask with the new red mask
		red_mask = red_mask + red_mask1;
		
		//threshold to find only certain color 
		cv::inRange(imgHSV, orange_low, orange_high, orange_mask); 
		cv::inRange(imgHSV, green_low, green_high, green_mask);  
		cv::inRange(imgHSV, white_low, white_high, white_mask);
		cv::inRange(imgHSV, black_low, black_high, black_mask);
		
		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		cv::findContours(orange_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object */
	
	
}


//subscribes to lidar points
void LidarCallBackTaylor(const amore::NED_waypoints::ConstPtr& lidar_point) {
	
	size_indicies = lidar_point->quantity; //remeber this is the amount of clusters from the lidar_subscriber 
	
	for (size_t i = 0; i < size_indicies; i++){
	//saves to array incoming points in x and y
		array_lidar_x[i] = lidar_point->points[i].x;
		array_lidar_y[i] = lidar_point->points[i].y;
//		printf("x coming in is: %f\n", array_lidar_x[i]);
//		printf("x coming in is: %f\n", array_lidar_y[i]);
	}
	
	std::sort(array_lidar_x,array_lidar_x+size_indicies); //sorts smallest to biggest //how to sort array of points
	std::sort(array_lidar_y,array_lidar_y+size_indicies); 
	
	//for (size_t i = 0; i < size_indicies; i++){
	//printd array of incoming points in x and y sorted
//		printf("x coming in array sorted array is: %f\n", array_lidar_x[i]);
//		printf("x coming in array sorted array is: %f\n", array_lidar_y[i]);
	//}
}


void cameraCallBack(const sensor_msgs::ImageConstPtr& camera_msg) {
	try 
	{
		org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		cv::imshow("front_left", org_img); //shows the image in front left window
		Mat background;
		Mat f_con;
		Mat imgHSV;
		org_img.copyTo(background); //copies original image to new image called background
		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV); //converts image to HSV for color detection
		
		
		HSV_change(imgHSV);
		
		
		//this is used to also detect the red totem because its Hue value is from 0-1
		cv::inRange(imgHSV, red_low, red_high, red_mask); 
		cv::inRange(imgHSV, red_low1, red_high1, red_mask1); 
		
		//add first red mask with the new red mask
		red_mask = red_mask + red_mask1;
		
		//threshold to find only certain color 
		cv::inRange(imgHSV, white_low, white_high, white_mask);
		cv::inRange(imgHSV, orange_low, orange_high, orange_mask); 
		cv::inRange(imgHSV, black_low, black_high, black_mask); 
		cv::inRange(imgHSV, green_low, green_high, green_mask);  
		
		//using a median filter build in function apply it to each mask with a kernal size of 3 to get rid of small noise
		cv::medianBlur(black_mask, black_mask, 3);
		cv::medianBlur(white_mask, white_mask, 3);
		cv::medianBlur(orange_mask, orange_mask, 3);
		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		
		//add each mask together to create one image with all colors on it which is used for blob detection
		cv::Mat mask_t;
		mask_t = white_mask + green_mask + orange_mask + black_mask + red_mask;
		
		//applies each mask to the original image using bitwise_and and outputs it to a new mat file which isn't useful rn
		cv::bitwise_and(org_img, org_img, mask=red_mask, red_res);
		cv::bitwise_and(org_img, org_img, mask=white_mask, white_res); 
		cv::bitwise_and(org_img, org_img, mask=orange_mask, orange_res);
		cv::bitwise_and(org_img, org_img, mask=green_mask, green_res);
		cv::bitwise_and(org_img, org_img, mask=black_mask, black_res);
				
		//to calculate moments using built in function moments contours must be found
		cv::findContours(mask_t, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
				size_mask_t = contours.size();
		
				vector<Moments> mu(contours.size());
				for( int i = 0; i<contours.size(); i++ ) {
					mu[i] = cv::moments(contours[i], false);
				}
			
				//finds contours of all colored buoys
				std::vector<cv::Point> pts;
				vector<Point2f> mc(contours.size());
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
				//saves to array x centriod and y centriod
				color_x[i] = mc[i].x;
				color_y[i] = mc[i].y;
			}
		}
		
		std::sort(color_x, color_x+size_mask_t); //sorts smallest to biggest
		std::sort(color_y, color_y+size_mask_t); 
		
		//prints the sorted arrays
		/* 	for (size_t i = 0; i < sizeof(color_x); i++){
			printf("x color_x is  %d\n", color_x[i]);
			}
				for (size_t i = 0; i < sizeof(color_y); i++){
			printf("y color_y is  %d\n", color_y[i]);
			} */
		
		
		//for each findContours it looks at each colored mask and calculates the contrours of the object and then draws the correct colored rectangle around the object
		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			
				cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
				cv::Rect boundRect = cv::boundingRect(contours[i]);
				size_red = contours.size();
			
				float x1;
				float x2;
				float y1;
				float y2;
			
				//this calculates the moments of the contours but these change depeding on scaling
				vector<Moments> mu(contours.size());
				for( int i = 0; i<contours.size(); i++ ) {
					mu[i] = cv::moments(contours[i], false);
				}
			
			
				std::vector<cv::Point> pts;
				vector<Point2f> mc(contours.size());
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				//finds extreme points of buoys
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
				});

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
				//buoy classification Ry and Rx
				Ry = y1 / y2;
				Rx = x1 / x2;
			
				red_x = mc[i].x;
				red_y = mc[i].y;
//				printf("x centriod of red is: %d\n", red_x);
//				printf("y centriod of red is: %d\n", red_y);
				//finds color centriod in the array of color_x
				for(int i = 0; i < sizeof(color_x); i++) {
					if(color_x[i] == red_x){
						cout << "Red x element found at index " << i <<"\n";
						ydistance_red = array_lidar_y[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Red buoy y is: %f\n", ydistance_red);
					break;
					}
				 }
				 //finds color centriod in the array of color_y
				 for(int i = 0; i < sizeof(color_y); i++) {
					if(color_y[i] == red_y){
						cout << "Red y element found at index " << i <<"\n";
						xdistance_red = array_lidar_x[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Red buoy x is: %f\n", xdistance_red);
					break;
					}
				 }
			
				area = cv::contourArea(contours[i]);
				perimeter = cv::arcLength(contours[i], true);
				compactness = (area) / (pow(perimeter, 2));
				if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
					reg_buoy = reg_buoy + 1;	
					if (reg_buoy >=5) {
						color_type = "mb_marker_buoy_";
						color_type_buoy = "red";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_red;
						task3_message.pose.position.longitude = xdistance_red;
						task3_pub.publish(task3_message); 
						reg_buoy = 0;
						color_type_buoy = " ";
						break;
					}
				}
				if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
					circle_buoy = circle_buoy + 1;
					if (circle_buoy >=5) {
						color_type = "mb_round_buoy_";
						color_type_buoy = "red";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_red;
						task3_message.pose.position.longitude = xdistance_red;
						task3_pub.publish(task3_message); 
						circle_buoy = 0;
						color_type_buoy = " ";
						break;
					}
				}	
			}
		}	
		
 		cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
				cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
				size_green = contours.size();	
				
				float x1;
				float x2;
				float y1;
				float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
				vector<Moments> mu(contours.size());
				for( int i = 0; i<contours.size(); i++ ) {
					mu[i] = cv::moments(contours[i], false);
				}
			
				std::vector<cv::Point> pts;
				vector<Point2f> mc(contours.size());
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
				});

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;		
				
				green_x = mc[i].x;
				green_y = mc[i].y;
//				printf("x centriod of green is: %d\n", green_x);
//				printf("y centriod of green is: %d\n", green_y);
				
				for(int i = 0; i < sizeof(color_x); i++) {
					if(color_x[i] == green_x){
						cout << "Green x element found at index " << i <<"\n";
						ydistance_green = array_lidar_y[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Green buoy y is: %f\n", ydistance_green);
					break;
					}
				 }
				 
				 for(int i = 0; i < sizeof(color_y); i++) {
					if(color_y[i] == green_y){
						cout << "Green y element found at index " << i <<"\n";
						xdistance_green = array_lidar_x[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Green buoy x is: %f\n", xdistance_green);
					break;
					}
				 }
			
				area = cv::contourArea(contours[i]);
				perimeter = cv::arcLength(contours[i], true);
				compactness = (area) / (pow(perimeter, 2));
				if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
					reg_buoy = reg_buoy + 1;	
					if (reg_buoy >=5) {
						color_type = "mb_marker_buoy_";
						color_type_buoy = "green";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_green;
						task3_message.pose.position.longitude = xdistance_green;
						task3_pub.publish(task3_message); 
						reg_buoy = 0;
						color_type_buoy = " ";
					break;
					}
				}
				if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
					circle_buoy = circle_buoy + 1;
					if (circle_buoy >=5) {
						color_type = "mb_round_buoy_";
						color_type_buoy = "green";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_green;
						task3_message.pose.position.longitude = xdistance_green;
						task3_pub.publish(task3_message); 
						circle_buoy = 0;
						color_type_buoy = " ";
					break;
					}
				}
			}
		}	

		cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { //need to add this because with fog the background of trees are white and it draws a big rectangle around the trees
				cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(255, 255, 255), 3);
				size_white = contours.size();
					
				float x1;
				float x2;
				float y1;
				float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
				vector<Moments> mu(contours.size());
				for( int i = 0; i<contours.size(); i++ ) {
					mu[i] = cv::moments(contours[i], false);
				}
			
				std::vector<cv::Point> pts;
				vector<Point2f> mc(contours.size());
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
				});

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
				white_x = mc[i].x;
				white_y = mc[i].y;
//				printf("x centriod of white is: %d\n", white_x);
//				printf("y centriod of white is: %d\n", white_y);
			
				for(int i = 0; i < sizeof(color_x); i++) {
					if(color_x[i] == white_x){
						cout << "White x element found at index " << i <<"\n";
						ydistance_white = array_lidar_y[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("White buoy y is: %f\n", ydistance_white);
					break;
					}
				 }
				 
				 for(int i = 0; i < sizeof(color_y); i++) {
					if(color_y[i] == white_y){
						cout << "White y element found at index " << i <<"\n";
						xdistance_white = array_lidar_x[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("White buoy x is: %f\n", xdistance_white);
					break;
					}
				 }
			
				area = cv::contourArea(contours[i]);
				perimeter = cv::arcLength(contours[i], true);
				compactness = (area) / (pow(perimeter, 2));
				if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
					reg_buoy = reg_buoy + 1;	
					if (reg_buoy >=5) {
						color_type = "mb_marker_buoy_";
						color_type_buoy = "white";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_white;
						task3_message.pose.position.longitude = xdistance_white;
						task3_pub.publish(task3_message); 
						reg_buoy = 0;
						color_type_buoy = " ";
						break;
					}
				}
				if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
					circle_buoy = circle_buoy + 1;
					if (circle_buoy >=5) {
						color_type = "mb_round_buoy_";
						color_type_buoy = "white";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_white;
						task3_message.pose.position.longitude = xdistance_white;
						task3_pub.publish(task3_message); 
						circle_buoy = 0;
						color_type_buoy = " ";
							break;
					}
				}
			}
		} 	
		
		cv::findContours(orange_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
				cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 179, 255), 3);
				size_orange = contours.size();
			
				float x1;
				float x2;
				float y1;
				float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
				vector<Moments> mu(contours.size());
				for( int i = 0; i<contours.size(); i++ ) {
					mu[i] = cv::moments(contours[i], false);
				}
			
				std::vector<cv::Point> pts;
				vector<Point2f> mc(contours.size());
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
				});
					
				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
				
				orange_x = mc[i].x;
				orange_y = mc[i].y;
//				printf("x centriod of orange is: %d\n", orange_x);
//				printf("y centriod of orange is: %d\n", orange_y);
				
				for(int i = 0; i < sizeof(color_x); i++) {
					if(color_x[i] == orange_x){
						cout << "Orange x element found at index " << i <<"\n";
						ydistance_orange = array_lidar_y[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Orange buoy y is: %f\n", ydistance_orange);
					break;
					}
				 }
				 
				 for(int i = 0; i < sizeof(color_y); i++) {
					if(color_y[i] == orange_y){
						cout << "Orange y element found at index " << i <<"\n";
						xdistance_orange = array_lidar_x[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Orange buoy x is: %f\n", xdistance_orange);
					break;
					}
				 }
			
				area = cv::contourArea(contours[i]);
				perimeter = cv::arcLength(contours[i], true);
				compactness = (area) / (pow(perimeter, 2));
				if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
					reg_buoy = reg_buoy + 1;	
					if (reg_buoy >=5) {
						color_type = "mb_marker_buoy_";
						color_type_buoy = "orange";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_orange;
						task3_message.pose.position.longitude = xdistance_orange;
						task3_pub.publish(task3_message); 
						reg_buoy = 0;
						color_type_buoy = " ";
					break;
					}
				}
				if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
					circle_buoy = circle_buoy + 1;
					if (circle_buoy >=5) {
						color_type = "mb_round_buoy_";
						color_type_buoy = "orange";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_orange;
						task3_message.pose.position.longitude = xdistance_orange;
						task3_pub.publish(task3_message); 
						circle_buoy = 0;
						color_type_buoy = " ";
					break;
					}
				}
			}
		}
		
	 	cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
				cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 0), 3); 
				size_black = contours.size();
			
				float x1;
				float x2;
				float y1;
				float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
				vector<Moments> mu(contours.size());
				for( int i = 0; i<contours.size(); i++ ) {
					mu[i] = cv::moments(contours[i], false);
				}
			
				std::vector<cv::Point> pts;
				vector<Point2f> mc(contours.size());
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
				});

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
				
				black_x = mc[i].x;
				black_y = mc[i].y;
//				printf("x centriod of black is: %d\n", black_x);
//				printf("y centriod of black is: %d\n", black_y);
				
				for(int i = 0; i < sizeof(color_x); i++) {
					if(color_x[i] == black_x){
						cout << "Black x element found at index " << i <<"\n";
						ydistance_black = array_lidar_y[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Black buoy y is: %f\n", ydistance_black);
					break;
					}
				 }
				 
				 for(int i = 0; i < sizeof(color_y); i++) {
					if(color_y[i] == black_y){
						cout << "Black y element found at index " << i <<"\n";
						xdistance_black = array_lidar_x[(size_indicies-1)-i]; //lidar distance coordinates so y is left to right
						printf("Black x is: %f\n", xdistance_black);
					break;
					}
				 }
			
				area = cv::contourArea(contours[i]);
				perimeter = cv::arcLength(contours[i], true);
				compactness = (area) / (pow(perimeter, 2));
				if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
					reg_buoy = reg_buoy + 1;	
					if (reg_buoy >=5) {
						color_type = "mb_marker_buoy_";
						color_type_buoy = "black";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_black;
						task3_message.pose.position.longitude = xdistance_black;
						task3_pub.publish(task3_message); 
						reg_buoy = 0;
						color_type_buoy = " ";
						break;
					}
				}
				if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
					circle_buoy = circle_buoy + 1;
					if (circle_buoy >=5) {
						color_type = "mb_round_buoy_";
						color_type_buoy = "black";
						color_type_buoy.insert(0, color_type);
						printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
						current_time = ros::Time::now(); //time
						task3_message.header.seq +=1;								// sequence number
						task3_message.header.stamp = current_time;				// sets stamp to current time
						task3_message.header.frame_id = color_type_buoy.c_str(); //header frame
						task3_message.pose.position.latitude = ydistance_black;
						task3_message.pose.position.longitude = xdistance_black;
						task3_pub.publish(task3_message); 
						circle_buoy = 0;
						color_type_buoy = " ";
						break;
					}
				}
			}
		} 	
		
		cv::imshow("updated", background);  //shows background where the colors are detected 
//		cv::imshow("mask_new", mask_t);  //corner detection window
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
	ros::NodeHandle nh2;
	
	ros::NodeHandle nh3;
	
	cv::namedWindow("front_left"); //creates new windows for each camera
	cv::namedWindow("updated", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("mask_t", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("mask_new", cv::WINDOW_AUTOSIZE);
	
	image_transport::ImageTransport it(nh); //transports the images from the subscriber
	
	//image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/middle_camera/image_raw", 1, cameraCallBack); //front left camera
	image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, cameraCallBack); //front left camera
	
	ros::Subscriber lidar_sub = nh2.subscribe("lidar_point", 10, LidarCallBackTaylor); //subscribes to my lidar points

	task3_pub = nh3.advertise<geographic_msgs::GeoPoseStamped>("/vrx/perception/landmark", 10); 
	
	ros::Time::init();
	current_time = ros::Time::now();   // sets current time to the time it is now
	last_time = ros::Time::now();        // sets last time to the time it is now

	ros::spin();
	cv::destroyWindow("front_left"); //destroys the new windows
	
	ros::Rate loop_rate(100);
	
	while(ros::ok()) {
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
