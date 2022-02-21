#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "ros/ros.h"
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
#include <vector>
#include <string>

using namespace cv;
using namespace std;


//below is the hsv ranges for each color default lighting
cv::Scalar yellow_low = cv::Scalar(16, 154, 65); 
cv::Scalar yellow_high = cv::Scalar(54, 255, 255); 
cv::Scalar green_low = cv::Scalar(59, 115, 25);  
cv::Scalar green_high = cv::Scalar(100, 255, 255); 
cv::Scalar blue_low = cv::Scalar(112, 170, 0);   
cv::Scalar blue_high = cv::Scalar(125, 255, 255);
cv::Scalar red_low = cv::Scalar(130, 170, 60);  
cv::Scalar red_high = cv::Scalar(179, 255, 255); 
cv::Scalar red_low1 = cv::Scalar(0, 185, 54); 
cv::Scalar red_high1 = cv::Scalar(1, 255, 255); 
cv::Scalar orange_low = cv::Scalar(2, 170, 50); 
cv::Scalar orange_high = cv::Scalar(15, 255, 255); 
cv::Scalar white_low = cv::Scalar(0, 0, 80); 
cv::Scalar white_high = cv::Scalar(0, 0, 255); 
cv::Scalar black_low = cv::Scalar(0, 0, 0); 
cv::Scalar black_high = cv::Scalar(179, 0, 35); 

cv::Mat mask;
cv::Mat mask_new;

//mask images for each colour
cv::Mat red_mask; 
cv::Mat red_mask1;
cv::Mat orange_mask;
cv::Mat yellow_mask;
cv::Mat green_mask;
cv::Mat blue_mask;
cv::Mat black_mask;
cv::Mat white_mask;

//used to apply the colored mask to the original image
cv::Mat red_res; 
cv::Mat orange_res;
cv::Mat green_res;
cv::Mat blue_res;
cv::Mat black_res;
cv::Mat white_res;
cv::Mat yellow_res;

//for drawing contrours around the objects
std::vector <std::vector <cv::Point>> contours;
std::vector <std::vector <cv::Point>> contours1;
std::vector <std::vector <cv::Point>> contours2;

cv::Mat org_img;
cv::Mat org_img1;
cv::Mat org_img2;

//height and width of image 
float height;
float width; 

float compactness;
float area;
float perimeter;
float Ry;
float Rx;
			
			
int reg_buoy;
int circle_buoy; 

int color_red = 0;
int color_blue= 0;
int color_green = 0;
int color_orange = 0;
int color_black = 0;
int color_white = 0;
int color_yellow = 0;

int red_x;
int red_y;

string color_type_buoy = " ";
string color_type = " ";

int size_indicies;
float centriodx;
float centriody;

float array_lidar_x [100];
float array_lidar_y [100];
int track;

void LidarCallBackTaylor(const geometry_msgs::Point::ConstPtr& lidar_point) {
	
	size_indicies = lidar_point->z; //remeber this is the amount of clusters from the lidar_subscriber 

	centriodx = lidar_point->x;
	centriody = lidar_point->y;
	printf("%f\n", lidar_point->x);
	printf("%f\n", lidar_point->y);
	printf("%d\n", size_indicies);
	
	//saves to array incoming points in x and y
	array_lidar_x[track] = lidar_point->x;
	array_lidar_y[track] = lidar_point->y;
	track = track+1;
	printf("track is %d\n", track);
	if (track == size_indicies) {
		track = 0;
	}
	//left of here
	
	printf("x0 is: %f\n", array_lidar_x[0]);
	printf("x1 is: %f\n", array_lidar_x[1]);
	printf("y0 is: %f\n", array_lidar_y[0]);
	printf("y1 is: %f\n", array_lidar_y[1]);
	
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
		
		//this is used to also detect the red totem because its Hue value is from 0-1
		cv::inRange(imgHSV, red_low, red_high, red_mask); 
		cv::inRange(imgHSV, red_low1, red_high1, red_mask1); 
		
		//add first red mask with the new red mask
		red_mask = red_mask + red_mask1;
		
		//threshold to find only certain color 
		cv::inRange(imgHSV, blue_low, blue_high, blue_mask); 
		cv::inRange(imgHSV, white_low, white_high, white_mask);
		cv::inRange(imgHSV, orange_low, orange_high, orange_mask); 
		cv::inRange(imgHSV, black_low, black_high, black_mask); 
		cv::inRange(imgHSV, green_low, green_high, green_mask);  
		cv::inRange(imgHSV, yellow_low, yellow_high, yellow_mask);
		
		//using a median filter build in function apply it to each mask with a kernal size of 3 to get rid of small noise
		cv::medianBlur(black_mask, black_mask, 3);
		cv::medianBlur(blue_mask, blue_mask, 3);
		cv::medianBlur(white_mask, white_mask, 3);
		cv::medianBlur(orange_mask, orange_mask, 3);
		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		cv::medianBlur(yellow_mask, yellow_mask, 3);
		
		//add each mask together to create one image with all colors on it which is used for blob detection
		cv::Mat mask_t;
		mask_t = white_mask + green_mask + orange_mask + black_mask + red_mask;
		
		//applies each mask to the original image using bitwise_and and outputs it to a new mat file which isn't useful rn
		cv::bitwise_and(org_img, org_img, mask=red_mask, red_res);
		cv::bitwise_and(org_img, org_img, mask=blue_mask, blue_res);
		cv::bitwise_and(org_img, org_img, mask=mask, white_res); 
		cv::bitwise_and(org_img, org_img, mask=orange_mask, orange_res);
		cv::bitwise_and(org_img, org_img, mask=mask, green_res);
		cv::bitwise_and(org_img, org_img, mask=black_mask, black_res);
		cv::bitwise_and(org_img, org_img, mask=yellow_mask, yellow_res);
		
		
			
//				printf("%f\n", Ry); 
//			printf("Rx is: %f\n", Rx); 
			
//				circle(background, leftmost, 8, (0, 50, 255), -1)
//				circle(background, val.second->x, 8, (0, 255, 255), -1)
//				circle(background, val.first->y, 8, (255, 50, 0), -1)
//				circle(background, val.second->y, 8, (255, 255, 0), -1)
			
			//look up table for compactness and calculate r1 and r2 for each buoy 
				
		//to calculate moments using built in function moments contours must be found
		cv::findContours(mask_t, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		
		//for each findContours it looks at each colored mask and calculates the contrours of the object and then draws the correct colored rectangle around the object
		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ) {
			mu[i] = cv::moments(contours[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			
			std::vector<cv::Point> pts;
			vector<Point2f> mc(contours.size());
			pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			
				red_x = mc[i].x;
				red_y = mc[i].y;
				printf("x centriod of red is: %d\n", red_x);
				printf("y centriod of red is: %d\n", red_y);
			
			
			
			
			area = cv::contourArea(contours[i]);
			perimeter = cv::arcLength(contours[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "red";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					
			}
		}	
		
 		cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
				
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ) {
			mu[i] = cv::moments(contours[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			vector<Point2f> mc(contours.size());
			pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours[i]);
			perimeter = cv::arcLength(contours[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "green";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
			
			}
		}	
		
		/* cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(255, 0, 0), 3);
				
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ) {
			mu[i] = cv::moments(contours[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			vector<Point2f> mc(contours.size());
			pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours[i]);
			perimeter = cv::arcLength(contours[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "blue";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
						circle_buoy = circle_buoy + 1;
						if (circle_buoy >=5) {
							color_type = "mb_round_buoy_";
								color_type_buoy = "blue";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
			
			}
		} */

		cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { //need to add this because with fog the background of trees are white and it draws a big rectangle around the trees
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(255, 255, 255), 3);
					
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ) {
			mu[i] = cv::moments(contours[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			vector<Point2f> mc(contours.size());
			pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours[i]);
			perimeter = cv::arcLength(contours[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "white";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ) {
			mu[i] = cv::moments(contours[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			vector<Point2f> mc(contours.size());
			pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours[i]);
			perimeter = cv::arcLength(contours[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "orange";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ) {
			mu[i] = cv::moments(contours[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			vector<Point2f> mc(contours.size());
			pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours[i]);
			perimeter = cv::arcLength(contours[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "black";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
				
			}
		} 	
		
		/* cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 255), 3); 
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu(contours.size());
		for( int i = 0; i<contours.size(); i++ ) {
			mu[i] = cv::moments(contours[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			vector<Point2f> mc(contours.size());
			pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc[i].x;
				y2 = (val.second -> y) - mc[i].y;
				x1 = mc[i].x - (val.first -> x);
				y1 = mc[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours[i]);
			perimeter = cv::arcLength(contours[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "yellow";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
						circle_buoy = circle_buoy + 1;
						if (circle_buoy >=5) {
							color_type = "mb_round_buoy_";
								color_type_buoy = "yellow";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					
			}
		}     */
			

			//while(ros::ok())
	
		
		cv::imshow("updated", background);  //shows background where the colors are detected 
		cv::imshow("mask_new", mask_t);  //corner detection window
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
		Mat background1;
		Mat f_con1;
		Mat imgHSV1;
		org_img1.copyTo(background1); //copies original image to new image called background
		cv::cvtColor(org_img1, imgHSV1, cv::COLOR_BGR2HSV); //converts image to HSV for color detection
		
		//this is used to also detect the red totem because its Hue value is from 0-1
		cv::inRange(imgHSV1, red_low, red_high, red_mask); 
		cv::inRange(imgHSV1, red_low1, red_high1, red_mask1); 
		
		//add first red mask with the new red mask
		red_mask = red_mask + red_mask1;
		
		//threshold to find only certain color 
		cv::inRange(imgHSV1, blue_low, blue_high, blue_mask); 
		cv::inRange(imgHSV1, white_low, white_high, white_mask);
		cv::inRange(imgHSV1, orange_low, orange_high, orange_mask); 
		cv::inRange(imgHSV1, black_low, black_high, black_mask); 
		cv::inRange(imgHSV1, green_low, green_high, green_mask);  
		cv::inRange(imgHSV1, yellow_low, yellow_high, yellow_mask);
		
		//using a median filter build in function apply it to each mask with a kernal size of 3 to get rid of small noise
		cv::medianBlur(black_mask, black_mask, 3);
		cv::medianBlur(blue_mask, blue_mask, 3);
		cv::medianBlur(white_mask, white_mask, 3);
		cv::medianBlur(orange_mask, orange_mask, 3);
		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		cv::medianBlur(yellow_mask, yellow_mask, 3);
		
		//add each mask together to create one image with all colors on it which is used for blob detection
		cv::Mat mask_t1;
		mask_t1 = white_mask + green_mask + orange_mask + black_mask + red_mask;
		
		//applies each mask to the original image using bitwise_and and outputs it to a new mat file which isn't useful rn
		cv::bitwise_and(org_img1, org_img1, mask=red_mask, red_res);
		cv::bitwise_and(org_img1, org_img1, mask=blue_mask, blue_res);
		cv::bitwise_and(org_img1, org_img1, mask=mask, white_res); 
		cv::bitwise_and(org_img1, org_img1, mask=orange_mask, orange_res);
		cv::bitwise_and(org_img1, org_img1, mask=mask, green_res);
		cv::bitwise_and(org_img1, org_img1, mask=black_mask, black_res);
		cv::bitwise_and(org_img1, org_img1, mask=yellow_mask, yellow_res);
		
		
			
//				printf("%f\n", Ry); 
//			printf("Rx is: %f\n", Rx); 
			
//				circle(background, leftmost, 8, (0, 50, 255), -1)
//				circle(background, val.second->x, 8, (0, 255, 255), -1)
//				circle(background, val.first->y, 8, (255, 50, 0), -1)
//				circle(background, val.second->y, 8, (255, 255, 0), -1)
			
			//look up table for compactness and calculate r1 and r2 for each buoy 
				
		//to calculate moments using built in function moments contours must be found
		cv::findContours(mask_t1, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			
		//for each findContours it looks at each colored mask and calculates the contrours of the object and then draws the correct colored rectangle around the object
		cv::findContours(red_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu1(contours1.size());
		for( int i = 0; i<contours1.size(); i++ ) {
			mu1[i] = cv::moments(contours1[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			vector<Point2f> mc1(contours1.size());
			std::vector<cv::Point> pts;
			//vector<Point2f> mc1(contours1.size());
			pts = contours1[i];
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc1[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc1[i].x;
				y2 = (val.second -> y) - mc1[i].y;
				x1 = mc1[i].x - (val.first -> x);
				y1 = mc1[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours1[i]);
			perimeter = cv::arcLength(contours1[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "red";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							reg_buoy = 0;
							color_type_buoy = " ";
							color_type = " ";
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							color_type = " ";
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					
			}
		}	
		
 		cv::findContours(green_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
				
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu1(contours1.size());
		for( int i = 0; i<contours1.size(); i++ ) {
			mu1[i] = cv::moments(contours1[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			vector<Point2f> mc1(contours1.size());
			std::vector<cv::Point> pts;
			//vector<Point2f> mc1(contours1.size());
			pts = contours1[i];
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc1[i].x;
				y2 = (val.second -> y) - mc1[i].y;
				x1 = mc1[i].x - (val.first -> x);
				y1 = mc1[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours1[i]);
			perimeter = cv::arcLength(contours1[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "green";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							reg_buoy = 0;
							color_type_buoy = " ";
							color_type = " ";
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							color_type = " ";
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
			
			}
		}	
		
		/* cv::findContours(blue_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(255, 0, 0), 3);
				
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu1(contours1.size());
		for( int i = 0; i<contours1.size(); i++ ) {
			mu1[i] = cv::moments(contours1[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			//vector<Point2f> mc1(contours1.size());
			pts = contours1[i];
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				right_blob_cnt = right_blob_cnt + 1;
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc1[i].x;
				y2 = (val.second -> y) - mc1[i].y;
				x1 = mc1[i].x - (val.first -> x);
				y1 = mc1[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours1[i]);
			perimeter = cv::arcLength(contours1[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "blue";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
									DisparityFunc();
							reg_buoy = 0;
							color_type_buoy = " ";
							color_type = " ";
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
						circle_buoy = circle_buoy + 1;
						if (circle_buoy >=5) {
							color_type = "mb_round_buoy_";
								color_type_buoy = "blue";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
									DisparityFunc();
							color_type = " ";
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
			
			}
		} */

		cv::findContours(white_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { //need to add this because with fog the background of trees are white and it draws a big rectangle around the trees
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(255, 255, 255), 3);
					
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu1(contours1.size());
		for( int i = 0; i<contours1.size(); i++ ) {
			mu1[i] = cv::moments(contours1[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			vector<Point2f> mc1(contours1.size());
			std::vector<cv::Point> pts;
			//vector<Point2f> mc1(contours1.size());
			pts = contours1[i];
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc1[i].x;
				y2 = (val.second -> y) - mc1[i].y;
				x1 = mc1[i].x - (val.first -> x);
				y1 = mc1[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours1[i]);
			perimeter = cv::arcLength(contours1[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "white";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							reg_buoy = 0;
							color_type_buoy = " ";
							color_type = " ";
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							color_type = " ";
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
			
			}
		} 	
		
		cv::findContours(orange_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 179, 255), 3);
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu1(contours1.size());
		for( int i = 0; i<contours1.size(); i++ ) {
			mu1[i] = cv::moments(contours1[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			vector<Point2f> mc1(contours1.size());
			std::vector<cv::Point> pts;
			//vector<Point2f> mc1(contours1.size());
			pts = contours1[i];
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc1[i].x;
				y2 = (val.second -> y) - mc1[i].y;
				x1 = mc1[i].x - (val.first -> x);
				y1 = mc1[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours1[i]);
			perimeter = cv::arcLength(contours1[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "orange";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							reg_buoy = 0;
							color_type_buoy = " ";
							color_type = " ";
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							color_type = " ";
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
			
			}
		}
		
	 	cv::findContours(black_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 0), 3); 
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu1(contours1.size());
		for( int i = 0; i<contours1.size(); i++ ) {
			mu1[i] = cv::moments(contours1[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			vector<Point2f> mc1(contours1.size());
			std::vector<cv::Point> pts;
			//vector<Point2f> mc1(contours1.size());
			pts = contours1[i];
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc1[i].x;
				y2 = (val.second -> y) - mc1[i].y;
				x1 = mc1[i].x - (val.first -> x);
				y1 = mc1[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours1[i]);
			perimeter = cv::arcLength(contours1[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "black";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
							reg_buoy = 0;
							color_type_buoy = " ";
							color_type = " ";
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
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
							color_type = " ";
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
				
			}
		} 	
		
		/* cv::findContours(yellow_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 255), 3); 
			
			float x1;
			float x2;
			float y1;
			float y2;
			
			//this calculates the moments of the contours but these change depeding on scaling
		vector<Moments> mu1(contours1.size());
		for( int i = 0; i<contours1.size(); i++ ) {
			mu1[i] = cv::moments(contours1[i], false);
	//		printf("moments: %f %f %f %f %f %f %f %f %f %f\n", mu[i].m00, mu[i].m10, mu[i].m01, mu[i].m20, mu[i].m11, mu[i].m02, mu[i].m30,mu[i].m21, mu[i].m12, mu[i].m03);
		}
			
			std::vector<cv::Point> pts;
			//vector<Point2f> mc1(contours1.size());
			pts = contours1[i];
				mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);   
//				printf("the x centriod is: %f\n", mc[i].x);
//				printf("the y centriod is: %f\n", mc[i].y);
				Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
				right_blob_cnt = right_blob_cnt + 1;
				
				auto val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.x < b.x;
					});
//				leftmost.x = val.first->x;
//				leftmost.y = val.first->y;
//				printf("leftMost x is: %d\n", val.first->x); 
//				printf("rightMost x is: %d\n", val.second->x); 

				val = std::minmax_element(pts.begin(), pts.end(), [](Point const& a, Point const& b){
					return a.y < b.y;
				});
				
//				printf("topMost y is: %d\n", val.first->y); 
//				printf("bottomMost y is: %d\n", val.second->y); 
			
				x2 = (val.second -> x) - mc1[i].x;
				y2 = (val.second -> y) - mc1[i].y;
				x1 = mc1[i].x - (val.first -> x);
				y1 = mc1[i].y - (val.first -> y);
			
				Ry = y1 / y2;
				Rx = x1 / x2;
			
			
			area = cv::contourArea(contours1[i]);
			perimeter = cv::arcLength(contours1[i], true);
//					printf("the area is: %f\n", area); 
//					printf("the arc length is: %f\n", perimeter);
					compactness = (area) / (pow(perimeter, 2));
//					printf("%f\n", compactness);
						if ((compactness >= 0.01942425 & compactness <= 0.03975825) && (Ry >= 1.50754025 & Ry <= 1.74948625)) {
						reg_buoy = reg_buoy + 1;	
						if (reg_buoy >=5) {
							color_type = "mb_marker_buoy_";
								color_type_buoy = "yellow";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
									DisparityFunc();
							reg_buoy = 0;
							color_type_buoy = " ";
							color_type = " ";
							//printf("regular buoy\n");
							//cv::putText(background, "Cone Buoy ",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					if ((compactness >= 0.065152 & compactness <= 0.071328)  && (Ry >= 1.016995375 & Ry <= 1.342738375)) { //could do or but picks up totem
						circle_buoy = circle_buoy + 1;
						if (circle_buoy >=5) {
							color_type = "mb_round_buoy_";
								color_type_buoy = "yellow";
								color_type_buoy.insert(0, color_type);
								printf("the buoy identification string is: %s\n", color_type_buoy.c_str());
									DisparityFunc();
							color_type = " ";
							circle_buoy = 0;
							color_type_buoy = " ";
							//printf("circle buoy\n");
							//cv::putText(background, "Circle Buoy",(boundRect.br(), boundRect.tl()), 2, 2, (255, 0, 0));
							break;
						}
					}
					
			}
		}     */
			

			//while(ros::ok())
	
		
		cv::imshow("updated1", background1);  //shows background where the colors are detected 
		//cv::imshow("mask_new", mask_t);  //corner detection window
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
	
	cv::namedWindow("front_left"); //creates new windows for each camera
	cv::namedWindow("updated", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("front_right"); //creates new windows for each camera
	cv::namedWindow("updated1", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("mask_t", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("mask_new", cv::WINDOW_AUTOSIZE);
	
	image_transport::ImageTransport it(nh); //transports the images from the subscriber
	image_transport::ImageTransport it1(nh1);
	
	image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, cameraCallBack); //front left camera
	
	image_transport::Subscriber camera_sub1 = it1.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 1, cameraCallBack1); //front rightcamera
	
	ros::Subscriber lidar_sub = nh2.subscribe("lidar_point", 100, LidarCallBackTaylor); //subscribes to my lidar points


	ros::spin();
	cv::destroyWindow("front_left"); //destroys the new windows
	cv::destroyWindow("front_right"); //destroys the new windows
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()) {
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
