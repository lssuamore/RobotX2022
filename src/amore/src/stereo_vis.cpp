// Filename:								stereo_vis.cpp
// Creation Date:						12/04/2021
// Last Revision Date:
// Author(s) [email]:					Shaede Perzanowski [sperzanowski1@lssu.edu]
//												Brad Hacker [bhacker@lssu.edu]
// Revisor(s) [Revision Date]:
// Organization/Institution:			Lake Superior State University

//...................................................About stereo_vis.cpp.....................................................................
// The stereo_vis.cpp file is used to access image data from the on-board camera sensors of the USV. 
// The program uses OpenCV to detect a red and a green buoy, and then it uses perspective geometry
// to find the location of each buoy with respect to the USV.

// Inputs and Outputs of the stereo_vis.cpp file
//				Inputs: Camera sensor data from forward facing cameras.
//				Outputs: Buoy locations (published data) and classifications (visual display)


//................................................Included Libraries and Message Types..........................................
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
#include "nav_msgs/Odometry.h"
#include "time.h"
#include <vector>
#include <string>
#include "std_msgs/Float32.h"
//...........................................End of Included Libraries and Message Types....................................

//..............................................................Namespaces................................................................
using namespace cv;
using namespace std;
//........................................................End of Namespaces...........................................................

//.................................................................Constants..................................................................
#define BASELINE 0.2				// baseline between camera sensors [m]
#define Umax 1279.0					// maximum pixel index for u (a pictures's x)
#define Umin 0.0						// minimum pixel index for u (a pictures's x)
#define PHI -33.724223				// latitude of ENU origin
#define LAMBDA 150.679736		// longitude of ENU origin
#define R11 -0.489690849			//_	The rotation matrix from ENU to ECEF
#define	R12 -0.484073338			//  |
#define	R13 -0.725172997			//	|
#define	R21 -0.871896136			//	|
#define	R22 0.271874452			//	|
#define	R23 0.407285416			//	|		
#define	R31 0.0							//	|
#define	R32 0.831719476			//	|
#define	R33 -0.555196104			//_|
#define PI 3.14159265
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
//below is the hsv ranges for each color default lighting

geographic_msgs::GeoPoseStamped task3_message; //publisher message type
ros::Publisher task3_pub; //publisher for judges topic
ros::Time current_time, last_time;  // creates time variables

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
cv::Mat mask_new1;

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

// Two vectors to hold the centroids of the BLOBs
vector<Point2f> MC(10); // For left image
vector<Point2f> MC1(10); // For right image
int left_blob_cnt = 0; // Holds the count of BLOBs from left camera
int right_blob_cnt = 0; // Holds the count of BLOBs from left camera

// Look-up-table for finding accurate lateral translation of buoy from the USV
float lat_scale_LUT[10][9] = {												  // 7m	8m	9m	10m	11m	12m	13m	14m	15m
	{1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1, 1.000, 1.000}, // 0m
	{1.838, 1.610, 1.439, 1.290, 1.176, 1.075, 1, 0.930, 0.735}, // 1m		columns are forward longitudinal translation
	{0.947, 0.826, 0.727, 0.658, 0.597, 0.543, 1, 0.468, 0.431}, // 2m
	{0.624, 0.544, 0.489, 0.438, 0.397, 0.365, 1, 0.313, 0.287}, // 3m		rows are lateral translation, either direction
	{0.472, 0.410, 0.363, 0.325, 0.295, 0.273, 1, 0.233, 0.218}, // 4m
	{1.000, 0.331, 0.291, 0.261, 0.236, 0.214, 1, 0.186, 0.175}, // 5m
	{1.000, 1.000, 0.245, 0.218, 0.196, 0.181, 1, 0.153, 0.143}, // 6m
	{1.000, 1.000, 1.000, 0.188, 0.171, 0.156, 1, 0.131, 0.122}, // 7m
	{1.000, 1.000, 1.000, 1.000, 0.147, 0.135, 1, 0.114, 0.106}, // 8m
	{1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1, 0.101, 0.097}  // 9m
};
string Rstring = "_`_`_`_`_ Right Buoy _`_`_`_`_  {V}"; // Right Buoy ID
string Lstring = "_`_`_`_`_ Left Buoy _`_`_`_`_  {V}"; // Left Buoy ID
string buoy_ID;
float LX, LY, RX, RY; // Variables for the coordinates of the buoys w.r.t USV
std_msgs::Float32 lx, ly, rx, ry; // message type for the coordinates of the buoys w.r.t USV

float N_USV, E_USV, D_USV, PSI_USV;								// USV position in NED
float new_lat, new_long;															// determined lat and long

float u_x[100];
float v_y[100];
float x_offset[100];
float y_offset[100];
char colors[100];
//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................
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



// THIS FUNCTION: Obtains the USV NED position
// ACCEPTS: 
// RETURNS: 
// =============================================================================
void NED_Position(nav_msgs::Odometry usv_posi)
{
	N_USV = usv_posi.pose.pose.position.x;
	E_USV = usv_posi.pose.pose.position.y;
	D_USV = usv_posi.pose.pose.position.z;
	PSI_USV = usv_posi.pose.pose.orientation.z;
} // end of NED_Position()

// THIS FUNCTION: Converts from the relative USV frame to spherical ECEF
// ACCEPTS: 
// RETURNS: 
// =============================================================================
void USVtoECEF(float Bn, float Be, float Un, float Ue, float Ud, float Upsi)
{
	// variables
	// Could add Bd if the program gets improved. This would allow us to find the buoy in 3-space
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;									// transformation from local NED to global NED (Rz)
	float bn, be, bd, bE, bN, bU, x, y, z;													// variables to hold buoy position in various frames
	
	r11 = cos(Upsi);
	r12 = -sin(Upsi);
	//r13 = 0.0;
	r21 = sin(Upsi);
	r22 = cos(Upsi);
	//r23 = 0.0;
	//r31 = 0.0;
	//r32 = 0.0;
	//r33 = 1.0;
	
	// Matrix equation to transform local x to global x_hat: x_hat = Rz*x
	bn = (r11*Bn + r12*Be) + Un;
	be = (r21*Bn + r22*Be) + Ue;
	bd = Ud;	
	// convert from global NED to global ENU
	bE = bn;
	bN = be;
	bU = -Ud - 1.0;
	ROS_INFO("               E: %.8f ", bE);
	ROS_INFO("               N: %.8f ", bN);
	ROS_INFO("               U: %.8f ", bU);
	// convert from global ENU to rectangular ECEF
	x = (R11*bE + R12*bN + R13*bU) - 4630032.0;
	y = (R21*bE + R22*bN + R23*bU) + 2600407.0;
	z = (R31*bE + R32*bN + R33*bU)  -3521046.0;
	new_lat = (atan2(sqrt(pow(x,2)+pow(x,2)),z)*180.0/PI)-151.9952361;
	new_long = atan2(y,x)*180.0/PI;
	ROS_INFO("               lat: %.8f ", new_lat);
	ROS_INFO("               long: %.8f ", new_long);
} // end of USVtoECEF()


// THIS FUNCTION: Finds the best focal length of the camera sensors, based off a LUT
// ACCEPTS: the estimated longitudinal translation to the buoy (zz)
// RETURNS: the corrected focal length (focal_length)
// =============================================================================
float fpLUT(float zz)
{
	float focal_length;
	zz = round(zz - 6.0);				// use longitudinal distance to find entry from LUT
	switch(int(zz))
	{
		case 0:								// <7m
			focal_length = 865.0;
			break;
		case 1:								// 7m
			focal_length = 855.0;
			break;
		case 2:								// 8m
			focal_length = 846.0;
			break;
		case 3:								// 9m
			focal_length = 839.2;
			break;
		case 4:								// 10m
			focal_length = 837.0;
			break;
		case 5:								// 11m
			focal_length = 829.0;
			break;
		case 6:								// 12m
			focal_length = 826.1;
			break;
		case 7:								// 13m
			focal_length = 823.0;
			break;
		case 8:								// 14m
			focal_length = 821.0;
			break;
		case 9:								// 15m
			focal_length = 815.9;
			break;
		default:
			focal_length = 845.0;
	}
	return focal_length;
} // end of fpLUT()

// THIS FUNCTION: Determines the initial lateral gain to multiply the first lateral disparity calculation by, based off a LUT
// ACCEPTS: the approximated lateral translation to the buoy (latzz)
// RETURNS: the lateral gain (gainer)
// =============================================================================
float lateral_gainLUT(float latzz)
{
	float gainer;
	latzz = abs(round(latzz));
	// This LUT is referenced to a longitudinal translation of 13m
	switch(int(latzz))
	{
		case 0:
			gainer = 62.5;
			break;
		case 1:
			gainer = 62.5;
			break;
		case 2:
			gainer = 62.5;
			break;
		case 3:
			gainer = 62.5;
			break;
		case 4:
			gainer = 62.5;
			break;
		case 5:
			gainer = 62.5;
			break;
		case 6:
			gainer = 61.86;
			break;
		case 7:
			gainer = 62.5;
			break;
		case 8:
			gainer = 63.0;
			break;
		case 9:
			gainer = 63.38;
			break;
		default:
			gainer = 62.5;
	}
	return gainer;
} // end of lateral_gainLUT()

// THIS FUNCTION: Determines the final lateral gain to multiply the lateral disparity calculation by, based off a LUT (lat_scale_LUT)
// ACCEPTS: the approximated lateral translation (lat_z) and longitudinal (long_z) translation to the buoy
// RETURNS: final lateral gain (lats)
// =============================================================================
float lateral_scale(float lat_z, float long_z)
{
	float lats = 1.0, lat_z_prev, error_s;
	int r, c;
	c = int(round(long_z) - 7.0);							// Use the longitudinal translation to determine the column of the LUT to look at
	if(c<0)														//_ Do not let c break the LUT columns
	{																//	|
		c = 0;													//	|
	}																//	|
	if(c>8)														//	|
	{																//	|
		c = 8;													//	|
	}																//_|
	lat_z_prev = lat_z;
	lat_z = 1.0/lat_z;
	if(c==6)
	{
		return lats;
	}
	for(r=1;r<10;r++)
	{
		if(lat_z>=lat_scale_LUT[r][c])
		{
			lats = float(r)*lat_scale_LUT[r][c];
			break;
		}
	}
	return lats;
} // end of lateral_scale()

// THIS FUNCTION: 
// ACCEPTS: centroid coordinates
// RETURNS: nothing
// =============================================================================
void DisparityFunc()
{
	float d[left_blob_cnt];						// The disparity between camera images [pixels]
	float f_p = 820.0;							// experimental focal length [pixels]
	float z[left_blob_cnt];						// Distance along USV surge to the projection of the point [m]
	float z_0;										// Used to obtain more accurate estimate of z [m]
	float s_left[left_blob_cnt], s_right[left_blob_cnt];	// Distances from centrod to each image's interior vertical [pixels]
	float ds_left[left_blob_cnt];				// The "lateral disparity" for each camera image [pixels]
	float ds_right[left_blob_cnt];			// The "lateral disparity" for each camera image [pixels]
	float lat_z_left[left_blob_cnt];			// Distance along USV sway to the projection of the point [m]
	float lat_z_right[left_blob_cnt];		// Distance along USV sway to the projection of the point [m]
	float lat_z_avg[left_blob_cnt];
	float g_left = 62.5;
	float g_right = 62.5;
	float k_lat = -1.0;
	float ds_lat = -1.0;
	for(int i=0; i<left_blob_cnt; i++)
	{
		d[i] = MC[i].x - MC1[i].x;
		z_0 = (f_p*BASELINE)/d[i];
		f_p = fpLUT(z_0);
		z[i] = (f_p*BASELINE)/d[i];
		
		s_left[i] = MC[i].x - (Umax/2.0);
		s_right[i] = MC1[i].x - (Umax/2.0);
		ds_left[i] = (z[i]*s_left[i])/f_p;
		ds_right[i] = (z[i]*s_right[i])/f_p;
		ds_lat = (ds_left[i]+ds_right[i])/2.0;
		lat_z_left[i] = g_left*((z[i]*ds_left[i])/f_p - 0.001);
		lat_z_right[i] = g_right*((z[i]*ds_right[i])/f_p + 0.002);
		g_left = lateral_gainLUT(lat_z_left[i]);
		g_right = lateral_gainLUT(lat_z_right[i]);
		lat_z_left[i] = g_left*((z[i]*ds_left[i])/f_p - 0.001);
		lat_z_right[i] = g_right*((z[i]*ds_right[i])/f_p + 0.002);
		lat_z_avg[i] = (lat_z_left[i] + lat_z_right[i])/2.0;
		k_lat = lateral_scale(lat_z_avg[i], z[i]);
		lat_z_avg[i] = lat_z_avg[i]*k_lat;
		
		if(isfinite(lat_z_avg[i])&&isfinite(z[i]))
		{
			if(i==0)
			{
				buoy_ID = Rstring;
				RX = z[i];
				RY = lat_z_avg[i];
			}
			else
			{
				buoy_ID = Lstring;
				LX = z[i];
				LY = lat_z_avg[i];
			}
			//ROS_INFO(buoy_ID);
			ROS_INFO("________vvvvvvvvvvv  {V}  vvvvvvvvvvv___________________");
			ROS_INFO("                 Left Camera                            |");
			ROS_INFO("               u: %.0f [pixels]|", MC[i].x);
			ROS_INFO("               v: %.0f [pixels]", MC[i].y);
			ROS_INFO(".................Right Camera...........................|");
			ROS_INFO("               u: %.0f [pixels]", MC1[i].x);
			ROS_INFO("               v: %.0f [pixels]", MC1[i].y);
			ROS_INFO("            Disparity: %.0f [pixels]", d[i]);
			ROS_INFO("         Lateral Disparity:  %.0f [pixels]", ds_lat);
			ROS_INFO("Longitudinal Translation from USV (x): %.3f [m]", z[i]);
			ROS_INFO("  Lateral Translation from USV (y): %.3f [m]", lat_z_avg[i]);
			ROS_INFO("________^^^^^^^^^^^  {V}  ^^^^^^^^^^^___________________|\n");
			/* printf("u: %.0f [pixels]   |||   %.0f [pixels]\n", MC[i].x, MC1[i].x);
			printf("v: %.0f [pixels]   |||   %.0f [pixels]\n", MC[i].y, MC1[i].y);
			printf("Disparity: %.0f [pixels]\n             Lateral Disparity:  %.0f [pixels]\n", d[i], d[i]);
			printf("Translation from USV (x,y): (%.3f [m] , %.3f [m])\n\n", z[i], lat_z_avg[i]); */
		}
		
		else
		{
			ROS_ERROR("Non-finite Disparity Calculation. {V}");
		}
	}
	
	for (int j=0; j<left_blob_cnt; j++) // organize the calculated distances to their respective buoy types and color 
	{
		for (int g=0; g<left_blob_cnt; g++) // search the entire array
		{
			if ((u_x[j]==MC[g].x) && (v_y[j]==MC[g].y))
			{
				x_offset[j] = z[g];
				y_offset[j] = lat_z_avg[g];
			}
		}
	}




	int i = 0;
				
					if (size_red > 0) {
					for (size_t j=0; j < size_red; ++j) {
					USVtoECEF(x_offset[i], y_offset[i], N_USV, E_USV, D_USV, PSI_USV);
					xdistance_red = new_lat;
					ydistance_red = new_long;
					printf("red lat is %f\n", xdistance_red);
					printf("red long is %f\n", ydistance_red); 
					i++;
					}
					size_red = 0;
				}
				
					if (size_green > 0) {
					for (size_t j=0; j < size_green; ++j) {
					USVtoECEF(x_offset[i], y_offset[i], N_USV, E_USV, D_USV, PSI_USV);
					xdistance_green = new_lat;
					ydistance_green = new_long;
					printf("green lat is %f\n", xdistance_green);
					printf("green long is %f\n", ydistance_green);
					i++;
					}
					size_green = 0;
				}
				
				
					if (size_white > 0) {
						for (size_t j=0; j < size_white; ++j) {
								USVtoECEF(x_offset[i], y_offset[i], N_USV, E_USV, D_USV, PSI_USV);
					xdistance_white = new_lat;
					ydistance_white = new_long;
					printf("white lat is %f\n", xdistance_white);
					printf("white long is %f\n", ydistance_white);
					i++;
						}
						size_white = 0;
				}
				
				
					if (size_orange > 0) {
						for (size_t j=0; j < size_orange; ++j) {
								USVtoECEF(x_offset[i], y_offset[i], N_USV, E_USV, D_USV, PSI_USV);
					xdistance_orange = new_lat;
					ydistance_orange = new_long;
					printf("orange lat is %f\n", xdistance_orange);
					printf("orange long is %f\n", ydistance_orange);
					i++;
						}
						size_orange = 0;
				}
				
					if (size_black > 0) {
					for (size_t j=0; j < size_black; ++j) {
							USVtoECEF(x_offset[i], y_offset[i], N_USV, E_USV, D_USV, PSI_USV);
					xdistance_black = new_lat;
					ydistance_black = new_long;
					printf("black lat is %f\n", xdistance_black);
					printf("black long is %f\n", ydistance_black);
					i++;
					}
					size_black = 0;
				}
				



	left_blob_cnt = 0;
} // end of DisparityFunc()

// THIS FUNCTION: 
// ACCEPTS: 
// RETURNS: nothing
// =============================================================================
void cameraCallBack(const sensor_msgs::ImageConstPtr& camera_msg)
{
	int counter = 0; // used for placement in arrays
	
	try 
	{
		org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		//cv::imshow("Left Camera", org_img); //converts the ros image to bgr type
		Mat background;
		Mat grey_frame;
		Mat threshold;
		Mat f_con;
		Mat imgHSV;
		org_img.copyTo(background);
		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV);
		
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
		
	cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		for (size_t i=0; i < contours.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
			
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
				for( int i = 0; i<contours.size(); i++ ) {
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
				u_x[counter] = mc[i].x;
				v_y[counter] = mc[i].y;
				colors[counter] = 'r';
				counter += 1;
				}
				//Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
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
						task3_message.pose.position.latitude = xdistance_red;
						task3_message.pose.position.longitude = ydistance_red;
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
						task3_message.pose.position.latitude = xdistance_red;
						task3_message.pose.position.longitude = ydistance_red;
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
				for( int i = 0; i<contours.size(); i++ ) {
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
				u_x[counter] = mc[i].x;
				v_y[counter] = mc[i].y;
				colors[counter] = 'g';
				counter += 1;
				}
				//Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
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
						task3_message.pose.position.latitude = xdistance_green;
						task3_message.pose.position.longitude = ydistance_green;
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
						task3_message.pose.position.latitude = xdistance_green;
						task3_message.pose.position.longitude = ydistance_green;
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
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { //need to add this because with fog the background of trees are white and it draws a big rectangle around the trees
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
				for( int i = 0; i<contours.size(); i++ ) {
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
				u_x[counter] = mc[i].x;
				v_y[counter] = mc[i].y;
				colors[counter] = 'w';
				counter += 1;
				}				
				//Scalar color = Scalar(128,0,0);
			//	circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
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
						task3_message.pose.position.latitude = xdistance_white;
						task3_message.pose.position.longitude = ydistance_white;
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
						task3_message.pose.position.latitude = xdistance_white;
						task3_message.pose.position.longitude = ydistance_white;
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
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
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
				for( int i = 0; i<contours.size(); i++ ) {
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
				u_x[counter] = mc[i].x;
				v_y[counter] = mc[i].y;
				colors[counter] = 'o';
				counter += 1;
				}				
				//Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
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
						task3_message.pose.position.latitude = xdistance_orange;
						task3_message.pose.position.longitude = ydistance_orange;
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
						task3_message.pose.position.latitude = xdistance_orange;
						task3_message.pose.position.longitude = ydistance_orange;
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
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
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
				for( int i = 0; i<contours.size(); i++ ) {
				pts = contours[i];
				mc[i] = Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
				u_x[counter] = mc[i].x;
				v_y[counter] = mc[i].y;
				colors[counter] = 'b';
				counter += 1;
				}
				//Scalar color = Scalar(128,0,0);
				//circle(mask_t, mc[i], 4, color, -1, 8, 0 );
				
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
						task3_message.pose.position.latitude = xdistance_black;
						task3_message.pose.position.longitude = ydistance_black;
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
						task3_message.pose.position.latitude = xdistance_black;
						task3_message.pose.position.longitude = ydistance_black;
						task3_pub.publish(task3_message); 
						circle_buoy = 0;
						color_type_buoy = " ";
						break;
					}
				}
			}
		} 	
		
		cv::SimpleBlobDetector::Params blob;
		
//		blob.minThreshold = 240;
//		blob.maxThreshold = 255;
		blob.filterByArea = false;
		blob.minArea = 20;
		blob.filterByCircularity = false;
//		blob.minCircularity = 0.1;
		blob.filterByConvexity = false;
//		blob.minConvexity = 0.87;
		blob.filterByInertia = false;
//		blob.minInertiaRatio = 0.01;
//		 cv::SimpleBlobDetector detector(blob);

		Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(blob);

		std::vector <cv::KeyPoint> keypoints;
		cv::bitwise_not(mask_t, mask_new); //black BLOB on white background
		detector->detect(mask_new, keypoints);

		if (keypoints.size() == 0) //nothing was found
		{
			ROS_ERROR("No Object Detected by Left Camera. {V}");
		}
		else 
		{			
			cv::findContours(mask_t, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			for (size_t i=0; i < contours.size(); ++i) {
				cv::Rect boundRect = cv::boundingRect(contours[i]);
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
				size_mask_t = contours.size();
			
			// Find the moments of each contour 
				vector<Moments> mu_m(contours.size());
				for( int i = 0; i<contours.size(); i++ )
				{
					mu_m[i] = cv::moments(contours[i], false);
				}
			
			// Find the centroid of each BLOB based on the contour moments
			// ...important note: centroids change depending on scaling
				vector<Point2f> mc_m(contours.size());
				for( int i = 0; i<contours.size(); i++)
				{
					mc_m[i] = Point2f( mu_m[i].m10/mu_m[i].m00 , mu_m[i].m01/mu_m[i].m00 );
					MC[i] = mc_m[i];
					left_blob_cnt = left_blob_cnt + 1;
				}
				DisparityFunc();
			
			// Place a small circle centered at the centroid of the BLOB
				for( int i = 0; i<contours.size(); i++ )
				{
					Scalar color = Scalar(128,0,0);
					circle(mask_t, mc_m[i], 4, color, 1, 8, 0 );
				} 
			}
		}
	}
		
		
		cv::imshow("Left Camera Updated", background);   
		cv::imshow("Left Camera Features", mask_t); 
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Left Camera could not convert from '%s' to 'bgr8'. {V}", camera_msg -> encoding.c_str()); //prints out the encoding string
	}
	
	for (int k=0; k<counter; k++)
	{
		printf("x: %.2f, y: %.2f, color: %c, x_off: %.2f, y_off: %.2f\n", u_x[k], v_y[k], colors[k], x_offset[k], y_offset[k]);
	}
	
} // end of cameraCallBack()

// THIS FUNCTION: 
// ACCEPTS: 
// RETURNS: nothing
// =============================================================================
void cameraCallBack1(const sensor_msgs::ImageConstPtr& camera_msg1) {

	try 
	{
		org_img1 = cv_bridge::toCvShare(camera_msg1, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		//cv::imshow("Right Camera", org_img1); //converts the ros image to bgr type
		Mat background1;
		Mat grey_frame1;
		Mat threshold1;
		Mat f_con1;
		Mat imgHSV1;
		org_img1.copyTo(background1);
		cv::cvtColor(org_img1, imgHSV1, cv::COLOR_BGR2HSV);
		
		HSV_change(imgHSV1);
		
		
		//this is used to also detect the red totem because its Hue value is from 0-1
		cv::inRange(imgHSV1, red_low, red_high, red_mask); 
		cv::inRange(imgHSV1, red_low1, red_high1, red_mask1); 
		
		//add first red mask with the new red mask
		red_mask = red_mask + red_mask1;
		
		//threshold to find only certain color 
		cv::inRange(imgHSV1, white_low, white_high, white_mask);
		cv::inRange(imgHSV1, orange_low, orange_high, orange_mask); 
		cv::inRange(imgHSV1, black_low, black_high, black_mask); 
		cv::inRange(imgHSV1, green_low, green_high, green_mask);  
		
		//using a median filter build in function apply it to each mask with a kernal size of 3 to get rid of small noise
		cv::medianBlur(black_mask, black_mask, 3);
		cv::medianBlur(white_mask, white_mask, 3);
		cv::medianBlur(orange_mask, orange_mask, 3);
		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		
		//add each mask together to create one image with all colors on it which is used for blob detection
		cv::Mat mask_t1;
		mask_t1 = white_mask + green_mask + orange_mask + black_mask + red_mask;
		
		//applies each mask to the original image using bitwise_and and outputs it to a new mat file which isn't useful rn
		cv::bitwise_and(org_img1, org_img1, mask=red_mask, red_res);
		cv::bitwise_and(org_img1, org_img1, mask=white_mask, white_res); 
		cv::bitwise_and(org_img1, org_img1, mask=orange_mask, orange_res);
		cv::bitwise_and(org_img1, org_img1, mask=green_mask, green_res);
		cv::bitwise_and(org_img1, org_img1, mask=black_mask, black_res);
		
		// Finds contour around coloured BLOBS and draws a rectangle around the object
		//for each findContours it looks at each colored mask and calculates the contrours of the object and then draws the correct colored rectangle around the object
/* 		cv::findContours(red_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //finds contour around coloured objects and draws a rectangle around the object
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
				cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
			}
		}
		
 		cv::findContours(green_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
				cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
			}
		}

		cv::findContours(white_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { //need to add this because with fog the background of trees are white and it draws a big rectangle around the trees
				cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(255, 255, 255), 3);
			}
		}
		
		cv::findContours(orange_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
				cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 179, 255), 3);
			}
		}
		
	 	cv::findContours(black_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i) {
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			if ((boundRect.area() > 20) && (boundRect.width < 1000)) { 
				cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 0), 3); 
			}
		} */
		
		cv::SimpleBlobDetector::Params blob1;
		
//		blob1.minThreshold = 240;
//		blob1.maxThreshold = 255;
		blob1.filterByArea = false;
		blob1.minArea = 20;
		blob1.filterByCircularity = false;
//		blob1.minCircularity = 0.1;
		blob1.filterByConvexity = false;
//		blob1.minConvexity = 0.87;
		blob1.filterByInertia = false;
//		blob1.minInertiaRatio = 0.01;
//		 cv::SimpleBlobDetector detector(blob);

		Ptr<cv::SimpleBlobDetector> detector1 = cv::SimpleBlobDetector::create(blob1);

		std::vector <cv::KeyPoint> keypoints1;
		cv::bitwise_not(mask_t1, mask_new1); //black BLOB on white background
		detector1->detect(mask_new1, keypoints1);

		if (keypoints1.size() == 0) //nothing was found
		{
			ROS_ERROR("Nothing Detected by Right Camera. {V}");
		}
		else 
		{			
			cv::findContours(mask_t1, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			for (size_t i=0; i < contours1.size(); ++i) {
				cv::Rect boundRect = cv::boundingRect(contours1[i]);
				if ((boundRect.area() > 100) && (boundRect.width < 1000)) { 
			
			// Find the moments of each contour 
				vector<Moments> mu1(contours1.size());
				for( int i = 0; i<contours1.size(); i++ )
				{
					mu1[i] = cv::moments(contours1[i], false);
				}
			
			// Find the centroid of each BLOB based on the contour moments
			// ...important note: centroids change depending on scaling
				vector<Point2f> mc1(contours1.size());
				for( int i = 0; i<contours1.size(); i++)
				{
					mc1[i] = Point2f( mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00 );
					MC1[i] = mc1[i];
					right_blob_cnt = right_blob_cnt + 1;
				}
			// DisparityFunc() should not be used in this subroutine      {SSP, 12/04/2021}
			
			// Place a small circle centered at the centroid of the BLOB
				for( int i = 0; i<contours1.size(); i++ )
				{
					Scalar color = Scalar(128,0,0);
					circle(mask_t1, mc1[i], 4, color, 1, 8, 0 );
				}	 
			}
		}
	}
		//cv::imshow("Right Camera Updated", background1);   
		cv::imshow("Right Camera Features", mask_t1); 
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Right Camera could not convert from '%s' to 'bgr8'. {V}", camera_msg1 -> encoding.c_str()); //prints out the encoding string
	}
	
} // end of cameraCallBack1()

//............................................................End of Functions............................................................


//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_vis");
	
	// Variables
	int i = 0;
	// Initializations
	/* cv::namedWindow("Left Camera"); //creates new windows for each camera
	cv::namedWindow("Right Camera"); */
	cv::namedWindow("Left Camera Updated", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Left Camera Features", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Right Camera Updated", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Right Camera Features", cv::WINDOW_AUTOSIZE);
  
	// Node handles
	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
	ros::NodeHandle nh3;
	
	// Subscribers
	image_transport::ImageTransport it(nh); //transports the images from the subscriber
	image_transport::ImageTransport it1(nh1);
	image_transport::Subscriber camera_sub = it.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, cameraCallBack); //front left camera
	image_transport::Subscriber camera_sub1 = it1.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 1, cameraCallBack1); //front rightcamera
	
	ros::Subscriber nedpos_sub = nh3.subscribe("usv_ned", 1, NED_Position); // subscribes to NED position
	
	task3_pub = nh2.advertise<geographic_msgs::GeoPoseStamped>("/vrx/perception/landmark", 1); 
	
	// Initialize simulation time
	ros::Time::init();
  
	// Set the loop sleep rate
	ros::Rate loop_rate(100);

	ros::spinOnce();
	/* cv::destroyWindow("Left Camera"); //destroys the new windows
	cv::destroyWindow("Right Camera"); */
	cv::destroyWindow("Left Camera Updated");
	cv::destroyWindow("Left Camera Features");
	//cv::destroyWindow("Right Camera Updated");
	cv::destroyWindow("Right Camera Features");
	
	while(ros::ok())
	{	
				
		ros::spinOnce();
		loop_rate.sleep();
		
	}
} // end of main()
//............................................................End of Main Program...........................................................