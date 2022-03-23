#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include "iostream"
#include "stdio.h"
#include "time.h"

float latitude;
float longitude;
float altitude;

void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	altitude = gps_msg->altitude; //sets altitude rom gps
	/* printf("the latitude is: %f\n", latitude); //prints
	printf("the longitude is: %f\n", longitude);
	printf("the altitude is: %f\n", th); */
}

void gpsNode(const sensor_msgs::NavSatFix::ConstPtr& node_msg) {
	latitude = node_msg->latitude; //sets latitude from gps
	longitude = node_msg->longitude; //sets longitude from gps
	altitude = node_msg->altitude; //sets altitude rom gps
	/* printf("the latitude is: %f\n", latitude); //prints
	printf("the longitude is: %f\n", longitude);
	printf("the altitude is: %f\n", th); */
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Geoposition");

	ros::NodeHandle nh;
	ros::NodeHandle nh1;

	ros::Subscriber topic_sub = nh.subscribe("/gps/fix", 10, gpsCallBack);
	
	ros::Subscriber topic_sub1 = nh1.subscribe("fix", 10, gpsNode);
	
	
	
	ros::Rate loop_rate(10);

	while(ros::ok()) {

	ros::spinOnce();
	loop_rate.sleep();
	
	}

}
