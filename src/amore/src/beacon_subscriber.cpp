#include "ros/ros.h"
#include "iostream"
#include "stdio.h"
#include "usv_msgs/RangeBearing.h"

float range;
float bearing;
float elevation;


void BeaconCallBack(const usv_msgs::RangeBearing::ConstPtr& beacon_msg) {
	range = beacon_msg->range;
	bearing = beacon_msg->bearing;
	elevation = beacon_msg->elevation;
	printf("the range is: %f\n", range); 
	printf("the bearing is: %f\n", bearing); 
	printf("the elevation is: %f\n", elevation); 
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Beacon");

	ros::NodeHandle nh;

	ros::Subscriber topic_sub = nh.subscribe("/wamv/sensors/pingers/pinger/range_bearing", 10, BeaconCallBack); //subscribes to sensor that detects underwater beacon

	ros::Rate loop_rate(10);

	while(ros::ok()) {

	ros::spinOnce();
	loop_rate.sleep();
	}

}
