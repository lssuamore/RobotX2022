#include "ros/ros.h"
#include "std_msgs/Float32.h"

int main (int argc, char **argv) {

	ros::init(argc, argv, "Publisher");

	ros::NodeHandle nh;

	ros::Publisher topic_pub = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1000);
	ros::Publisher topic_pub1 = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1000);
//	ros::Publisher topic_pub2 = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1000);
//	ros::Publisher topic_pub3 = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 1000);

	ros::Rate loop_rate(1);

	while(ros::ok()) {

	std_msgs::Float32 msg, msg1;

	msg.data = 1.0;
	topic_pub.publish(msg);
	msg1.data = 1.0;
	topic_pub1.publish(msg1);

	ros::spinOnce();
	loop_rate.sleep();
	}

	return 0;
}

