#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

void callback(const sensor_msgs::LaserScan& msg) {
	for (int i = 0; i < 666; ++i) {
		ROS_INFO("range %i: %f\n", i, msg.ranges[i]);
		ROS_INFO("intensity %i: %f\n", i, msg.intensities[i]);
	}
	ROS_INFO("--------------------------------------------------------------------");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("base_scan", 1000, callback);
    ros::spin();
    return 0;
}
