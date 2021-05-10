#include "ros/node_handle.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& incoming_scan)
{
  for (auto& element: incoming_scan->ranges) {
	ROS_INFO("Received data: %.2f", element);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_listener");
  ros::NodeHandle n;

  ros::Subscriber laser_sub = n.subscribe("ros_robotics/laser/scan", 1000, laserCallback);

  ros::spin();
  return 0;
}

