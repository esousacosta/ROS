#include "ros/node_handle.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& incoming_scan)
{
  // The initial angle is -pi (pointing to the back of the robot),
  // and the final angle is pi (pointing in the same direction, but after
  // having swept all around the robot in a counter-clockwise movement.
  auto angle = -3.14;
  auto angle_increment = incoming_scan->angle_increment;
  int ind = 0;

  for (auto& element: incoming_scan->ranges) {
	ROS_INFO("Received data: [dist., angle] = [%.2f, %.2f]", element, angle + angle_increment * ind);
	ind++;
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

