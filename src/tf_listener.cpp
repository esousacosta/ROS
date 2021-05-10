#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "tf/LinearMath/Transform.h"
#include "tf/exceptions.h"
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener)
{
  // Here we're creating a random point that we'd like to transform from
  // the "base_laser" frame to the "base_link" frame
  geometry_msgs::PointStamped fabricated_laser_point;
  fabricated_laser_point.header.frame_id = "base_laser";

  // Puttin the actual time as the stamp for the point
  fabricated_laser_point.header.stamp = ros::Time();

  // Here is the actual point
  fabricated_laser_point.point.x = 1.0;
  fabricated_laser_point.point.y = 0.2;
  fabricated_laser_point.point.z = 0.0;
  
  // Now we're trying to perform the transformation, and if doesn't succeed for whatever reason,
  // the user is warned about the error.
  try {
	geometry_msgs::PointStamped base_point;
	listener.transformPoint("base_link", fabricated_laser_point, base_point);

	// Printing the information on screen
	ROS_INFO("base_laser: (%.2f, %.2f, %.2f) ----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
			 fabricated_laser_point.point.x, fabricated_laser_point.point.y, fabricated_laser_point.point.z,
			 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  } catch (tf::TransformException& ex) {
	ROS_ERROR("Received an exception while trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  // The transformation is performed every second
  ros::Timer times = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();
}

