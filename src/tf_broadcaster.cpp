#include "ros/node_handle.h"
#include "ros/rate.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  // This object is responsible for broadcasting the TF for the frames in question
  tf::TransformBroadcaster broadcaster;

  while (n.ok()) {
	// Broadcasting a StampedTransform to the available listeners
	broadcaster.sendTransform(
							  tf::StampedTransform(
												   tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
												   ros::Time::now(), "base_link", "base_laser"));
	r.sleep();
  }
}
