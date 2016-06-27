#include "recovery_supervisor/points_to_path.h"
#include <stdlib.h>

#include <ros/ros.h>

namespace points_to_path
{
PointsToPath::PointsToPath()
{
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  points_sub_ = nh.subscribe("clicked_point", 10, &PointsToPath::newPointCallback, this);

  path_pub_ = nh_private.advertise<nav_msgs::Path>("path", false);

  ros::Rate r(10);
  while (ros::ok())
  {
    path_pub_.publish(current_path_);
    ros::spinOnce();
    r.sleep();
  }
}

void PointsToPath::newPointCallback(const geometry_msgs::PointStamped& msg)
{
  ROS_INFO("new pt: (%f,%f)", msg.point.x, msg.point.y);
  if (current_path_.poses.empty())
  {
    //initialize a new path
    current_path_.header.stamp = ros::Time::now();
    current_path_.header.frame_id = "map";
  }

  geometry_msgs::PoseStamped new_pose;
  new_pose.header = msg.header;
  new_pose.pose.position.x = msg.point.x;
  new_pose.pose.position.y = msg.point.y;
  new_pose.pose.position.z = msg.point.z;
  new_pose.pose.orientation.x = 0;
  new_pose.pose.orientation.y = 0;
  new_pose.pose.orientation.z = 0;
  new_pose.pose.orientation.w = 1;

  current_path_.poses.push_back(new_pose);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_to_path");

  points_to_path::PointsToPath ptp;

  return EXIT_SUCCESS;
}
