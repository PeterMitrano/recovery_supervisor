#include "recovery_supervisor/points_to_path.h"
#include <stdlib.h>

#include <ros/ros.h>

namespace points_to_path
{
PointsToPath::PointsToPath() : demonstrating_(false), was_demonstrating_(false)
{
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  demo_status_sub_ =
      nh.subscribe("recovery_supervisor/demonstration_status", 10, &PointsToPath::demoStatusCallback, this);
  points_sub_ = nh.subscribe("clicked_point", 10, &PointsToPath::newPointCallback, this);

  path_pub_ = nh_private.advertise<nav_msgs::Path>("path", false);
  const_path_pub_ = nh_private.advertise<nav_msgs::Path>("const_path", false);

  ros::Rate r(10);
  while (ros::ok())
  {
    if (!current_path_.poses.empty() && !current_path_.header.frame_id.empty())
    {
      const_path_pub_.publish(current_path_);
    }

    ros::spinOnce();
    r.sleep();
  }
}

void PointsToPath::demoStatusCallback(const std_msgs::Bool& msg)
{
  was_demonstrating_ = demonstrating_;
  demonstrating_ = msg.data;

  if (demonstrating_ && !was_demonstrating_)
  {
    // clear and start new path
    ROS_INFO("starting new path.");
    path_mutex_.lock();
    current_path_.poses.clear();
    current_path_.header.stamp = ros::Time::now();
    current_path_.header.frame_id = "map";
    path_mutex_.unlock();
  }
}

void PointsToPath::newPointCallback(const geometry_msgs::PointStamped& msg)
{
  if (demonstrating_)
  {
    geometry_msgs::PoseStamped new_pose;
    new_pose.header = msg.header;
    new_pose.pose.position.x = msg.point.x;
    new_pose.pose.position.y = msg.point.y;
    new_pose.pose.position.z = msg.point.z;
    new_pose.pose.orientation.x = 0;
    new_pose.pose.orientation.y = 0;
    new_pose.pose.orientation.z = 0;
    new_pose.pose.orientation.w = 1;

    path_mutex_.lock();
    current_path_.poses.push_back(new_pose);
    path_mutex_.unlock();

    path_pub_.publish(current_path_);
  }
}
}  // namespace points_to_path

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_to_path");

  points_to_path::PointsToPath ptp;

  return EXIT_SUCCESS;
}
