#pragma once

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mutex>

namespace points_to_path
{
class PointsToPath
{
public:
  PointsToPath();

private:
  bool demonstrating_;
  bool was_demonstrating_;
  std::mutex path_mutex_;

  ros::Publisher const_path_pub_;
  ros::Publisher path_pub_;
  ros::Subscriber demo_status_sub_;
  ros::Subscriber points_sub_;
  std::vector<geometry_msgs::PointStamped> points_;

  nav_msgs::Path current_path_;
  std::vector<geometry_msgs::PointStamped> current_points_;

  void newPointCallback(const geometry_msgs::PointStamped& msg);
  void demoStatusCallback(const std_msgs::Bool& msg);
};
}
