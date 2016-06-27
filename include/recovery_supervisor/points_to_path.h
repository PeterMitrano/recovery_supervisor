#pragma once

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

namespace points_to_path
{
class PointsToPath{
  public:
    PointsToPath();
  private:
    ros::Subscriber points_sub_;
    ros::Publisher path_pub_;
    std::vector<geometry_msgs::PointStamped> points_;

    nav_msgs::Path current_path_;
    std::vector<geometry_msgs::PointStamped> current_points_;

    void newPointCallback(const geometry_msgs::PointStamped& msg);
};
}
