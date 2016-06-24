#pragma once

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

namespace points_to_path
{
class PointsToPath{
  public:
    PointsToPath();
  private:
    ros::Subscriber points_sub_;

    void newPointCallback(const geometry_msgs::PointStamped& msg);
};
}
