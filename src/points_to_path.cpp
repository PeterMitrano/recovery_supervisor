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

  ros::spin();
}

void PointsToPath::newPointCallback(const geometry_msgs::PointStamped& msg)
{
  ROS_INFO("new pt: (%f,%f)", msg.point.x, msg.point.y);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_to_path");

  points_to_path::PointsToPath ptp;

  return EXIT_SUCCESS;
}
