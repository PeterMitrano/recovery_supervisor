#pragma once

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <vector>

namespace recovery_supervisor
{
class RecoverySupervisor
{
public:
  /** Main entry point for the node. Subscribes to the following topics
   * (names are configurable on parameter sever, theres are just defaults):
   * teleop commands: ~/teleop/cmd_vel geometry_msgs::Twist
   * odometry: ~/odometry nav_msgs::Odometry
   * local costmaps: /move_base/local_costmap/costmap_updates map_msgs::OccupancyGridUpdate
   * global costmaps: /move_base/gobal_costmap/costmap_updates map_msgs::OccupancyGridUpdate
   * moving obstacles: ~/moving_obstacles ??
   * labeled static features ~/labeled_features ??
   * move_base failure: ~/move_base/status actionlib_msgs::GoalStatusArray
   * joystick for ending demonstration: /joy sensor_msgs::Joy
   *
   */
  RecoverySupervisor();

private:
  int finish_demonstration_button_;
  double minimum_displacement_;
  double stagnation_check_period_;
  bool starting_demonstration_;
  bool ending_demonstration_;
  bool demonstrating_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber local_costmap_sub_;
  ros::Subscriber global_costmap_sub_;
  ros::Time stagnation_start_time_;
  tf::Pose start_stagnation_pose_;

  std::vector<geometry_msgs::Twist> velocities_;

  /**
   * Logs velocity commands sent to the robot whilst an demonstration is occuring
   */
  void teleopCallback(const geometry_msgs::Twist& msg);

  /**
   * Logs pose and determines when the robot is sufficiently stuck that it needs help.
   * If it stuck, it will note that an demonstration is needed, and notify the demonstrator.
   */
  void odometryCallback(const nav_msgs::Odometry& msg);

  /** logs costmaps sent during demonstration */
  void localCostmapCallback(const map_msgs::OccupancyGridUpdate& msg);

  /** logs costmaps sent during demonstration */
  void globalCostmapCallback(const map_msgs::OccupancyGridUpdate& msg);

  /** signal end of teleop */
  void joyCallback(const sensor_msgs::Joy& msg);

  // void MovingObstaclesCallback();
  // void LabeledStaticFeaturesCallback();

  /**
   * logs the status of the move_base goal. If failure is detected,
   * it will note that an demonstration is needed, and notify the demonstrator
   */
  void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& msg);

  void notifyDemonstrator();
};
}
