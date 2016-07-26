#pragma once

#include "recovery_supervisor/recovery_point.h"

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/publisher.h>
#include <recovery_supervisor_msgs/PosVelTimeGoalDemo.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <mutex>
#include <string>
#include <vector>

namespace recovery_supervisor
{
/**
 * @brief Recovery Supervisor is meant to work along side move_base
 * and several other nodes in order to determine if a demonstration
 * is needed. It also handles bagging data during demonstrations.
 * It's a convoluted mess, and definitely has lots of concurrency errors.
 */
class RecoverySupervisor
{
public:
  /** Main entry point for the node.
   * looks for abortion of move_base or not moving far enough in a given time.
   * then records from many topics into ros bags.
   * pressing a button ends the demonstration, closes the bag,
   * and opens a new one in preparation
   */
  RecoverySupervisor();

private:
  int bag_index_;
  int finish_demonstration_button_;
  int first_recovery_count_;
  int force_demonstration_button_;
  int maximum_first_recovery_count_;
  ros::Duration estimate_trip_time_;
  double average_forward_velocity_;
  double maximum_displacement_jump_;
  double max_trip_time_error_factor_;
  double minimum_displacement_;
  bool starting_demonstration_;
  bool ending_demonstration_;
  bool demonstrating_;
  bool has_goal_;
  bool has_path_;
  bool first_msg_;
  bool first_amcl_msg_;

  recovery_supervisor_msgs::PosVelTimeGoalDemo current_demo_;

  geometry_msgs::Pose current_goal_pose_;
  geometry_msgs::PoseStamped last_amcl_pose_;
  geometry_msgs::PoseStamped latest_pose_;
  geometry_msgs::PoseStamped last_recovery_pose_;
  geometry_msgs::Twist last_velocity_;

  nav_msgs::Path current_demo_path_;
  nav_msgs::Path current_amcl_path_;

  pcl::PointCloud<RecoveryPoint>* recovery_cloud_;
  pcl_ros::Publisher<RecoveryPoint> recovery_cloud_pub_;

  ros::Publisher cancel_pub_;
  ros::Publisher demo_pub_;
  ros::Publisher complete_demo_path_pub_;
  ros::Publisher cropped_path_pub_;
  ros::Publisher failure_location_pub_;
  ros::Publisher status_pub_;

  ros::Subscriber amcl_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber demo_path_sub_;
  ros::Subscriber global_path_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber new_goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber recovery_status_sub_;
  ros::Subscriber tf_sub_;
  ros::Subscriber velocity_sub_;

  ros::Time trip_time_start_time_;
  std::mutex bag_mutex_;
  std::string bag_file_directory_;
  std::string current_goal_id_;

  rosbag::Bag* bag_;

  /** computes position changes and
   * tracks localization so we know if it goes crazy */
  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

  int indexOfClosestPose(geometry_msgs::PoseStamped, const nav_msgs::Path path);

  nav_msgs::Path crop_path(const nav_msgs::Path demo_path, const nav_msgs::Path amcl_path);

  /** logs path coming from points_to_path */
  void demoPathCallback(const nav_msgs::Path& msg);

  /** helper function for calculating euclidian distance */
  double dist(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);

  /** gets global path in order to compute and ETA for the plan */
  void globalPlanCallback(const nav_msgs::Path& msg);

  int hourOfDay();

  /** signal end of teleop */
  void joyCallback(const sensor_msgs::Joy& msg);

  /** signal start of new goal (either clicked or sent as msg) */
  void newGoalCallback(const geometry_msgs::PoseStamped& msg);

  /**
   * logs the status of the move_base goal. If failure is detected,
   * it will note that an demonstration is needed, and notify the demonstrator
   */
  void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& msg);

  void moveBaseGoalCallback(const geometry_msgs::PoseStamped& msg);

  void notifyDemonstrator();

  /** odom for publishing failure locations */
  void odometryCallback(const nav_msgs::Odometry& msg);

  /** logs recovery instances. Counting these gives us failure information */
  void recoveryCallback(const move_base_msgs::RecoveryStatus& msg);

  void recoveryStatsMsgToRecoveryPoint(move_base_msgs::RecoveryStatus msg, RecoveryPoint* pt);

  /**
   * Logs velocity commands sent to the robot whilst an demonstration is occuring
   */
  void teleopCallback(const geometry_msgs::Twist& msg);

  /**
   * Logs tf whilst an demonstration is occuring
   */
  void tfCallback(const tf2_msgs::TFMessage& msg);

  void velocityCallback(const geometry_msgs::Twist& msg);
};
}  // namespace recovery_supervisor
