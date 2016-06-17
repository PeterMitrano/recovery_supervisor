#include <geometry_msgs/Vector3.h>
#include <stdlib.h>

#include "recovery_supervisor/recovery_supervisor.h"

namespace recovery_supervisor
{
RecoverySupervisor::RecoverySupervisor()
  : starting_demonstration_(false)
  , ending_demonstration_(false)
  , demonstrating_(false)
  , has_goal_(false)
  , latest_goal_id_("uninitialized")
{
  // fetch parameters
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  private_nh.param<int>("finish_demonstration_button", finish_demonstration_button_, 7);

  // controls how long between checks for lack of progress (seconds)
  private_nh.param<double>("stagnation_check_period", stagnation_check_period_, 10);

  // controls how far the robot must move in stagnation_check_period (meters)
  private_nh.param<double>("minimum_displacement", minimum_displacement_, 2);

  cmd_vel_sub_ = private_nh.subscribe("cmd_vel", 10, &RecoverySupervisor::teleopCallback, this);

  odom_sub_ = nh.subscribe("odom", 10, &RecoverySupervisor::odometryCallback, this);
  joy_sub_ = nh.subscribe("joy", 1, &RecoverySupervisor::joyCallback, this);
  status_sub_ = nh.subscribe("move_base/status", 10, &RecoverySupervisor::moveBaseStatusCallback, this);
  goal_sub_ = nh.subscribe("move_base_simple/goal", 10, &RecoverySupervisor::moveBaseGoalCallback, this);
  local_costmap_sub_ =
      nh.subscribe("move_base/local_costmap/costmap_updates", 10, &RecoverySupervisor::localCostmapCallback, this);
  global_costmap_sub_ =
      nh.subscribe("move_base/global_costmap/costmap_updates", 10, &RecoverySupervisor::globalCostmapCallback, this);

  ROS_INFO("finish_demonstration_button %d", finish_demonstration_button_);
  ROS_INFO("stagnation_check_period %f", stagnation_check_period_);
  ROS_INFO("minimum_displacement %f", minimum_displacement_);

  stagnation_start_time_ = ros::Time::now();

  while (ros::ok())
  {
    if (starting_demonstration_)
    {
      notifyDemonstrator();
      starting_demonstration_ = false;
      has_goal_ = false;
      demonstrating_ = true;
    }

    if (ending_demonstration_)
    {
      ending_demonstration_ = false;
      demonstrating_ = false;

      stagnation_start_time_ = ros::Time::now();
      velocities_.clear();

      ROS_INFO("Demonstration complete!");
    }

    ros::spinOnce();
  }
}

void RecoverySupervisor::teleopCallback(const geometry_msgs::Twist& msg)
{
  if (demonstrating_)
  {
    velocities_.push_back(msg);
  }
}

void RecoverySupervisor::odometryCallback(const nav_msgs::Odometry& msg)
{
  if (!demonstrating_ && has_goal_)
  {
    ros::Time next_check_time = stagnation_start_time_ + ros::Duration(stagnation_check_period_);

    // check how far we've moved periodically
    if (msg.header.stamp > next_check_time)
    {
      tf::Pose current_pose;
      tf::poseMsgToTF(msg.pose.pose, current_pose);

      double displacement = (current_pose.getOrigin() - start_stagnation_pose_.getOrigin()).length();
      start_stagnation_pose_ = current_pose;
      stagnation_start_time_ = msg.header.stamp;

      // if it's not far enough, we are stuck.
      if (displacement < minimum_displacement_)
      {
        starting_demonstration_ = true;
        ROS_INFO("stagnation!");
      }
    }
  }
}

void RecoverySupervisor::localCostmapCallback(const map_msgs::OccupancyGridUpdate& msg)
{
}

void RecoverySupervisor::globalCostmapCallback(const map_msgs::OccupancyGridUpdate& msg)
{
}

// void MovingObstaclesCallback();
// void LabeledStaticFeaturesCallback();

void RecoverySupervisor::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& msg)
{
  if (demonstrating_ || msg.status_list.empty())
  {
    return;
  }

  int status = msg.status_list[0].status;
  std::string goal_id = msg.status_list[0].goal_id.id;

  if (status == actionlib_msgs::GoalStatus::ABORTED)
  {
    if (latest_goal_id_ != "uninitialized" && goal_id != latest_goal_id_)
    {
      starting_demonstration_ = true;
    }
  }
  else if (status == actionlib_msgs::GoalStatus::SUCCEEDED)
  {
    has_goal_ = false;
  }
  else if (status == actionlib_msgs::GoalStatus::ACTIVE)
  {
    latest_goal_id_ = goal_id;
  }
}

void RecoverySupervisor::joyCallback(const sensor_msgs::Joy& msg)
{
  if (demonstrating_)
  {
    if (msg.buttons.at(finish_demonstration_button_) == 1)
    {
      ending_demonstration_ = true;
    }
  }
}

void RecoverySupervisor::moveBaseGoalCallback(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("new goal");
  if (!demonstrating_)
  {
    has_goal_ = true;
    latest_goal_ = msg;
  }
}

void RecoverySupervisor::notifyDemonstrator()
{
  // do we want a full nav? or just until we're unlikely to get stuck anymore
  ROS_INFO("Help me! I'm stuck. Please navigate me to my goal.");
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recovery_supervisor");

  recovery_supervisor::RecoverySupervisor supervisor;

  return EXIT_SUCCESS;
}
