#include <geometry_msgs/Vector3.h>
#include <stdlib.h>

#include "recovery_supervisor/recovery_supervisor.h"

namespace recovery_supervisor
{
RecoverySupervisor::RecoverySupervisor()
  : starting_demonstration_(false), ending_demonstration_(false), demonstrating_(false)
{
  // fetch parameters
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  private_nh.param<int>("finish_demonstration_button", finish_demonstration_button_, 7);

  // controls how long between checks for lack of progress (seconds)
  private_nh.param<double>("stagnation_check_period", stagnation_check_period_, 10);

  // controls how far the robot must move in stagnation_check_period (meters)
  private_nh.param<double>("minimum_displacement", minimum_displacement_, 2);

  private_nh.subscribe("cmd_vel", 10, &RecoverySupervisor::teleopCallback, this);

  odom_sub_ = nh.subscribe("odom", 10, &RecoverySupervisor::odometryCallback, this);
  joy_sub_ = nh.subscribe("joy", 1, &RecoverySupervisor::joyCallback, this);
  status_sub_ = nh.subscribe("move_base/status", 10, &RecoverySupervisor::moveBaseStatusCallback, this);
  local_costmap_sub_ = nh.subscribe("move_base/local_costmap/costmap_updates", 10, &RecoverySupervisor::localCostmapCallback, this);
  global_costmap_sub_ = nh.subscribe("move_base/global_costmap/costmap_updates", 10, &RecoverySupervisor::globalCostmapCallback, this);

  ROS_INFO("finish_demonstration_button %d", finish_demonstration_button_);
  ROS_INFO("stagnation_check_period %f", stagnation_check_period_);

  stagnation_start_time_ = ros::Time::now();

  while (ros::ok())
  {
    if (starting_demonstration_)
    {
      notifyDemonstrator();
      starting_demonstration_ = false;
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
  if (demonstrating_)
  {
    return;
  }

  ros::Time next_check_time = stagnation_start_time_ + ros::Duration(stagnation_check_period_);

  //check how far we've moved periodically
  if (msg.header.stamp > next_check_time)
  {
    tf::Pose current_pose;
    tf::poseMsgToTF(msg.pose.pose, current_pose);

    double displacement = (current_pose.getOrigin() - start_stagnation_pose_.getOrigin()).length();
    start_stagnation_pose_ = current_pose;
    stagnation_start_time_ = msg.header.stamp;

    //if it's not far enough, we are stuck.
    if (displacement < minimum_displacement_)
    {
      starting_demonstration_ = true;
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
  if (msg.status_list.empty())
  {
    return;
  }

  int status = msg.status_list[0].status;

  if (status == actionlib_msgs::GoalStatus::ABORTED || status == actionlib_msgs::GoalStatus::LOST ||
      status == actionlib_msgs::GoalStatus::REJECTED)
  {
    // we consider all these to be cause for intervention
    starting_demonstration_ = true;
  }
}

void RecoverySupervisor::joyCallback(const sensor_msgs::Joy& msg)
{
  if (msg.buttons.at(finish_demonstration_button_) == 1)
  {
    ending_demonstration_ = true;
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
