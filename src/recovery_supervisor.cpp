#include <geometry_msgs/Vector3.h>
#include <stdlib.h>

#include "recovery_supervisor/recovery_supervisor.h"

namespace recovery_supervisor
{
RecoverySupervisor::RecoverySupervisor()
  : starting_demonstration_(false), ending_demonstration_(false), demonstrating_(false), has_goal_(false), bag_index_(0)
{
  // fetch parameters
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  private_nh.param<int>("finish_demonstration_button", finish_demonstration_button_, 7);
  private_nh.param<int>("force_demonstration_button", force_demonstration_button_, 6);

  // controls how long between checks for lack of progress (seconds)
  private_nh.param<double>("stagnation_check_period", stagnation_check_period_, 20);

  // controls how far the robot must move in stagnation_check_period (meters)
  private_nh.param<double>("minimum_displacement", minimum_displacement_, 1.5);

  private_nh.param<std::string>("bag_file_name", bag_file_name_, "mybag");

  cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &RecoverySupervisor::teleopCallback, this);

  tf_sub_ = nh.subscribe("tf", 1, &RecoverySupervisor::tfCallback, this);
  odom_sub_ = nh.subscribe("odom", 1, &RecoverySupervisor::odometryCallback, this);
  joy_sub_ = nh.subscribe("joy", 1, &RecoverySupervisor::joyCallback, this);
  status_sub_ = nh.subscribe("move_base/status", 1, &RecoverySupervisor::moveBaseStatusCallback, this);
  local_costmap_sub_ =
      nh.subscribe("move_base/local_costmap/costmap_updates", 10, &RecoverySupervisor::localCostmapCallback, this);
  global_costmap_sub_ =
      nh.subscribe("move_base/global_costmap/costmap_updates", 10, &RecoverySupervisor::globalCostmapCallback, this);

  cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", false);

  bag_ = new rosbag::Bag();
  bag_->open(bag_file_name_ + std::to_string(bag_index_) + ".bag", rosbag::bagmode::Write);

  ROS_INFO("finish_demonstration_button %d", finish_demonstration_button_);
  ROS_INFO("stagnation_check_period %f", stagnation_check_period_);
  ROS_INFO("minimum_displacement %f", minimum_displacement_);
  ROS_INFO("bag_file_name %s", bag_->getFileName().c_str());

  while (ros::ok())
  {
    if (starting_demonstration_)
    {
      notifyDemonstrator();
      starting_demonstration_ = false;
      has_goal_ = false;
      demonstrating_ = true;

      actionlib_msgs::GoalID cancel_msg;
      cancel_msg.stamp = ros::Time(0);
      cancel_pub_.publish(cancel_msg);
    }

    if (ending_demonstration_)
    {
      ending_demonstration_ = false;
      demonstrating_ = false;

      stagnation_start_time_ = ros::Time::now();
      bag_mutex_.lock();
      bag_->close();
      bag_index_++;
      bag_->open(bag_file_name_ + std::to_string(bag_index_) + ".bag", rosbag::bagmode::Write);
      bag_mutex_.unlock();

      ROS_INFO("Demonstration complete!");
    }

    ros::spinOnce();
  }
}

void RecoverySupervisor::teleopCallback(const geometry_msgs::Twist& msg)
{
  if (demonstrating_)
  {
    double magnitude = hypot(msg.linear.x, msg.linear.y);
    if (magnitude != 0.0 || msg.angular.z != 0.0)
    {
      bag_mutex_.lock();
      bag_->write("cmd_vel", ros::Time::now(), msg);
      bag_mutex_.unlock();
    }
  }
}

void RecoverySupervisor::odometryCallback(const nav_msgs::Odometry& msg)
{
  if (!demonstrating_ && has_goal_)
  {
    ros::Time next_check_time = stagnation_start_time_ + ros::Duration(stagnation_check_period_);
    ros::Time current_time = ros::Time::now();

    // check how far we've moved periodically
    if (current_time > next_check_time)
    {
      tf::Pose current_pose;
      tf::poseMsgToTF(msg.pose.pose, current_pose);

      double displacement = (current_pose.getOrigin() - start_stagnation_pose_.getOrigin()).length();
      start_stagnation_pose_ = current_pose;
      stagnation_start_time_ = current_time;

      // if it's not far enough, we are stuck.
      if (displacement < minimum_displacement_)
      {
        starting_demonstration_ = true;
        ROS_INFO("HEARTBEAT: dead");
      }
      else
      {
        ROS_INFO("HEARTBEAT: Still alive");
      }
    }
  }
  else if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("odom", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

void RecoverySupervisor::localCostmapCallback(const map_msgs::OccupancyGridUpdate& msg)
{
  if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("move_base/local_costmap/costmap_updates", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

void RecoverySupervisor::globalCostmapCallback(const map_msgs::OccupancyGridUpdate& msg)
{
  if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("move_base/global_costmap/costmap_updates", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

void RecoverySupervisor::tfCallback(const tf2_msgs::TFMessage& msg)
{
  if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("/tf", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

// void MovingObstaclesCallback();
// void LabeledStaticFeaturesCallback();

void RecoverySupervisor::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& msg)
{
  if (demonstrating_ || msg.status_list.empty())
  {
    return;
  }

  for (unsigned long i = 0; i < msg.status_list.size(); i++)
  {
    int status = msg.status_list[i].status;
    std::string goal_id = msg.status_list[i].goal_id.id;

    if (status == actionlib_msgs::GoalStatus::ACTIVE)
    {
      if (goal_id != current_goal_id_)
      {
        has_goal_ = true;
        stagnation_start_time_ = ros::Time::now();
        current_goal_id_ = goal_id;
      }
    }
    else if (status == actionlib_msgs::GoalStatus::ABORTED)
    {
      if (has_goal_)
      {
        starting_demonstration_ = true;
      }
    }
    else if (status == actionlib_msgs::GoalStatus::SUCCEEDED)
    {
      if (has_goal_ && current_goal_id_ == goal_id)
      {
        has_goal_ = false;
      }
    }
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
  else if (msg.buttons.at(force_demonstration_button_) == 1)
  {
    starting_demonstration_ = true;
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
