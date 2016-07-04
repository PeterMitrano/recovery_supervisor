#include <boost/filesystem.hpp>
#include <std_msgs/Bool.h>
#include <stdlib.h>

#include "recovery_supervisor/recovery_supervisor.h"

namespace recovery_supervisor
{
RecoverySupervisor::RecoverySupervisor()
  : bag_index_(0), straf_count_(0), starting_demonstration_(false), ending_demonstration_(false), demonstrating_(false), has_goal_(false)
{
  // fetch parameters
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  private_nh.param<int>("finish_demonstration_button", finish_demonstration_button_, 7);
  private_nh.param<int>("force_demonstration_button", force_demonstration_button_, 6);
  private_nh.param<int>("maximum_straf_count", maximum_straf_count_, 2);

  // controls how far the robot must move in stagnation_check_period (meters)
  private_nh.param<double>("minimum_displacement", minimum_displacement_, 2.0);

  std::string bag_file_directory_prefix;
  private_nh.param<std::string>("bag_file_directory_prefix", bag_file_directory_prefix, "my_bags");

  cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &RecoverySupervisor::teleopCallback, this);
  demo_path_sub_ = nh.subscribe("demo_path", 10, &RecoverySupervisor::demoPathCallback, this);
  footprint_sub_ = nh.subscribe("laser_footprint", 100, &RecoverySupervisor::footprintCallback, this);
  joy_sub_ = nh.subscribe("joy", 1, &RecoverySupervisor::joyCallback, this);
  local_costmap_sub_ =
      nh.subscribe("move_base/local_costmap/costmap", 100, &RecoverySupervisor::localCostmapCallback, this);
  local_costmap_update_sub_ = nh.subscribe("move_base/local_costmap/costmap_updates", 100,
                                           &RecoverySupervisor::localCostmapUpdateCallback, this);
  odom_sub_ = nh.subscribe("odom", 1, &RecoverySupervisor::odometryCallback, this);
  status_sub_ = nh.subscribe("move_base/status", 1, &RecoverySupervisor::moveBaseStatusCallback, this);
  straf_recovery_sub_ = nh.subscribe("straf_recovery_cycles", 10, &RecoverySupervisor::strafRecoveryCallback, this);
  tf_sub_ = nh.subscribe("tf", 1, &RecoverySupervisor::tfCallback, this);

  cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", false);
  status_pub_ = private_nh.advertise<std_msgs::Bool>("demonstration_status", false);
  failure_location_pub_ = private_nh.advertise<geometry_msgs::Pose>("failure_locations", false);

  bag_ = new rosbag::Bag();

  // we rant to group all bag files from this session into a folder
  // name the folder with time stamp
  time_t now = time(0);
  char buf[40];
  strftime(buf, sizeof(buf), "%d%m%Y-%H%M%S", localtime(&now));
  bag_file_directory_ += bag_file_directory_prefix;
  bag_file_directory_ += "-";
  bag_file_directory_ += buf;

  boost::filesystem::path boost_bag_dir(bag_file_directory_);
  if (!boost::filesystem::create_directory(boost_bag_dir))
  {
    ROS_ERROR("Failed to create directory %s", bag_file_directory_.c_str());
    return;
  }

  std::string bag_name = bag_file_directory_ + "/" +
    std::to_string(bag_index_) + ".bag";

  ROS_INFO("finish_demonstration_button %d", finish_demonstration_button_);
  ROS_INFO("minimum_displacement %f", minimum_displacement_);
  ROS_INFO("maximum_straf_count %d", maximum_straf_count_);
  ROS_INFO("bag_file_directory %s", bag_name.c_str());

  bag_->open(bag_name, rosbag::bagmode::Write);

  ros::Rate r(10);
  while (ros::ok())
  {
    if (starting_demonstration_)
    {
      notifyDemonstrator();
      starting_demonstration_ = false;
      has_goal_ = false;
      demonstrating_ = true;

      // log pose of failure and cancel goal

      failure_location_pub_.publish(latest_pose_);

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
      bag_->open(bag_file_directory_ + std::to_string(bag_index_) + ".bag", rosbag::bagmode::Write);
      bag_mutex_.unlock();

      ROS_INFO("Demonstration complete!");
    }

    std_msgs::Bool status;
    status.data = demonstrating_;
    status_pub_.publish(status);

    ros::spinOnce();
    r.sleep();
  }
}

void RecoverySupervisor::demoPathCallback(const nav_msgs::Path& msg)
{
  if (demonstrating_)
  {
  }
}

void RecoverySupervisor::footprintCallback(const geometry_msgs::PolygonStamped& msg)
{
  ROS_INFO_ONCE("Footprint received.");
  if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("laser_footprint", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

void RecoverySupervisor::joyCallback(const sensor_msgs::Joy& msg)
{
  ROS_INFO_ONCE("joystick received.");
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

void RecoverySupervisor::localCostmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO_ONCE("Local costmap received.");
  if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("move_base/local_costmap/costmap", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

void RecoverySupervisor::localCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate& msg)
{
  ROS_INFO_ONCE("Local costmap updates received.");
  if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("move_base/local_costmap/costmap_updates", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

void RecoverySupervisor::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& msg)
{
  ROS_INFO_ONCE("Movebase status received.");
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

void RecoverySupervisor::notifyDemonstrator()
{
  // do we want a full nav? or just until we're unlikely to get stuck anymore
  ROS_INFO("Help me! I'm stuck. Please navigate me to my goal.");
}

void RecoverySupervisor::odometryCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO_ONCE("odom received.");
  if (!demonstrating_)
  {
    latest_pose_ = msg.pose.pose;
  }
}

void RecoverySupervisor::strafRecoveryCallback(const std_msgs::Int32& msg)
{
  // this shouldn't happen, but just in case, ignore it
  if (demonstrating_ || !has_goal_)
  {
    return;
  }

  straf_count_++;

  ROS_INFO("straf recovery count: %d, my count: %d", msg.data, straf_count_);

  tf::Stamped<tf::Point> latest_tf_pose;
  tf::pointMsgToTF(latest_pose_.position, latest_tf_pose);

  tf::Stamped<tf::Point> last_straf_tf_pose;
  tf::pointMsgToTF(last_straf_location_, last_straf_tf_pose);

  double displacement_since_last_straf = (last_straf_tf_pose - latest_tf_pose).length();

  ROS_INFO("displacement_since_last_straf %f", displacement_since_last_straf);

  // if we've moved far enough, reset.
  if (displacement_since_last_straf > minimum_displacement_)
  {
    straf_count_ = 0;
  }

  last_straf_location_ = latest_pose_.position;

  if (straf_count_ > maximum_straf_count_)
  {
    // we've recovered the max number of allowed times in the same area
    // consider this failure and reset.
    ROS_WARN("too many strafs.");
    starting_demonstration_ = true;
    straf_count_ = 0;
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

void RecoverySupervisor::tfCallback(const tf2_msgs::TFMessage& msg)
{
  ROS_INFO_ONCE("tf received.");
  if (demonstrating_)
  {
    bag_mutex_.lock();
    bag_->write("tf", ros::Time::now(), msg);
    bag_mutex_.unlock();
  }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recovery_supervisor");

  recovery_supervisor::RecoverySupervisor supervisor;

  return EXIT_SUCCESS;
}
