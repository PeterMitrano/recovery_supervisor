#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>

#include "recovery_supervisor/recovery_supervisor.h"

namespace recovery_supervisor
{
RecoverySupervisor::RecoverySupervisor()
  : bag_index_(0)
  , first_recovery_count_(0)
  , starting_demonstration_(false)
  , ending_demonstration_(false)
  , demonstrating_(false)
  , has_goal_(false)
{
  // fetch parameters
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  private_nh.param<int>("finish_demonstration_button", finish_demonstration_button_, 7);
  private_nh.param<int>("force_demonstration_button", force_demonstration_button_, 6);
  private_nh.param<int>("maximum_first_recovery_count_recovery_count", maximum_first_recovery_count_, 3);

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
  recovery_status_sub_ = nh.subscribe("move_base/recovery_status", 10, &RecoverySupervisor::recoveryCallback, this);
  tf_sub_ = nh.subscribe("tf", 1, &RecoverySupervisor::tfCallback, this);

  cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", false);
  status_pub_ = private_nh.advertise<std_msgs::Bool>("demonstration_status", false);
  failure_location_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("failure_locations", false);

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

  std::string bag_name = bag_file_directory_ + "/" + std::to_string(bag_index_) + ".bag";

  ROS_INFO("finish_demonstration_button %d", finish_demonstration_button_);
  ROS_INFO("minimum_displacement %f", minimum_displacement_);
  ROS_INFO("maximum_first_recovery_count_recovery_count %d", maximum_first_recovery_count_);
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
    }

    if (ending_demonstration_)
    {
      ending_demonstration_ = false;
      demonstrating_ = false;

      stagnation_start_time_ = ros::Time::now();
      bag_mutex_.lock();
      bag_->close();
      bag_index_++;
      std::string bag_name = bag_file_directory_ + "/" + std::to_string(bag_index_) + ".bag";
      bag_->open(bag_name, rosbag::bagmode::Write);
      bag_mutex_.unlock();

      ROS_INFO("Demonstrations disabled.");
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
  ROS_INFO("Demonstrations enabled.");
}

void RecoverySupervisor::odometryCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO_ONCE("odom received.");
  if (!demonstrating_)
  {
    //check for localization jumps
    double displacement = dist(latest_pose_, msg.pose.pose);
    if (displacement > maximum_displacement_jump_)
    {
      ROS_ERROR("Localization failure: jumped by %f meters", displacement);
      actionlib_msgs::GoalID msg;
      msg.id = current_goal_id_;
      cancel_pub_.publish(msg);
    }

    latest_pose_.pose = msg.pose.pose;
    latest_pose_.header = msg.header;
  }

  double displacement_since_last_recovery = dist(latest_pose_, last_recovery_pose_);

  // if we've moved far enough, we must be making progress so reset recovery count
  if (displacement_since_last_recovery > minimum_displacement_)
  {
    first_recovery_count_ = 0;
  }
}

void RecoverySupervisor::recoveryCallback(const move_base_msgs::RecoveryStatus& msg)
{
  first_recovery_count_++;
  ROS_INFO("new recovery (%s). recovery count: %d", msg.name.c_str(), first_recovery_count_);

  // we allow for for the first straf recovery
  // if first straf fails to help, or we repeat first straf without
  // making significant progress
  if (first_recovery_count_ > maximum_first_recovery_count_ || msg.index > 0)
  {
    // here we want to enable demonstration in rviz
    starting_demonstration_ = true;

    bag_mutex_.lock();
    bag_->write("/recovery_supervisor/failure_locations", ros::Time::now(), latest_pose_);
    bag_mutex_.unlock();
    failure_location_pub_.publish(latest_pose_);
  }

  last_recovery_pose_ = latest_pose_;
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

double RecoverySupervisor::dist(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{
  tf::Stamped<tf::Point> tf_p1;
  tf::pointMsgToTF(p1.pose.position, tf_p1);

  tf::Stamped<tf::Point> tf_p2;
  tf::pointMsgToTF(p2.pose.position, tf_p2);

  return (tf_p2 - tf_p1).length();
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recovery_supervisor");

  recovery_supervisor::RecoverySupervisor supervisor;

  return EXIT_SUCCESS;
}
