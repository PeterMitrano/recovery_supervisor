#include "recovery_supervisor/recovery_supervisor.h"

#include <boost/filesystem.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <chrono>
#include <recovery_supervisor_msgs/PosTimeGoalFeature.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <string>
#include <tf/transform_datatypes.h>

namespace recovery_supervisor
{
RecoverySupervisor::RecoverySupervisor()
  : bag_index_(0)
  , current_goal_id_(0)
  , first_recovery_count_(0)
  , starting_demonstration_(false)
  , ending_demonstration_(false)
  , demonstrating_(false)
  , has_goal_(false)
  , has_path_(false)
  , first_msg_(true)
  , first_amcl_msg_(true)
{
  {
    // fetch parameters
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    private_nh.param<int>("finish_demonstration_button", finish_demonstration_button_, 7);
    private_nh.param<int>("force_demonstration_button", force_demonstration_button_, 6);
    private_nh.param<int>("maximum_first_recovery_count_recovery_count", maximum_first_recovery_count_, 3);
    private_nh.param<double>("max_trip_time_error_factor", max_trip_time_error_factor_, 1.25);
    private_nh.param<double>("average_forward_velocity", average_forward_velocity_, 0.15);

    // controls how far the robot must move between recoveries
    private_nh.param<double>("minimum_displacement", minimum_displacement_, 2.0);

    // for checking localization errors
    private_nh.param<double>("maximum_displacement_jump", maximum_displacement_jump_, 2.0);

    std::string bag_file_directory_prefix;
    private_nh.param<std::string>("bag_file_directory_prefix", bag_file_directory_prefix, "my_bags");

    amcl_sub_ = nh.subscribe("amcl_pose", 10, &RecoverySupervisor::amclCallback, this);
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &RecoverySupervisor::teleopCallback, this);
    demo_path_sub_ = nh.subscribe("demo_path", 10, &RecoverySupervisor::demoPathCallback, this);
    global_path_sub_ = nh.subscribe("global_plan", 10, &RecoverySupervisor::globalPlanCallback, this);
    joy_sub_ = nh.subscribe("joy", 1, &RecoverySupervisor::joyCallback, this);
    new_goal_sub_ = nh.subscribe("nav_points/goal_id", 1, &RecoverySupervisor::newGoalCallback, this);
    odom_sub_ = nh.subscribe("odom", 1, &RecoverySupervisor::odometryCallback, this);
    status_sub_ = nh.subscribe("move_base/status", 1, &RecoverySupervisor::moveBaseStatusCallback, this);
    recovery_status_sub_ = nh.subscribe("move_base/recovery_status", 10, &RecoverySupervisor::recoveryCallback, this);
    //tf_sub_ = nh.subscribe("tf", 1, &RecoverySupervisor::tfCallback, this);
    //velocity_sub_ = nh.subscribe("velocity", 10, &RecoverySupervisor::velocityCallback, this);

    amcl_path_pub_ = private_nh.advertise<nav_msgs::Path>("amcl_path", false);
    cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", false);
    demo_pub_ = private_nh.advertise<recovery_supervisor_msgs::PosTimeGoalDemo>("demo", false);
    complete_demo_path_pub_ = private_nh.advertise<nav_msgs::Path>("complete_demo_path", false);
    cropped_path_pub_ = private_nh.advertise<nav_msgs::Path>("cropped_path", false);
    failure_location_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("failure_locations", false);
    recovery_cloud_pub_.advertise(private_nh, "recovery_cloud", false);
    status_pub_ = private_nh.advertise<std_msgs::Bool>("demonstration_status", false);
    state_feature_pub_ = private_nh.advertise<recovery_supervisor_msgs::PosTimeGoalFeature>("state_feature", false);

    bag_ = new rosbag::Bag();

    recovery_cloud_ = new pcl::PointCloud<RecoveryPoint>();
    recovery_cloud_->header.frame_id = "map";
    recovery_cloud_->points.clear();
    recovery_cloud_->width = 0;
    recovery_cloud_->height = 0;

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
  }

  std::string bag_name = bag_file_directory_ + "/" + std::to_string(bag_index_) + ".bag";

  ROS_INFO("finish_demonstration_button %d", finish_demonstration_button_);
  ROS_INFO("force_demonstration_button %d", force_demonstration_button_);
  ROS_INFO("minimum_displacement %f", minimum_displacement_);
  ROS_INFO("maximum_first_recovery_count_recovery_count %d", maximum_first_recovery_count_);
  ROS_INFO("bag_file_directory %s", bag_name.c_str());

  bag_->open(bag_name, rosbag::bagmode::Write);

  ros::Rate r(20);
  while (ros::ok())
  {
    if (has_goal_ && !demonstrating_)
    {
      // we want to continuously capture feature vectors so they can later be
      // used for training our weights. We do this at a fixed rate,
      // which should small enough to capture most changes in the parameters
      // but not unessecarily large. We use all the feature vectors from
      // the start of our plan until failure is detected to train.
      // This also means if you force start a demonstration, it will also stop
      // recording features at that point.
      recovery_supervisor_msgs::PosTimeGoalFeature feature_value;

      ////time of day in hours
      feature_value.hour = hourOfDay();
      // robot x, y, theta position
      feature_value.x = last_amcl_pose_.pose.position.x;
      feature_value.y = last_amcl_pose_.pose.position.y;
      feature_value.theta = tf::getYaw(last_amcl_pose_.pose.orientation);
      // goal id
      feature_value.goal = current_goal_id_;

      if (has_goal_)
      {
        current_demo_.feature_values.push_back(feature_value);
      }

      //also publish this as our current state
      state_feature_pub_.publish(feature_value);
    }

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

      bag_mutex_.lock();
      bag_->write("complete_demo_path", ros::Time::now(), current_demo_path_);
      bag_mutex_.unlock();

      // publish the now completed demo path
      // as well as the DemoPath message with all the info for learning
      complete_demo_path_pub_.publish(current_demo_path_);

      // for the Demo msg, we crop the amcl_path to match demo path.
      current_demo_.header.stamp = ros::Time::now();
      current_demo_.demo_path = current_demo_path_;
      current_demo_.odom_path = crop_path(current_demo_path_, current_amcl_path_);
      demo_pub_.publish(current_demo_);

      ROS_INFO("Demonstrations disabled.");
    }

    // publish if demo is enabled or not
    // and also recovery cloud (locations)
    // constantly for others to use or visualize
    std_msgs::Bool status;
    status.data = demonstrating_;
    status_pub_.publish(status);
    recovery_cloud_pub_.publish(*recovery_cloud_);

    ros::spinOnce();
    r.sleep();
  }
}

void RecoverySupervisor::amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  ROS_INFO_ONCE("amcl received.");
  //bag_mutex_.lock();
  //bag_->write("amcl_pose", ros::Time::now(), msg);
  //bag_mutex_.unlock();

  if (first_amcl_msg_)
  {
    first_amcl_msg_ = false;
  }
  else
  {
    // check for localization jumps
    geometry_msgs::PoseStamped curent_pose;
    curent_pose.pose = msg.pose.pose;
    curent_pose.header = msg.header;
    double displacement = dist(curent_pose, last_amcl_pose_);
    ROS_DEBUG("displacement %f", displacement);
    if (displacement > maximum_displacement_jump_)
    {
      ROS_ERROR("Localization failure: jumped by %f meters", displacement);
      actionlib_msgs::GoalID msg;
      msg.stamp = ros::Time(0);
      cancel_pub_.publish(msg);
    }
  }

  last_amcl_pose_.pose = msg.pose.pose;
  last_amcl_pose_.header = msg.header;

  if (!demonstrating_)
  {
    // assuming we haven't failed yet,
    // record our position as nav_msgs/Path
    current_amcl_path_.poses.push_back(last_amcl_pose_);
  }
}

nav_msgs::Path RecoverySupervisor::crop_path(const nav_msgs::Path demo_path, const nav_msgs::Path amcl_path)
{
  nav_msgs::Path new_path;
  new_path.header = amcl_path.header;

  amcl_path_pub_.publish(amcl_path);

  if (demo_path.poses.empty() || amcl_path.poses.empty())
  {
    return new_path;
  }

  // we find the point in the amcl_path closest to the starting point of the demo_path,
  // and only add points from the amcl_path after that.
  int start_index = indexOfClosestPose(demo_path.poses.front(), amcl_path);

  // now we find the point in the amcl_path closest to the end of the demo_path,
  // and only add points from the amcl_path before that.
  int end_index = indexOfClosestPose(demo_path.poses.back(), amcl_path);

  ROS_INFO("Cropping from %i (%f,%f) to %i (%f,%f)",
      start_index,
      amcl_path.poses[start_index].pose.position.x,
      amcl_path.poses[start_index].pose.position.y,
      end_index,
      amcl_path.poses[end_index].pose.position.x,
      amcl_path.poses[end_index].pose.position.y);

  // finally, add the poses to our new path an return
  for (int i = start_index; i <= end_index; i++)
  {
    new_path.poses.push_back(amcl_path.poses[i]);
  }

  cropped_path_pub_.publish(new_path);
  return new_path;
}

int RecoverySupervisor::hourOfDay()
{
  auto now = std::chrono::system_clock::now();
  time_t tt = std::chrono::system_clock::to_time_t(now);
  tm local_tm = *localtime(&tt);
  return local_tm.tm_hour;
}

int RecoverySupervisor::indexOfClosestPose(geometry_msgs::PoseStamped other_pose, const nav_msgs::Path path)
{
  int index = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < path.poses.size(); i++)
  {
    auto pose = path.poses[i];
    double d = dist(pose, other_pose);
    if (d < min_dist)
    {
      min_dist = d;
      index = i;
    }
    i++;
  }

  return index;
}

void RecoverySupervisor::demoPathCallback(const nav_msgs::Path& msg)
{
  if (demonstrating_)
  {
    current_demo_path_ = msg;
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

void RecoverySupervisor::globalPlanCallback(const nav_msgs::Path& msg)
{
  if (!has_path_)
  {
    has_path_ = true;

    // compute the length of the path
    double path_length;
    for (int i = 1; i < msg.poses.size(); i++)
    {
      geometry_msgs::PoseStamped previous_pose = msg.poses[i - 1];
      geometry_msgs::PoseStamped pose = msg.poses[i];
      path_length += dist(pose, previous_pose);
    }

    // compute ETA
    // m / (m/s) = s
    estimate_trip_time_ = ros::Duration(path_length / average_forward_velocity_, 0);
    ROS_DEBUG("estimated trip_time: %fs", estimate_trip_time_.toSec());
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

void RecoverySupervisor::newGoalCallback(const std_msgs::Int32& msg)
{
  ROS_INFO_ONCE("nav goal recieved.");
  has_goal_ = true;
  has_path_ = false;
  bag_mutex_.lock();
  bag_->write("nav_points/goal_id", ros::Time::now(), msg);
  bag_mutex_.unlock();

  current_goal_id_ = msg.data;

  // reset current demo and related messages
  current_demo_.header.stamp = ros::Time::now();
  current_demo_.feature_values.clear();
  current_amcl_path_.poses.clear();
  current_amcl_path_.header.stamp = ros::Time::now();
  current_amcl_path_.header.frame_id = "map";

  trip_time_start_time_ = ros::Time::now();
}

void RecoverySupervisor::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray& msg)
{
  ROS_INFO_ONCE("Movebase status received.");

  if (first_msg_)
  {
    first_msg_ = false;
    if (msg.status_list.size() > 0)
    {
      current_movebase_goal_ = msg.status_list[0].goal_id.id;
    }
  }

  actionlib_msgs::GoalStatus oldest_msg = msg.status_list[0];
  int status = oldest_msg.status;
  std::string goal_id = oldest_msg.goal_id.id;

  if (status == actionlib_msgs::GoalStatus::ABORTED)
  {
    if (has_goal_)
    {
      ROS_WARN("Goal was aborted.");
      starting_demonstration_ = true;
    }
    has_goal_ = false;
    current_movebase_goal_ = goal_id;
  }
  else if (status == actionlib_msgs::GoalStatus::SUCCEEDED)
  {
    if (current_movebase_goal_ != goal_id)
    {
      has_goal_ = false;

      // we successfully reached our goal! bag it.
      ROS_DEBUG("Goal reached!");
      bag_mutex_.lock();
      bag_->write("goals_reached", ros::Time::now(), latest_pose_);
      bag_mutex_.unlock();

      // now check if that took too long compared to our trip_time
      auto actual_trip_time = ros::Time::now() - trip_time_start_time_;
      auto max_allowed_trip_time = estimate_trip_time_ * max_trip_time_error_factor_;
      if (actual_trip_time > max_allowed_trip_time)
      {
        ROS_WARN("max allow time: %fs, actual time: %fs", max_allowed_trip_time.toSec(), actual_trip_time.toSec());
        // starting_demonstration_ = true;
      }

      current_movebase_goal_ = goal_id;
    }
  }
}

void RecoverySupervisor::notifyDemonstrator()
{
  ROS_INFO("Demonstrations enabled.");
}

void RecoverySupervisor::odometryCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO_ONCE("odom received.");

  //bag_mutex_.lock();
  //bag_->write("odom", ros::Time::now(), msg);
  //bag_mutex_.unlock();

  latest_pose_.pose = msg.pose.pose;
  latest_pose_.header = msg.header;

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

  // update my "point cloud" of recovery locations
  RecoveryPoint pt;
  recoveryStatsMsgToRecoveryPoint(msg, &pt);
  recovery_cloud_->push_back(pt);

  bag_mutex_.lock();
  bag_->write("/recovery_supervisor/recovery_locations", ros::Time::now(), msg);
  bag_mutex_.unlock();

  // we allow for for the first straf recovery
  // if first straf fails to help, or we repeat first straf without
  // making significant progress
  if (first_recovery_count_ > maximum_first_recovery_count_ || msg.index > 0)
  {
    // here we want to enable demonstration in rviz
    starting_demonstration_ = true;

    failure_location_pub_.publish(latest_pose_);
  }

  last_recovery_pose_ = latest_pose_;
}

void RecoverySupervisor::recoveryStatsMsgToRecoveryPoint(move_base_msgs::RecoveryStatus msg, RecoveryPoint* pt)
{
  pt->x = msg.pose_stamped.pose.position.x;
  pt->y = msg.pose_stamped.pose.position.y;
  pt->z = msg.pose_stamped.pose.position.z;

  pt->ox = msg.pose_stamped.pose.orientation.x;
  pt->oy = msg.pose_stamped.pose.orientation.y;
  pt->oz = msg.pose_stamped.pose.orientation.z;
  pt->ow = msg.pose_stamped.pose.orientation.w;

  pt->index = msg.index;
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
  bag_mutex_.lock();
  bag_->write("tf", ros::Time::now(), msg);
  bag_mutex_.unlock();
}

void RecoverySupervisor::velocityCallback(const geometry_msgs::Twist& msg)
{
  ROS_INFO_ONCE("velocity received.");
  bag_mutex_.lock();
  bag_->write("velocity", ros::Time::now(), msg);
  bag_mutex_.unlock();
  last_velocity_ = msg;
}

}  // namespace recovery_supervisor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recovery_supervisor");

  recovery_supervisor::RecoverySupervisor supervisor;

  return EXIT_SUCCESS;
}
