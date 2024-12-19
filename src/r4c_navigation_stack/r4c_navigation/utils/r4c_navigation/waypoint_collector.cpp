#include <fstream>
#include <iostream>
#include <iomanip>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eut_nav_utils/nav_utils.h>
#include <r4c_navigation/waypoint_collector.hpp>

WaypointCollector::WaypointCollector(): rclcpp::Node("WaypointCollector"), started_(false)
{
  cov_tresh_     = 0.01;
  waypoint_dist_ = 1.0;

  // Load parameters
  waypoint_dist_       = this->declare_parameter("waypoint_distance", 1.0);
  cov_tresh_           = this->declare_parameter("covariance_treshold", 0.01);
  waypoints_directory_ = this->declare_parameter("waypoints_directory", "");

  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 1);

  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    1,
    std::bind(&WaypointCollector::odomCallback, this, std::placeholders::_1));

  // Advertise services
  start_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/start",
    std::bind(&WaypointCollector::start, this, std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/stop",
    std::bind(&WaypointCollector::stop, this, std::placeholders::_1, std::placeholders::_2));
  save_srv_ = this->create_service<nav2_msgs::srv::LoadMap>(
    "~/save",
    std::bind(&WaypointCollector::save, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "[WaypointCollector] Initialized!");
}

bool WaypointCollector::start(const std_srvs::srv::Empty::Request::SharedPtr,
                              const std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "[WaypointCollector] Starting");
  // Clear waypoints vector
  waypoints_.clear();
  addWaypoint(odom_msg_, waypoints_);
  started_ = true;
  return true;
}

bool WaypointCollector::stop(const std_srvs::srv::Empty::Request::SharedPtr,
                             const std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "[WaypointCollector] Stopping");
  // Save last waypoint
  addWaypoint(odom_msg_, waypoints_);
  started_ = false;
  return true;
}

bool WaypointCollector::save(const nav2_msgs::srv::LoadMap::Request::SharedPtr request,
                             const nav2_msgs::srv::LoadMap::Response::SharedPtr response)
{
  if(waypoints_.empty())
  {
    RCLCPP_WARN(this->get_logger(),
                "[WaypointCollector] No waypoints have been collected. Could not save to file");
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    return false;
  }

  // TODO: If file already exists trigger warning and ask for user feedback to override file or
  // write a new file name

  // Check if the provided url is not empty
  if(request->map_url.empty())
  {
    RCLCPP_WARN(this->get_logger(),
                "[WaypointCollector] No url provided. Unable to save waypoints!");
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    return false;
  }

  std::string waypoints_full_path = waypoints_directory_ + "/" + request->map_url + ".csv";
  std::ofstream waypoints_file(waypoints_full_path.c_str());
  if(waypoints_file.is_open())
  {
    for(size_t i = 0; i < waypoints_.size(); i++)
    {
      // Work in rpy rather than quat, more intuitive
      tf2::Quaternion q(waypoints_[i].pose.orientation.x,
                        waypoints_[i].pose.orientation.y,
                        waypoints_[i].pose.orientation.z,
                        waypoints_[i].pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      waypoints_file << std::fixed << std::setprecision(2) << waypoints_[i].pose.position.x << ";"
                     << waypoints_[i].pose.position.y << ";" << yaw << std::endl;
    }

    RCLCPP_INFO(this->get_logger(),
                "[WaypointCollector] Saved %zu waypoints to file: %s",
                waypoints_.size(),
                request->map_url.c_str());
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[WaypointCollector] Unable to save waypoints to file.");
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    return false;
  }

  response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  return true;
}

void WaypointCollector::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Update odometry message
  odom_msg_ = *msg;

  // Add new waypoints after moving X meters from the last waypoint
  if(started_ &&
     nav_utils::euclideanDistance(msg->pose.pose, waypoints_.back().pose) >= waypoint_dist_)
  {
    // Add pose to path
    addWaypoint(odom_msg_, waypoints_);
  }
}

void WaypointCollector::addWaypoint(const nav_msgs::msg::Odometry& odom,
                                    std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
  if(odom.pose.covariance[0] < cov_tresh_)
  {  // Create pose stamped message with current odometry values
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose   = odom.pose.pose;
    // Append pose to path
    waypoints.push_back(ps);

    // Every time a new waypoint is added, publish waypoints vector as path
    // Create a path message
    nav_msgs::msg::Path gui_path;
    gui_path.poses.resize(waypoints_.size());
    gui_path.header.frame_id = waypoints[0].header.frame_id;
    gui_path.header.stamp    = waypoints[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(size_t i = 0; i < waypoints_.size(); i++)
    {
      gui_path.poses[i] = waypoints_[i];
    }

    path_pub_->publish(gui_path);
  }
  else
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Could not save waypoint due to invalid status");
  }
}