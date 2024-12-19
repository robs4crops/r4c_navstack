#ifndef WAYPOINT_COLLECTOR_WAYPOINT_COLLECTOR_H_
#define WAYPOINT_COLLECTOR_WAYPOINT_COLLECTOR_H_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <nav2_msgs/srv/load_map.hpp>

class WaypointCollector: public rclcpp::Node
{
  public:
  WaypointCollector();
  bool start(const std_srvs::srv::Empty::Request::SharedPtr,
             const std_srvs::srv::Empty::Response::SharedPtr);
  bool stop(const std_srvs::srv::Empty::Request::SharedPtr,
            const std_srvs::srv::Empty::Response::SharedPtr);
  bool save(const nav2_msgs::srv::LoadMap::Request::SharedPtr request,
            const nav2_msgs::srv::LoadMap::Response::SharedPtr response);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void addWaypoint(const nav_msgs::msg::Odometry& odom,
                   std::vector<geometry_msgs::msg::PoseStamped>& waypoints);

  private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr save_srv_;

  double cov_tresh_;
  double waypoint_dist_;
  bool started_;
  std::string waypoints_directory_;

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  nav_msgs::msg::Odometry odom_msg_;
};

#endif