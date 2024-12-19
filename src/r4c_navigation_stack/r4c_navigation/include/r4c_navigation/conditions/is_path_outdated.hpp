#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <nav_msgs/msg/path.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>

class IsPathOutdated: public BT::ConditionNode
{
  public:
  IsPathOutdated(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("path_timeout"), BT::InputPort<nav_msgs::msg::Path>("path")};
  };

  BT::NodeStatus tick() override;

  private:
  rclcpp::Node::SharedPtr node_;
  rcl_duration_value_t path_timeout_ns_;
  nav_msgs::msg::Path path_;
};