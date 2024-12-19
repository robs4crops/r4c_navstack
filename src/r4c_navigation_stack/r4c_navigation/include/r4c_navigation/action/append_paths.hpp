#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <nav_msgs/msg/path.hpp>

class AppendPaths: public BT::SyncActionNode
{
  public:
  AppendPaths(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path1"),
            BT::InputPort<nav_msgs::msg::Path>("path2"),
            BT::OutputPort<nav_msgs::msg::Path>("output_path")};
  };

  private:
  std::shared_ptr<rclcpp::Node> node_;
};