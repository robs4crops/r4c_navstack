#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <nav_msgs/msg/path.hpp>

#include "eut_crop_row_estimator/srv/get_path.hpp"

class GetPathService: public BT::SyncActionNode
{
  public:
  GetPathService(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout"),
            BT::InputPort<std::string>("service"),
            BT::OutputPort<nav_msgs::msg::Path>("path"),
            BT::OutputPort<bool>("path_updated")};
  };

  private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<eut_crop_row_estimator::srv::GetPath>::SharedPtr cli_;
  nav_msgs::msg::Path path_;
};