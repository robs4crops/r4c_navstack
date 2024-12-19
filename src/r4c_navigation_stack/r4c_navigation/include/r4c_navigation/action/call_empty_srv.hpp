#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <std_srvs/srv/empty.hpp>

class CallEmptySrv: public BT::SyncActionNode
{
  public:
  CallEmptySrv(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("service_name")};
  };

  private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_;
  std::string service_name_;
};