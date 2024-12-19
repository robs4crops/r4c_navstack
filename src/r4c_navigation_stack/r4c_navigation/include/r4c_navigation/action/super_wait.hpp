#ifndef R4C_NAVIGATION_ACTION_WAIT_AND_FAIL_H
#define R4C_NAVIGATION_ACTION_WAIT_AND_FAIL_H

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

class SuperWait: public BT::CoroActionNode
{
  public:
  SuperWait(const std::string& name, const BT::NodeConfiguration& conf);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout", 10.0, "Timeout to return Failure")};
  };
  BT::NodeStatus tick() override;

  private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double timeout_;
};

#endif
