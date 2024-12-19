#ifndef R4C_NAVIGATION_ACTION_PATH_BB_H
#define R4C_NAVIGATION_ACTION_PATH_BB_H

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/time.hpp>

class SetPathBB: public BT::CoroActionNode
{
  public:
  SetPathBB(const std::string& name, const BT::NodeConfiguration& conf);
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<nav_msgs::msg::Path>("path")};
  };
  BT::NodeStatus tick() override;

  private:
  geometry_msgs::msg::Pose pose_;
  geometry_msgs::msg::PoseStamped pose_stamped_;
  nav_msgs::msg::Path path_;
};

#endif
