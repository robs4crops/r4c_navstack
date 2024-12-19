#ifndef R4C_NAVIGATION_ACTIONS_SET_NAVIGATION_POSES_H
#define R4C_NAVIGATION_ACTIONS_SET_NAVIGATION_POSES_H

#include <rclcpp/rclcpp.hpp>
#include <r4c_navigation/bt_types.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <nav_2d_utils/conversions.hpp>

class SetNavigationPoses: public BT::SyncActionNode
{
  public:
  SetNavigationPoses(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_a"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_b"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_c"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_d")};
  };
  rclcpp::Node::SharedPtr node_;
};

#endif  // R4C_NAVIGATION_ACTIONS_SET_NAVIGATION_POSES_H
