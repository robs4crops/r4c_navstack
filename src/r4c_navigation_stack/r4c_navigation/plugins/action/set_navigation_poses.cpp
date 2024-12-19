#include <r4c_navigation/action/set_navigation_poses.hpp>

SetNavigationPoses::SetNavigationPoses(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("set_navigation_poses");
}

BT::NodeStatus SetNavigationPoses::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> input_poses;
  if(!getInput("input_goals", input_poses))
  {
    throw BT::RuntimeError("Missing required input [input_goals]");
  }

  geometry_msgs::msg::PoseStamped pose_a        = input_poses[0];
  geometry_msgs::msg::PoseStamped pose_b        = input_poses[1];
  geometry_msgs::msg::PoseStamped pose_c        = input_poses[2];
  geometry_msgs::msg::PoseStamped pose_d        = input_poses[3];

  if(!setOutput<geometry_msgs::msg::PoseStamped>("pose_a", pose_a))
  {
    throw BT::RuntimeError("Could not set output [pose_a]");
  }

  if(!setOutput<geometry_msgs::msg::PoseStamped>("pose_b", pose_b))
  {
    throw BT::RuntimeError("Could not set output [pose_b]");
  }

  if(!setOutput<geometry_msgs::msg::PoseStamped>("pose_c", pose_c))
  {
    throw BT::RuntimeError("Could not set output [pose_c]");
  }

  if(!setOutput<geometry_msgs::msg::PoseStamped>("pose_d", pose_d))
  {
    throw BT::RuntimeError("Could not set output [pose_d]");
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetNavigationPoses>("SetNavigationPoses");
}