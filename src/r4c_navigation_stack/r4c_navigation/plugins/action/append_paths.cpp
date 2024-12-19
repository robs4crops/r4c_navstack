#include <r4c_navigation/action/append_paths.hpp>

AppendPaths::AppendPaths(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("append_paths");
}

BT::NodeStatus AppendPaths::tick()
{
  nav_msgs::msg::Path path1;
  if(!getInput("path1", path1))
  {
    throw BT::RuntimeError("Missing required input [path1]");
  }

  nav_msgs::msg::Path path2;
  if(!getInput("path2", path2))
  {
    throw BT::RuntimeError("Missing required input [path2]");
  }

  nav_msgs::msg::Path output_path = path1;
  output_path.poses.insert(output_path.poses.end(), path2.poses.begin(), path2.poses.end());

  if(!setOutput("output_path", output_path))
  {
    throw BT::RuntimeError("Could not set output [output_path]");
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<AppendPaths>("AppendPaths");
}