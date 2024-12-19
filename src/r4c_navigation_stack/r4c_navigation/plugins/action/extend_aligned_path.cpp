#include <r4c_navigation/action/extend_aligned_path.hpp>

ExtendAlignedPath::ExtendAlignedPath(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("extend_aligned_path");
  output_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("extended_path", 1);
}

BT::NodeStatus ExtendAlignedPath::tick()
{
  nav_msgs::msg::Path input_path;
  if(!getInput("input_path", input_path))
  {
    throw BT::RuntimeError("Missing required input [input_path]");
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  if(!getInput("goal_pose", goal_pose))
  {
    throw BT::RuntimeError("Missing required input [goal_pose]");
  }

  double alignment_extension;
  if(!getInput("alignment_extension", alignment_extension))
  {
    throw BT::RuntimeError("Missing required input [alignment_extension]");
  }

  geometry_msgs::msg::PoseStamped last_path_pose = input_path.poses.back(); 

  // Calculate orientation from current position to end of parcel
  double dx = last_path_pose.pose.position.x - goal_pose.pose.position.x;
  double dy = last_path_pose.pose.position.y - goal_pose.pose.position.y;
  double orientation = atan2(dy,dx); //* 180.0 / 3.1415926;
  
  double direction_x = -cos(orientation);
  double direction_y = -sin(orientation);

  nav_msgs::msg::Path output_path = input_path;

  for (float i = 0; i <= alignment_extension; i += 0.1) {
    last_path_pose.pose.position.x = input_path.poses.back().pose.position.x + (i * direction_x);
    last_path_pose.pose.position.y = input_path.poses.back().pose.position.y + (i * direction_y);
    output_path.poses.push_back(last_path_pose);
  }

  if(!setOutput<nav_msgs::msg::Path>("output_path", output_path))
  {
    throw BT::RuntimeError("Could not set output [output_path]");
  }

  output_path_publisher_->publish(output_path);

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ExtendAlignedPath>("ExtendAlignedPath");
}