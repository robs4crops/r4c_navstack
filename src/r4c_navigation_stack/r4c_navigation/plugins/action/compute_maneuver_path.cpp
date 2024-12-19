#include <r4c_navigation/action/compute_maneuver_path.hpp>

ComputeManeuverPath::ComputeManeuverPath(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("compute_maneuver_path");
}

BT::NodeStatus ComputeManeuverPath::tick()
{
  nav_msgs::msg::Path input_path;
  if(!getInput("input_path", input_path))
  {
    throw BT::RuntimeError("Missing required input [input_path]");
  }

  geometry_msgs::msg::PoseStamped current_parcel_end;
  if(!getInput("current_parcel_end", current_parcel_end))
  {
    throw BT::RuntimeError("Missing required input [current_parcel_end]");
  }

  nav_msgs::msg::Path output_path = input_path;

  // Add for exit alignment
  for (int i = 0; i <= 10; ++i) {
    double ratio = static_cast<double>(i) / 10.0;
    geometry_msgs::msg::PoseStamped pose;
    pose = input_path.poses.front();

    pose.pose.position.x = input_path.poses.front().pose.position.x + ratio * (current_parcel_end.pose.position.x - input_path.poses.front().pose.position.x);
    pose.pose.position.y = input_path.poses.front().pose.position.y + ratio * (current_parcel_end.pose.position.y - input_path.poses.front().pose.position.y);

    output_path.poses.insert(output_path.poses.begin(),pose);
  }

  if(!setOutput<nav_msgs::msg::Path>("output_path", output_path))
  {
    throw BT::RuntimeError("Could not set output [output_path]");
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ComputeManeuverPath>("ComputeManeuverPath");
}