#include <r4c_navigation/action/load_control_point.hpp>

LoadControlPoint::LoadControlPoint(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{}

BT::NodeStatus LoadControlPoint::tick()
{
  // Construct ROS-equivalent of PoseStamped.
  PoseStamped input;
  if(!getInput<PoseStamped>("input", input))
  {
    throw BT::RuntimeError("Missing required input [input]");
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = input.frame_id;
  pose.pose.position.x = input.x;
  pose.pose.position.y = input.y;
  pose.pose.orientation.z = sin(input.theta / 2);
  pose.pose.orientation.w = cos(input.theta / 2);  

  if(!setOutput<geometry_msgs::msg::PoseStamped>("pose", pose))
  {
    throw BT::RuntimeError("Could not set output [pose]");
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LoadControlPoint>("LoadControlPoint");
}