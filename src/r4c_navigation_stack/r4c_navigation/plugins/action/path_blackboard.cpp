#include <r4c_navigation/action/path_blackboard.hpp>

SetPathBB::SetPathBB(const std::string& name, const BT::NodeConfiguration& conf):
  BT::CoroActionNode(name, conf)
{}

BT::NodeStatus SetPathBB::tick()
{
  path_.header.frame_id = "odom";
  path_.header.stamp.sec = rclcpp::Clock().now().seconds();
  path_.header.stamp.nanosec = rclcpp::Clock().now().nanoseconds();

  pose_.position.x = 1.0;
  pose_.position.y = 0.0;
  pose_.position.z = 0.0;

  pose_stamped_.pose = pose_;
  pose_stamped_.header.frame_id = "odom";
  pose_stamped_.header.stamp.sec = rclcpp::Clock().now().seconds();
  pose_stamped_.header.stamp.nanosec = rclcpp::Clock().now().nanoseconds();

  path_.poses.push_back(pose_stamped_);

  setOutput("path",path_);

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetPathBB>("SetPathBB");
}
