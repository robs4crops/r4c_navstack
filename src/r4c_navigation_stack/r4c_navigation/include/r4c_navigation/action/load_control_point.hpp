#ifndef R4C_NAVIGATION_ACTIONS_LOAD_CONTROL_POINT_H
#define R4C_NAVIGATION_ACTIONS_LOAD_CONTROL_POINT_H

#include <rclcpp/rclcpp.hpp>
#include <r4c_navigation/bt_types.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

// This action exists because nav2 standard actions (e.g. ComputePathToPose)
// do not include the "convertFromString" interfaces defined in bt_conversions.hpp.

class LoadControlPoint: public BT::SyncActionNode
{
  public:
  LoadControlPoint(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<PoseStamped>("input"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")};
  };
};

#endif  // R4C_NAVIGATION_ACTIONS_LOAD_CONTROL_POINT_H
