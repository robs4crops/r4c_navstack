#ifndef R4C_NAVIGATION_ACTIONS_COMPUTE_MANEUVER_PATH_H
#define R4C_NAVIGATION_ACTIONS_COMPUTE_MANEUVER_PATH_H

#include <rclcpp/rclcpp.hpp>
#include <r4c_navigation/bt_types.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <nav_2d_utils/conversions.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ComputeManeuverPath: public BT::SyncActionNode
{
  public:
  ComputeManeuverPath(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("input_path"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("current_parcel_end"),
            BT::OutputPort<nav_msgs::msg::Path>("output_path")};
  };
  rclcpp::Node::SharedPtr node_;
};

#endif  // R4C_NAVIGATION_ACTIONS_COMPUTE_MANEUVER_PATH_H
