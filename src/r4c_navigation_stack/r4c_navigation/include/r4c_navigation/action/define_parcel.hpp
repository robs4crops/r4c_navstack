#ifndef R4C_NAVIGATION_ACTIONS_DEFINE_PARCEL_H
#define R4C_NAVIGATION_ACTIONS_DEFINE_PARCEL_H

#include <rclcpp/rclcpp.hpp>
#include <r4c_navigation/bt_types.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <nav_2d_utils/conversions.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DefineParcel: public BT::SyncActionNode
{
  public:
  DefineParcel(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("enter_pose"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("exit_pose"),
            BT::InputPort<double>("enter_dist"),
            BT::InputPort<double>("exit_dist"),
            BT::InputPort<double>("row_width"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("parcel_origin"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("parcel_end"),
            BT::OutputPort<double>("parcel_width")};
  };
  rclcpp::Node::SharedPtr node_;
};

#endif  // R4C_NAVIGATION_ACTIONS_DEFINE_PARCEL_H
