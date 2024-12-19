#ifndef R4C_NAVIGATION_ACTIONS_GET_MANEUVER_ORIGIN_H
#define R4C_NAVIGATION_ACTIONS_GET_MANEUVER_ORIGIN_H

#include <rclcpp/rclcpp.hpp>
#include <r4c_navigation/bt_types.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <nav_2d_utils/conversions.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GetManeuverOrigin: public BT::SyncActionNode
{
  public:
  GetManeuverOrigin(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("parcel_origin"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("parcel_end"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("current_parcel_origin"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("current_parcel_end"),
            BT::InputPort<double>("man_exit_dist"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("man_origin_pose")};
  };
  rclcpp::Node::SharedPtr node_;
};

#endif  // R4C_NAVIGATION_ACTIONS_GET_MANEUVER_POSES_H
