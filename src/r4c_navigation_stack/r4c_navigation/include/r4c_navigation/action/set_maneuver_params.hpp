#ifndef R4C_NAVIGATION_ACTION_SET_MANEUVER_PARAMS_H
#define R4C_NAVIGATION_ACTION_SET_MANEUVER_PARAMS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>

#include <r4c_navigation/bt_types.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <Eigen/Core>


class SetManeuverParams: public BT::SyncActionNode
{
  public:
  SetManeuverParams(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("start"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::OutputPort<ManeuverParameters>("maneuver_parameters")};
  }

  BT::NodeStatus tick() override;

  private:
  geometry_msgs::msg::PoseStamped start_, goal_;
  ManeuverParameters maneuver_params_;

  std::string global_frame_{"map"};
  std::string base_frame_{"base_footprint"};

  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> ls_;
};

#endif  // R4C_NAVIGATION_ACTION_SET_MANEUVER_PARAMS_H