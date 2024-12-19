#ifndef R4C_NAVIGATION_ACTIONS_GET_ALIGNMENT_POSE_H
#define R4C_NAVIGATION_ACTIONS_GET_ALIGNMENT_POSE_H

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

class GetAlignmentPose: public BT::SyncActionNode
{
  public:
  GetAlignmentPose(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("parcel_origin"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("parcel_end"),
            BT::InputPort<double>("vert_alignment_dist"),
            BT::InputPort<double>("lat_alignment_dist"),
            BT::InputPort<double>("alignment_orientation"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("alignment_pose")};
  };
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> ls_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr alignment_pose_publisher_;
};

#endif  // R4C_NAVIGATION_ACTIONS_GET_ALIGNMENT_POSE_H
