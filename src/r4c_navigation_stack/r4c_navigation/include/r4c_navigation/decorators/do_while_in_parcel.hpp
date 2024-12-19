#ifndef R4C_NAVIGATION_ACTIONS_DO_WHILE_IN_PARCEL_H
#define R4C_NAVIGATION_ACTIONS_DO_WHILE_IN_PARCEL_H

#include "behaviortree_cpp_v3/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

class DoWhileInParcel: public BT::DecoratorNode
{
  public:
  DoWhileInParcel(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("src_node"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("dst_node"),
            BT::InputPort<double>("row_width"),
            BT::InputPort<bool>("invert_logic")};
  };
  
  BT::NodeStatus tick() override;

  private:
  // @todo: set as parameters when moving to action server.
  std::string global_frame_{"map"};
  std::string base_frame_{"base_footprint"};

  std::vector<Eigen::Vector2d> coords_;
  bool invert_logic_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_bounds_;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> ls_;
};

#endif  // R4C_NAVIGATION_ACTIONS_DO_WHILE_IN_PARCEL_H
