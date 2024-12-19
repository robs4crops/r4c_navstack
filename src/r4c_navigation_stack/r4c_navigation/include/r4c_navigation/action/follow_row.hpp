#ifndef R4C_NAVIGATION_ACTIONS_FOLLOW_ROW_H
#define R4C_NAVIGATION_ACTIONS_FOLLOW_ROW_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

/*
  Follow row action.
  Creates a path navigation action server on initialization.
  When ticked reads the current robot state (line/orientation) and extracts the
  coordinates up to where the robot should be in "row following mode". Then, it
  sends a goal to the action server to start row navigation.
*/
class FollowRow: public BT::StatefulActionNode
{
  public:
  FollowRow(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::Pose2D>("src_node"),
            BT::InputPort<geometry_msgs::msg::Pose2D>("dst_node")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  private:
  std::string global_frame_{"robot_map"};
  std::string robot_frame_{"robot_base_footprint"};
  std::string path_navigation_action_name_{"path_navigation"};
  std::string pure_pursuit_local_planner_{"pure_pursuit_local_planner/PurePursuit"};
  std::string reactive_navigation_topic_{"/central_line"};

  double reactive_navigation_max_vel_lin_{0.6};
  double reactive_navigation_max_vel_ang_{0.4};

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> ls_;

  geometry_msgs::msg::Pose2D src_node_;
  geometry_msgs::msg::Pose2D dst_node_;

  bool end_reached_{false};
  bool send_goal_{true};
};

#endif  // R4C_NAVIGATION_ACTIONS_FOLLOW_ROW_H