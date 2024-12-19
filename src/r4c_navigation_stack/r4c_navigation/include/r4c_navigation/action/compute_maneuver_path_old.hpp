#ifndef R4C_NAVIGATION_ACTION_GENERATE_MANEUVER_H
#define R4C_NAVIGATION_ACTION_GENERATE_MANEUVER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>

#include <r4c_navigation/bt_types.hpp>
#include <eut_maneuver_generation/srv/generate_maneuver.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <Eigen/Core>


/* Turning behavior from one row to another.
   Receives as input the global/robot frames and maneuver generation parameters.
   Generates maneuver path to be followed by the robot.
*/
class ComputeManeuverPath: public BT::SyncActionNode
{
  public:
  ComputeManeuverPath(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("start"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::InputPort<ManeuverParameters>("maneuver_parameters"),
            BT::OutputPort<nav_msgs::msg::Path>("path")};
  }

  BT::NodeStatus tick() override;

  private:
  geometry_msgs::msg::PoseStamped start_, goal_;
  ManeuverParameters maneuver_params_;

  std::string global_frame_{"map"};
  std::string base_frame_{"base_footprint"};
  std::string maneuver_generation_server_name_{"/generate_maneuver"};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<eut_maneuver_generation::srv::GenerateManeuver>::SharedPtr cli_;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> ls_;
};

#endif  // R4C_NAVIGATION_ACTION_GENERATE_MANEUVER_H