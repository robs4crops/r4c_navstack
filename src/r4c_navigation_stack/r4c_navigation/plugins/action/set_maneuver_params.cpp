#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "r4c_navigation/action/set_maneuver_params.hpp"

SetManeuverParams::SetManeuverParams(const std::string& name,
                                     const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  // Get ROS parameters
  node_ = std::make_shared<rclcpp::Node>("set_maneuver_params_bt_action");
  node_->declare_parameter("global_frame", global_frame_);
  node_->declare_parameter("base_frame", base_frame_);
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("base_frame", base_frame_);

  buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  ls_     = std::make_shared<tf2_ros::TransformListener>(*buffer_);
}

BT::NodeStatus SetManeuverParams::tick()
{
  // Get initial and final row IDs.
  if(!getInput<geometry_msgs::msg::PoseStamped>("start", start_))
  {
    throw BT::RuntimeError("Missing required input [start]");
  }

  if(!getInput<geometry_msgs::msg::PoseStamped>("goal", goal_))
  {
    throw BT::RuntimeError("Missing required input [goal]");
  }

  tf2::Quaternion q;
  tf2::fromMsg(start_.pose.orientation, q);
  q.normalize();  
  double start_yaw = tf2::getYaw(q);
  tf2::fromMsg(goal_.pose.orientation, q);
  q.normalize();  
  double goal_yaw = tf2::getYaw(q);

  RCLCPP_WARN_STREAM(node_->get_logger(), "Setting parameters for maneuver from: " << start_.pose.position.x << ","<< start_.pose.position.y << "," << start_yaw << " to " << goal_.pose.position.x <<","<< goal_.pose.position.y<< "," << goal_yaw );

  maneuver_params_.global_frame      = global_frame_;
  maneuver_params_.type              = 1;
  maneuver_params_.flip_orientation  = false;
  maneuver_params_.exit_dist         = 7;
  maneuver_params_.entry_dist        = 9.1;
  // maneuver_params_.radius            = 3.8;
  maneuver_params_.radius            = 2.8;
  maneuver_params_.waypoint_step     = 0.2;
  maneuver_params_.direction = 1;

  if(!setOutput<ManeuverParameters>("maneuver_parameters", maneuver_params_))
  {
    throw BT::RuntimeError("Missing required output [maneuver_parameters]");
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully set maneuver parameters.");
  return BT::NodeStatus::SUCCESS;
}


#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetManeuverParams>("SetManeuverParams");
}
