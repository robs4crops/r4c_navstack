#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "r4c_navigation/action/compute_maneuver_path.hpp"

ComputeManeuverPath::ComputeManeuverPath(const std::string& name,
                                         const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  // Get ROS parameters
  node_ = std::make_shared<rclcpp::Node>("generate_maneuver_bt_action");
  node_->declare_parameter("global_frame", global_frame_);
  node_->declare_parameter("base_frame", base_frame_);
  node_->declare_parameter("maneuver_generation_server_name", maneuver_generation_server_name_);
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("base_frame", base_frame_);
  node_->get_parameter("maneuver_generation_server_name", maneuver_generation_server_name_);

  buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  ls_     = std::make_shared<tf2_ros::TransformListener>(*buffer_);

  // Initialize manuever_generator service client
  cli_ = node_->create_client<eut_maneuver_generation::srv::GenerateManeuver>(
    maneuver_generation_server_name_);
}

BT::NodeStatus ComputeManeuverPath::tick()
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

  // Get maneuver parameters.
  if(!getInput<ManeuverParameters>("maneuver_parameters", maneuver_params_))
  {
    throw BT::RuntimeError("Missing required input [maneuver_parameters]");
  }

  RCLCPP_WARN_STREAM(node_->get_logger(), "Maneuver from: " << start_.pose.position.x << ","<< start_.pose.position.y << " to " << goal_.pose.position.x <<","<< goal_.pose.position.y);

  // Attempt to generate maneuver paths
  auto srv = std::make_shared<eut_maneuver_generation::srv::GenerateManeuver::Request>();

  // Copy fields from input into service request
  srv->direction        = maneuver_params_.direction;
  srv->entry_dist       = maneuver_params_.entry_dist;
  srv->exit_dist        = maneuver_params_.exit_dist;
  srv->flip_orientation = maneuver_params_.flip_orientation;
  srv->global_frame     = maneuver_params_.global_frame;
  srv->radius           = maneuver_params_.radius;
  srv->type             = maneuver_params_.type;
  srv->waypoint_step    = maneuver_params_.waypoint_step;

  // Initial position from initial coordinates
  srv->pose = start_.pose;

  // Derive perpendicular distance between lines from row coordinates.
  // This assumes lines are parallel. Orientation derived from exiting row.
  const double theta = tf2::getYaw(start_.pose.orientation);

  srv->d_lines = -sin(theta) * (goal_.pose.position.x - start_.pose.position.x) +
                 cos(theta) * (goal_.pose.position.y - start_.pose.position.y);

  auto future_result = cli_->async_send_request(srv);

  if(rclcpp::spin_until_future_complete(node_, future_result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to generate manuever. Aborting action");
    return BT::NodeStatus::FAILURE;
  }

  // https://github.com/ros2/rclcpp/issues/1968
  // In Humble it seems like the data pointed by the returned shared_future becomes invalid as soon
  // as get() is called for the first time, producing segmentation faults in subsequent calls to
  // get(), fixing it in this way:
  auto result = *future_result.get();

  // Sanity check
  if(result.maneuvers.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No maneuvers received. Aborting action");
    return BT::NodeStatus::FAILURE;
  }

  // Create path
  nav_msgs::msg::Path path;
  path.header = result.maneuvers[0].header;

  // Append the robot global pose as first to guarantee a connected path
  if(!buffer_->canTransform(global_frame_, base_frame_, rclcpp::Time(0), rclcpp::Duration(1, 0)))
  {
    RCLCPP_WARN(node_->get_logger(), "[IsInRow] Cannot look up transform!");
    return BT::NodeStatus::FAILURE;
  }
  auto base_tf = buffer_->lookupTransform(global_frame_, base_frame_, rclcpp::Time(0));
  geometry_msgs::msg::PoseStamped base_pos;
  base_pos.header             = base_tf.header;
  base_pos.pose.position.x    = base_tf.transform.translation.x;
  base_pos.pose.position.y    = base_tf.transform.translation.y;
  base_pos.pose.position.z    = base_tf.transform.translation.z;
  base_pos.pose.orientation.x = base_tf.transform.rotation.x;
  base_pos.pose.orientation.y = base_tf.transform.rotation.y;
  base_pos.pose.orientation.z = base_tf.transform.rotation.z;
  base_pos.pose.orientation.w = base_tf.transform.rotation.w;

  path.poses.push_back(base_pos);

  // Then include the poses from the generated maneuver
  path.poses.insert(path.poses.end(),
                    result.maneuvers[0].poses.begin(),
                    result.maneuvers[0].poses.end());                

  // Set output port for sendind the generated trajectories.
  // NOTE: assuming only one path!
  if(!setOutput<nav_msgs::msg::Path>("path", path))
  {
    throw BT::RuntimeError("Missing required output [path]");
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully computed maneuver path.");
  return BT::NodeStatus::SUCCESS;
}


#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ComputeManeuverPath>("ComputeManeuverPath");
}
