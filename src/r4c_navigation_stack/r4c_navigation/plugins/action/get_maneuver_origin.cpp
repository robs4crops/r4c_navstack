#include <r4c_navigation/action/get_maneuver_origin.hpp>

GetManeuverOrigin::GetManeuverOrigin(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("get_maneuver_origin");
}

BT::NodeStatus GetManeuverOrigin::tick()
{
  geometry_msgs::msg::PoseStamped current_parcel_origin;
  if(!getInput("current_parcel_origin", current_parcel_origin))
  {
    throw BT::RuntimeError("Missing required input [current_parcel_origin]");
  }

  geometry_msgs::msg::PoseStamped current_parcel_end;
  if(!getInput("current_parcel_end", current_parcel_end))
  {
    throw BT::RuntimeError("Missing required input [current_parcel_end]");
  }

  geometry_msgs::msg::PoseStamped parcel_origin;
  if(!getInput("parcel_origin", parcel_origin))
  {
    throw BT::RuntimeError("Missing required input [parcel_origin]");
  }

  geometry_msgs::msg::PoseStamped parcel_end;
  if(!getInput("parcel_end", parcel_end))
  {
    throw BT::RuntimeError("Missing required input [parcel_end]");
  }

  double man_exit_dist;
  if(!getInput("man_exit_dist", man_exit_dist))
  {
    throw BT::RuntimeError("Missing required input [man_exit_dist]");
  }
  
  // Compute maneuver origin pose
  double vector_x = current_parcel_end.pose.position.x - current_parcel_origin.pose.position.x;
  double vector_y = current_parcel_end.pose.position.y - current_parcel_origin.pose.position.y; 

  double magn = sqrt(pow(vector_x, 2) + pow(vector_y, 2));

  double norm_vector_x = vector_x/magn;
  double norm_vector_y = vector_y/magn;

  geometry_msgs::msg::PoseStamped man_origin_pose;
  man_origin_pose = current_parcel_end;
  man_origin_pose.pose.position.x = current_parcel_end.pose.position.x + norm_vector_x*man_exit_dist;
  man_origin_pose.pose.position.y = current_parcel_end.pose.position.y + norm_vector_y*man_exit_dist;

  RCLCPP_WARN_STREAM(node_->get_logger(),"Maneuver origin "<<man_origin_pose.pose.position.x<<","<<man_origin_pose.pose.position.y);

  if(!setOutput<geometry_msgs::msg::PoseStamped>("man_origin_pose", man_origin_pose))
  {
    throw BT::RuntimeError("Could not set output [man_origin_pose]");
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetManeuverOrigin>("GetManeuverOrigin");
}