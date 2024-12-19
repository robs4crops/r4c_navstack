#include <r4c_navigation/action/define_parcel.hpp>

DefineParcel::DefineParcel(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("define_parcel");
}

BT::NodeStatus DefineParcel::tick()
{
  geometry_msgs::msg::PoseStamped enter_pose;
  if(!getInput("enter_pose", enter_pose))
  {
    throw BT::RuntimeError("Missing required input [enter_pose]");
  }

  geometry_msgs::msg::PoseStamped exit_pose;
  if(!getInput("exit_pose", exit_pose))
  {
    throw BT::RuntimeError("Missing required input [exit_pose]");
  }

  double enter_dist;
  if(!getInput("enter_dist", enter_dist))
  {
    throw BT::RuntimeError("Missing required input [enter_dist]");
  }

  double exit_dist;
  if(!getInput("exit_dist", exit_dist))
  {
    throw BT::RuntimeError("Missing required input [exit_dist]");
  }

  double parcel_width;
  if(!getInput("row_width", parcel_width))
  {
    throw BT::RuntimeError("Missing required input [row_width]");
  }

  double enter_exit_vector_x = exit_pose.pose.position.x - enter_pose.pose.position.x;
  double enter_exit_vector_y = exit_pose.pose.position.y - enter_pose.pose.position.y; 

  double magn = sqrt(pow(enter_exit_vector_x, 2) + pow(enter_exit_vector_y, 2));

  double norm_vector_x = enter_exit_vector_x/magn;
  double norm_vector_y = enter_exit_vector_y/magn;

  geometry_msgs::msg::PoseStamped parcel_origin;
  parcel_origin = enter_pose;
  parcel_origin.pose.position.x = enter_pose.pose.position.x + norm_vector_x*enter_dist;
  parcel_origin.pose.position.y = enter_pose.pose.position.y + norm_vector_y*enter_dist;

  geometry_msgs::msg::PoseStamped parcel_end;
  parcel_end = exit_pose;
  parcel_end.pose.position.x = exit_pose.pose.position.x - norm_vector_x*exit_dist;
  parcel_end.pose.position.y = exit_pose.pose.position.y - norm_vector_y*exit_dist;

  if(!setOutput<geometry_msgs::msg::PoseStamped>("parcel_origin", parcel_origin))
  {
    throw BT::RuntimeError("Could not set output [parcel_origin]");
  }

  if(!setOutput<geometry_msgs::msg::PoseStamped>("parcel_end", parcel_end))
  {
    throw BT::RuntimeError("Could not set output [parcel_end]");
  }
  
  if(!setOutput<double>("parcel_width", parcel_width))
  {
    throw BT::RuntimeError("Could not set output [parcel_width]");
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<DefineParcel>("DefineParcel");
}