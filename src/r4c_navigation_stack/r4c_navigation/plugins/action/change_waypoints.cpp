#include <r4c_navigation/action/change_waypoints.hpp>

ChangeWaypoints::ChangeWaypoints(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  if(!getInput<std::string>("service_name", service_name_))
  {
    throw BT::RuntimeError("Missing required input [service_name]");
  }

  node_ = std::make_shared<rclcpp::Node>("change_waypoints");
  cli_  = node_->create_client<eut_fixed_gplanner::srv::ChangeWaypoints>(service_name_);
}

BT::NodeStatus ChangeWaypoints::tick()
{
  std::string filename;
  if(!getInput<std::string>("waypoints_file", filename))
  {
    throw BT::RuntimeError("Missing required input [waypoints_file]");
  }

  // Call service
  auto req = std::make_shared<eut_fixed_gplanner::srv::ChangeWaypoints::Request>();

  // TODO@juan.rascon: Change the almost-hardcoded URL with an input port.

  req->waypoints_url =
    ament_index_cpp::get_package_share_directory("r4c_navigation") + "/waypoints/" + filename + ".csv";

  auto result = cli_->async_send_request(req);

  if(rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service ChangeWaypoints");

    return BT::NodeStatus::FAILURE;
  }

  if(!result.get()->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to change waypoints");

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ChangeWaypoints>("ChangeWaypoints");
}