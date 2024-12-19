#include <r4c_navigation/action/call_empty_srv.hpp>

CallEmptySrv::CallEmptySrv(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config),
  node_{std::make_shared<rclcpp::Node>("call_empty_srv")}
{
  if(!getInput<std::string>("service_name", service_name_))
  {
    throw BT::RuntimeError("Missing required input [service_name]");
  }

  cli_ = node_->create_client<std_srvs::srv::Empty>(service_name_);
}

BT::NodeStatus CallEmptySrv::tick()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result  = cli_->async_send_request(request);

  if(rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service '%s'", service_name_.c_str());

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CallEmptySrv>("CallEmptySrv");
}
