#include <chrono>
#include <stdexcept>

#include "r4c_navigation/action/get_path_service.hpp"

using namespace std::chrono_literals;

GetPathService::GetPathService(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config),
  node_{std::make_shared<rclcpp::Node>("get_path_client")}
{
  std::string service{"/eut_crop_row_estimator/get_path"};  // default value

  getInput<std::string>("service", service);

  cli_ = node_->create_client<eut_crop_row_estimator::srv::GetPath>(service);
  
}

//***************************************************************************
//***************************************************************************

BT::NodeStatus GetPathService::tick()
{
  double timeout{0.0};

  if(!getInput<double>("timeout", timeout))
  {
    std::string msg{"The 'timeout' input port is required."};

    RCLCPP_ERROR_STREAM(node_->get_logger(), "[GetPathService_tick] " << msg << '.');

    return BT::NodeStatus::FAILURE;
  }

  if(timeout < 0.0)
  {
    std::string msg{"The 'timeout' input port must have a positive value."};

    RCLCPP_ERROR_STREAM(node_->get_logger(), "[GetPathService_tick] " << msg << '.');

    return BT::NodeStatus::FAILURE;
  }

  auto request     = std::make_shared<eut_crop_row_estimator::srv::GetPath::Request>();
  request->request = true;

  auto result = cli_->async_send_request(request);

  if(rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service GetPath");

    return BT::NodeStatus::FAILURE;
  }

  path_ = result.get()->path;
  // bool outdated = result.get()->outdated;

  auto now_stamp  = node_->now();
  auto path_stamp = rclcpp::Time{path_.header.stamp};

  auto elapsed_time = now_stamp - path_stamp;

  RCLCPP_DEBUG(node_->get_logger(),
               "[GetPathService] Current stamp: %f",
               now_stamp.seconds());
  RCLCPP_DEBUG(node_->get_logger(), "[GetPathService] Path stamp: %f", path_stamp.seconds());

  RCLCPP_DEBUG(node_->get_logger(),
               "[GetPathService] Elapsed stamp: %f",
               elapsed_time.seconds());

  if((elapsed_time.seconds() > timeout))
  {
    RCLCPP_ERROR(node_->get_logger(), "Provided path exceeded timeout!");

    return BT::NodeStatus::FAILURE;
  }

  setOutput<nav_msgs::msg::Path>("path", path_);

  // @todo: check if this can be set in the xml
  // path_updated="{path_updated}"
  // config().blackboard->set("path_updated", true);

  setOutput<bool>("path_updated", true);

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetPathService>("GetPathService");
}
