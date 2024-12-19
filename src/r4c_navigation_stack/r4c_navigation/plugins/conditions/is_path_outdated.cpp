#include "r4c_navigation/conditions/is_path_outdated.hpp"

IsPathOutdated::IsPathOutdated(const std::string& name, const BT::NodeConfiguration& config):
  BT::ConditionNode(name, config),
  node_{std::make_shared<rclcpp::Node>("is_path_outdated_node")},
  path_timeout_ns_{static_cast<rcl_duration_value_t>(10.0 * 1E9)}
{
  auto path_timeout{0.0};

  if(getInput("path_timeout", path_timeout))
  {
    // provide a deault value in case the value from the input port cannot be retrieved when
    // using this function in constructors.
    path_timeout_ns_ = static_cast<rcl_duration_value_t>(path_timeout * 1E9);
  }
}

//***************************************************************************
//***************************************************************************

BT::NodeStatus IsPathOutdated::tick()
{
  if(!getInput("path", path_))
  {
    std::string msg{"The 'path' input port is required."};

    RCLCPP_ERROR_STREAM(node_->get_logger(), "[IsPathOutdated_tick] " << msg << '.');

    return BT::NodeStatus::FAILURE;
  }

  auto time_difference = node_->now() - path_.header.stamp;

  if(time_difference.nanoseconds() > path_timeout_ns_)
  {
    RCLCPP_WARN(node_->get_logger(),
                " [IsPathOutdated_tick] Provided path exceeds configured timeout, failing...");

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<IsPathOutdated>("IsPathOutdated");
}
