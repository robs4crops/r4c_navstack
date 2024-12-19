#ifndef R4C_NAVIGATION_ACTIONS_CHANGE_WAYPOINTS_H
#define R4C_NAVIGATION_ACTIONS_CHANGE_WAYPOINTS_H

#include "behaviortree_cpp_v3/bt_factory.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eut_fixed_gplanner/srv/change_waypoints.hpp>

// Action that updates the waypoints used with the eut_fixed_gplanner,
// by calling the "ChangeWaypoints" service.

class ChangeWaypoints: public BT::SyncActionNode
{
  public:
  ChangeWaypoints(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("service_name"),
            BT::InputPort<std::string>("waypoints_file")};
  };

  private:
  std::string service_name_{"/change_waypoints_file"};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<eut_fixed_gplanner::srv::ChangeWaypoints>::SharedPtr cli_; 
};

#endif  // R4C_NAVIGATION_ACTIONS_CHANGE_WAYPOINTS_H
