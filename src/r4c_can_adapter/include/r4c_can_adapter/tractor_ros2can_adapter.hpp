#pragma once

#include <memory>  // smartpointers
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

#include "dbcppp/Message.h"
#include "dbcppp/Network.h"

#include "r4c_can_adapter/ros2can_adapter.hpp"
namespace r4c_can_adapter
{
  class TractorRos2CanAdapter: public Ros2CanAdapter
  {
    public:
    TractorRos2CanAdapter();

    private:
    void encode_twist_cmd_can_msg(
      const geometry_msgs::msg::Twist::ConstSharedPtr twist_cmd_msg) override;

    const std::unordered_map<uint64_t, CanMsgModel> tractor_dbc_model_{
      {CAN_MSG_ID::TWIST_CMD,
       {CAN_MSG_ID::TWIST_CMD,
        "SteeringCommands_YV",
        {"GuidanceYawRateCommand", "GuidanceVelocityCommand"}}}};
  };
}  // namespace r4c_can_adapter
