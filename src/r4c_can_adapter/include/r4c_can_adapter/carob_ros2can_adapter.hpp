#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "dbcppp/Message.h"
#include "dbcppp/Network.h"

#include "r4c_can_adapter/ros2can_adapter.hpp"

namespace r4c_can_adapter
{
  /////////////////////////////////////
  // Ros2CanAdapter
  /////////////////////////////////////

  class CarobRos2CanAdapter: public Ros2CanAdapter
  {
    public:
    CarobRos2CanAdapter();

    private:
    void encode_twist_cmd_can_msg(
      const geometry_msgs::msg::Twist::ConstSharedPtr twist_cmd_msg) override;

    double conversion_factor_;
    double wheel_spacing_;
    uint8_t ens_safety_counter_{0};

    const std::unordered_map<uint64_t, CanMsgModel> carob_dbc_model_{
      {CAN_MSG_ID::TWIST_CMD,
       {CAN_MSG_ID::TWIST_CMD,
        "SteeringCommand_RL",
        {"SafetyCounterENS", "GuidanceRightSpeedCommand", "GuidanceLeftSpeedCommand"}}}};
  };

}  // namespace r4c_can_adapter
