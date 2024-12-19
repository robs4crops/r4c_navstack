#pragma once

#include <cstdint>
#include <unordered_map>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "dbcppp/Message.h"
#include "dbcppp/Network.h"

#include "r4c_can_adapter/can2ros_adapter.hpp"
namespace r4c_can_adapter
{
  class TractorCan2RosAdapter: public Can2RosAdapter
  {
    public:
    TractorCan2RosAdapter();

    private:
    using tractor_can2ros_decoder =
      void (r4c_can_adapter::TractorCan2RosAdapter::*)(const dbcppp::IMessage& can_msg_def,
                                                       const can_msgs::msg::Frame& can_msg);

    void add_entry_decoding_map(const uint32_t can_id, tractor_can2ros_decoder dec);

    //---------------------------------

    void decode_machine_safety_status(const dbcppp::IMessage& can_msg_def,
                                      const can_msgs::msg::Frame& can_msg);
    //---------------------------------

    void decode_vel_feedback_can_msg(const dbcppp::IMessage& can_msg_def,
                                     const can_msgs::msg::Frame& can_msg);

    //---------------------------------

    void decode_wheels_data_can_msg(const dbcppp::IMessage& can_msg_def,
                                    const can_msgs::msg::Frame& can_msg);

    // --------------------------------

    void init_decoding_map();

    // --------------------------------

    void rcv_can_msg(const can_msgs::msg::Frame::ConstSharedPtr can_msg) override;

    // --------------------------------

    void filter_ublox_fix_msg(const sensor_msgs::msg::NavSatFix::ConstSharedPtr ublox_fix_msg);

    // --------------------------------

    std::unordered_map<uint32_t, std::pair<tractor_can2ros_decoder, const dbcppp::IMessage*>>
      tractor_decoding_map_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ublox_fix_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_wheel_angle_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_wheel_linear_vel_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_wheel_linear_vel_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_echo_pub_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ublox_filtered_fix_pub_;

    double steering_bias_deg_;

    // Ideally, the linear velocity in the speed field of the GBSD CAN message should closely match
    // the velocity shown on the tractor's speedometer.
    // However, these two values do not appear to be close enough for the tractor. Therefore, we can
    // take simultaneous samples of the speedometer's velocity and the velocity in the GBSD command
    // (calculated from the linear velocities of the tractor's left and right rear wheels).
    // Then, we can fit all pairs of values using least squares to create a linear function.
    // From that point onward, for each subsequent calculation, we first obtain the tractor’s linear
    // velocity as before, from the left and right rear wheels, and then apply this value to the
    // linear function to get a final linear velocity for the platform that more closely matches the
    // speedometer's reading.

    double slope_gbsd{0.0};
    double intercept_gbsd{0.0};

    enum TRACTOR_CAN_MSG_ID
    {
      // Messages received from the can bus.
      MACHINE_SAFETY_STATUS = 385,
      VEL_FEEDBACK          = 785,
      WHEELS_DATA           = 787,
    };

    const std::unordered_map<uint64_t, CanMsgModel> tractor_dbc_model_{
      {TRACTOR_CAN_MSG_ID::MACHINE_SAFETY_STATUS,
       {TRACTOR_CAN_MSG_ID::MACHINE_SAFETY_STATUS,
        "MachineSafetyStatus",
        {"SafetyStatusReason",
         "StatusWordCAIUS",
         "SoftStopCommand",
         "QuickStopCommand",
         "PowerCutStatus",
         "ToolActuatorStatus",
         "RobotActuatorStatus",
         "ToolInverterStatus",
         "BrakeStatus",
         "PropulsionInverterStatus"}}},
      {TRACTOR_CAN_MSG_ID::VEL_FEEDBACK,
       {TRACTOR_CAN_MSG_ID::VEL_FEEDBACK,
        "SteeringFeedback_YV",
        {"GuidanceYawRateTarget",
         "GuidanceVelocityTarget",
         "GuidanceYawRateEstimated",
         "GuidanceVelocityEstimated"}}},
      {TRACTOR_CAN_MSG_ID::WHEELS_DATA,
       {TRACTOR_CAN_MSG_ID::WHEELS_DATA,
        "CaesarWheelSensors",
        {"CaesarRightWheelSpeed", "CaesarLeftWheelSpeed", "CaesarLeftWheelAngle"}}}};
  };

}  // namespace r4c_can_adapter
