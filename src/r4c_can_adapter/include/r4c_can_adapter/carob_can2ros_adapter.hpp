#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <r4c_msgs/msg/agcbox_ens_control.hpp>
#include <r4c_msgs/msg/agcbox_gnss_metadata.hpp>
#include <r4c_msgs/msg/carob_inverter.hpp>
#include <r4c_msgs/msg/gnss_enu_vector.hpp>

#include "r4c_can_adapter/can2ros_adapter.hpp"
namespace r4c_can_adapter
{
  class CarobCan2RosAdapter: public Can2RosAdapter
  {
    public:
    CarobCan2RosAdapter();

    private:
    using carob_can2ros_decoder =
      void (r4c_can_adapter::CarobCan2RosAdapter::*)(const dbcppp::IMessage& can_msg_def,
                                                     const can_msgs::msg::Frame& can_msg);

    void add_entry_decoding_map(const uint32_t can_id, carob_can2ros_decoder dec);

    // --------------------------------

    void decode_inverter_can_msg(const dbcppp::IMessage& can_msg_def,
                                 const can_msgs::msg::Frame& can_msg) noexcept;

    // --------------------------------

    void init_decoding_map();

    // --------------------------------

    void rcv_can_msg(const can_msgs::msg::Frame::ConstSharedPtr can_msg) override;

    // --------------------------------

    double inverter_max_torque_;
    double linear_vel_from_rpm_;  // coefficient to get linear vel in m/s from rpm
    double wheel_radius_;
    double wheel_spacing_;

    std::unordered_map<uint32_t, std::pair<carob_can2ros_decoder, const dbcppp::IMessage*>>
      carob_decoding_map_;

    rclcpp::Publisher<r4c_msgs::msg::CarobInverter>::SharedPtr left_track_inverter_pub_;

    rclcpp::Publisher<r4c_msgs::msg::CarobInverter>::SharedPtr right_track_inverter_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_track_linear_vel_pub_;
    std_msgs::msg::Float64 left_track_linear_vel_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_track_linear_vel_pub_;
    std_msgs::msg::Float64 right_track_linear_vel_;

    geometry_msgs::msg::Twist twist_;

    enum CAROB_CAN_MSG_ID
    {
      LEFT_INVERTER_STATUS  = 390,
      RIGHT_INVERTER_STATUS = 391,
    };

    const std::unordered_map<uint64_t, CanMsgModel> carob_dbc_model_{
      {CAROB_CAN_MSG_ID::LEFT_INVERTER_STATUS,
       {CAROB_CAN_MSG_ID::LEFT_INVERTER_STATUS,
        "InverterLeftDrivingStatus",
        {"InverterLeftTorque", "InverterLeftVelocityRPM", "InverterLeftStatusWord"}}},
      {CAROB_CAN_MSG_ID::RIGHT_INVERTER_STATUS,
       {CAROB_CAN_MSG_ID::RIGHT_INVERTER_STATUS,
        "InverterRightDrivingStatus",
        {"InverterRightTorque", "InverterRightVelocityRPM", "InverterRightStatusWord"}}}};
  };
}  // namespace r4c_can_adapter
