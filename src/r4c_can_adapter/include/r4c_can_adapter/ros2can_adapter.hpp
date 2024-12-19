#pragma once

#include <memory>  // smartpointers
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

#include <r4c_msgs/msg/agcbox_ens_status.hpp>
#include <r4c_msgs/msg/gbsd.hpp>

#include "dbcppp/Message.h"
#include "dbcppp/Network.h"

#include "r4c_can_adapter/can_msg_model.hpp"
namespace r4c_can_adapter
{
  class Ros2CanAdapter: public rclcpp::Node
  {
    public:
    Ros2CanAdapter();

    protected:
    void encode_ens_status_can_msg(
      const r4c_msgs::msg::AgcboxEnsStatus::ConstSharedPtr ens_status_msg);

    void encode_gbsd_can_msg(const r4c_msgs::msg::Gbsd::ConstSharedPtr gbsd_msg);

    void encode_gnss_wgs84_fix_can_msg(
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_wgs84_fix_msg);

    virtual void encode_twist_cmd_can_msg(
      const geometry_msgs::msg::Twist::ConstSharedPtr twist_msg) = 0;

    std::unique_ptr<dbcppp::INetwork> dbc_;
    std::unordered_map<uint64_t, const dbcppp::IMessage*> can_msg_defs_;

    rclcpp::Subscription<r4c_msgs::msg::AgcboxEnsStatus>::SharedPtr ens_status_sub_;
    rclcpp::Subscription<r4c_msgs::msg::Gbsd>::SharedPtr gbsd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_wgs84_fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_msg_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trajectory_radius_pub_;

    enum CAN_MSG_ID
    {
      ENS_STATUS     = 401,
      GNSS_WGS84_FIX = 2314731804,  // 0x89F8011C
      TWIST_CMD      = 657,
      GBSD           = 2365475326  // 0x8CFE49FE
    };

    const std::unordered_map<uint64_t, CanMsgModel> dbc_model_{
      {CAN_MSG_ID::ENS_STATUS,
       {CAN_MSG_ID::ENS_STATUS, "ENSStatus", {"MissionWidth", "StatusWordENS", "RequestMotion"}}},
      {CAN_MSG_ID::GNSS_WGS84_FIX,
       {CAN_MSG_ID::GNSS_WGS84_FIX, "NobuGnssWgs84", {"Longitude", "Latitude"}}},
      {CAN_MSG_ID::GBSD,
       {CAN_MSG_ID::GBSD,
        "Gbsd",
        {"GroundBasedMachineDirection", "GroundBasedMachineDistance", "GroundBasedMachineSpeed"}}}};
  };
}  // namespace r4c_can_adapter
