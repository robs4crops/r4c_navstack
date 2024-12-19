#include <fstream>
#include <functional>
#include <limits>

#include "r4c_can_adapter/dbc_model_checker.hpp"
#include "r4c_can_adapter/ros2can_adapter.hpp"

namespace r4c_can_adapter
{
  Ros2CanAdapter::Ros2CanAdapter():
    rclcpp::Node{"ros2can_adapter"},
    ens_status_sub_{this->create_subscription<r4c_msgs::msg::AgcboxEnsStatus>(
      this->declare_parameter("ens_status_topic", "agcbox/ens/status"),
      10,
      std::bind(&Ros2CanAdapter::encode_ens_status_can_msg, this, std::placeholders::_1))},
    gbsd_sub_{this->create_subscription<r4c_msgs::msg::Gbsd>(
      this->declare_parameter("gbsd_topic", "gbsd"),
      10,
      std::bind(&Ros2CanAdapter::encode_gbsd_can_msg, this, std::placeholders::_1))},
    gnss_wgs84_fix_sub_{this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("gnss_wgs84_fix_topic", "nobu/gnss/wgs84_fix"),
      10,
      std::bind(&Ros2CanAdapter::encode_gnss_wgs84_fix_can_msg, this, std::placeholders::_1))},
    twist_cmd_sub_{this->create_subscription<geometry_msgs::msg::Twist>(
      this->declare_parameter("twist_cmd_topic", "cmd_vel"),
      10,
      std::bind(&Ros2CanAdapter::encode_twist_cmd_can_msg, this, std::placeholders::_1))},
    can_msg_pub_{this->create_publisher<can_msgs::msg::Frame>(
      this->declare_parameter("ouput_can_msg_topic", "can_msg_to_can_bus"),
      10)},
    trajectory_radius_pub_{this->create_publisher<std_msgs::msg::Float64>("trajectory_radius", 10)}
  {
    const auto dbc_file = declare_parameter("dbc_file", "");

    if(!std::filesystem::exists(dbc_file))
    {
      constexpr std::string_view str_1{"The file '"};
      constexpr std::string_view str_2{"' doest not exist in disk"};
      constexpr auto size = str_1.size() + str_2.size();

      std::string message;
      message.reserve(size + dbc_file.length());
      message.append(str_1).append(dbc_file).append(str_2);

      throw std::invalid_argument{message};
    }

    if(std::filesystem::is_empty(dbc_file))
    {
      constexpr std::string_view str_1{"The file '"};
      constexpr std::string_view str_2{"' is empty"};
      constexpr auto size = str_1.size() + str_2.size();

      std::string message;
      message.reserve(size + dbc_file.length());
      message.append(str_1).append(dbc_file).append(str_2);

      throw std::invalid_argument{message};
    }

    std::ifstream dbc_fs{dbc_file};
    dbc_ = dbcppp::INetwork::LoadDBCFromIs(dbc_fs);

    for(const auto& can_msg_def: dbc_->Messages())
    {
      can_msg_defs_.insert({can_msg_def.Id(), &can_msg_def});
    }

    DbcModelChecker dbc_model_checker{dbc_model_, *dbc_};
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Ros2CanAdapter::encode_ens_status_can_msg(
    const r4c_msgs::msg::AgcboxEnsStatus::ConstSharedPtr ens_status_msg)
  {
    const auto& can_msg_def = *can_msg_defs_[CAN_MSG_ID::ENS_STATUS];

    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = ens_status_msg->header.stamp;
    // can_ids have 11 bits in standar messages and 29 bits in extended messages.
    // 11 bits to 1 is 0x7FF.
    // 29 bits to 1 is 0x1FFFFFFF.
    can_msg.id  = static_cast<decltype(can_msg.id)>(can_msg_def.Id() & 0x1FFFFFFF);
    can_msg.dlc = static_cast<decltype(can_msg.dlc)>(can_msg_def.MessageSize());
    // If id is bigger than 0x7FF, then we can use up to 29 bits for the can_id.
    can_msg.is_extended = can_msg.id > 0x000007FF ? true : false;

    auto sigs = can_msg_def.Signals_Size();

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "RequestMotion")
      {
        sig.Encode(sig.PhysToRaw(ens_status_msg->request_motion), can_msg.data.data());
      }
      else if(sig_name == "StatusWordENS")
      {
        sig.Encode(sig.PhysToRaw(ens_status_msg->status_word), can_msg.data.data());
      }
      else  // MissionWidth
      {
        sig.Encode(sig.PhysToRaw(ens_status_msg->mission_width), can_msg.data.data());
      }
    }

    can_msg_pub_->publish(can_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Ros2CanAdapter::encode_gbsd_can_msg(const r4c_msgs::msg::Gbsd::ConstSharedPtr gbsd_msg)
  {
    const auto& can_msg_def = *can_msg_defs_[CAN_MSG_ID::GBSD];

    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = gbsd_msg->header.stamp;
    // can_ids have 11 bits in standar messages and 29 bits in extended messages.
    // 11 bits to 1 is 0x7FF.
    // 29 bits to 1 is 0x1FFFFFFF.
    can_msg.id  = static_cast<decltype(can_msg.id)>(can_msg_def.Id() & 0x1FFFFFFF);
    can_msg.dlc = static_cast<decltype(can_msg.dlc)>(can_msg_def.MessageSize());
    // If id is bigger than 0x7FF, then we can use up to 29 bits for the can_id.
    can_msg.is_extended = can_msg.id > 0x000007FF ? true : false;

    auto sigs = can_msg_def.Signals_Size();

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "GroundBasedMachineDirection")
      {
        sig.Encode(sig.PhysToRaw(gbsd_msg->direction), can_msg.data.data());
      }
      else if(sig_name == "GroundBasedMachineDistance")
      {
        sig.Encode(sig.PhysToRaw(gbsd_msg->distance), can_msg.data.data());
      }
      else if(sig_name == "GroundBasedMachineSpeed")
      {
        sig.Encode(sig.PhysToRaw(gbsd_msg->speed), can_msg.data.data());
      }
    }

    can_msg_pub_->publish(can_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Ros2CanAdapter::encode_gnss_wgs84_fix_can_msg(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_wgs84_fix_msg)
  {
    const auto& can_msg_def = *can_msg_defs_[CAN_MSG_ID::GNSS_WGS84_FIX];

    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = gnss_wgs84_fix_msg->header.stamp;
    // can_ids have 11 bits in standar messages and 29 bits in extended messages.
    // 11 bits to 1 is 0x7FF.
    // 29 bits to 1 is 0x1FFFFFFF.
    can_msg.id  = static_cast<decltype(can_msg.id)>(can_msg_def.Id() & 0x1FFFFFFF);
    can_msg.dlc = static_cast<decltype(can_msg.dlc)>(can_msg_def.MessageSize());
    // If id is bigger than 0x7FF, then we can use up to 29 bits for the can_id.
    can_msg.is_extended = can_msg.id > 0x000007FF ? true : false;

    auto sigs = can_msg_def.Signals_Size();

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "Latitude")
      {
        // In deg.
        sig.Encode(sig.PhysToRaw(gnss_wgs84_fix_msg->latitude), can_msg.data.data());
      }
      else if(sig_name == "Longitude")
      {
        // In deg.
        sig.Encode(sig.PhysToRaw(gnss_wgs84_fix_msg->longitude), can_msg.data.data());
      }
    }

    can_msg_pub_->publish(can_msg);
  }
}  // namespace r4c_can_adapter
