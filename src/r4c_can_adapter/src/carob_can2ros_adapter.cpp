#include <cmath>
#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <angles/angles.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <r4c_msgs/msg/gbsd.hpp>

#include "r4c_can_adapter/dbc_model_checker.hpp"
#include "r4c_can_adapter/carob_can2ros_adapter.hpp"

namespace r4c_can_adapter
{
  CarobCan2RosAdapter::CarobCan2RosAdapter():
    Can2RosAdapter{},
    inverter_max_torque_{this->declare_parameter("inverter_max_torque", 0.0)},
    wheel_radius_{this->declare_parameter("wheel_radius", 0.0)},
    wheel_spacing_{this->declare_parameter("wheel_spacing", 0.0)},
    left_track_inverter_pub_{this->create_publisher<r4c_msgs::msg::CarobInverter>(
      this->declare_parameter("left_track_inverter_topic", "left_track/inverter"),
      10)},
    right_track_inverter_pub_{this->create_publisher<r4c_msgs::msg::CarobInverter>(
      this->declare_parameter("right_track_inverter_topic", "right_track/inverter"),
      10)},
    left_track_linear_vel_pub_{this->create_publisher<std_msgs::msg::Float64>(
      this->declare_parameter("left_track_linear_vel_topic", "left_track/linear_vel"),
      10)},
    right_track_linear_vel_pub_{this->create_publisher<std_msgs::msg::Float64>(
      this->declare_parameter("right_track_linear_vel_topic", "right_track/linear_vel"),
      10)}
  {
    constexpr auto rad_per_sec_from_rpm = (2.0 * M_PI) / 60.0;

    linear_vel_from_rpm_ = wheel_radius_ * rad_per_sec_from_rpm *
                           this->declare_parameter("sprocket_radius_carob", 0.0) *
                           (1.0 / this->declare_parameter("reduction_ratio_carob", 1.0));

    DbcModelChecker dbc_model_checker{carob_dbc_model_, *dbc_};

    init_decoding_map();

    left_track_linear_vel_.data  = std::numeric_limits<double>::quiet_NaN();
    right_track_linear_vel_.data = std::numeric_limits<double>::quiet_NaN();
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void CarobCan2RosAdapter::add_entry_decoding_map(const uint32_t can_id, carob_can2ros_decoder dec)
  {
    auto predicate = [&can_id](const dbcppp::IMessage& imessage) {
      return imessage.Id() == can_id;
    };

    auto dbc_iterable = dbc_->Messages();

    auto can_msg_def_it = std::find_if(dbc_iterable.begin(), dbc_iterable.end(), predicate);

    if(can_msg_def_it != dbc_iterable.end())
    {
      const auto& can_msg_def = *can_msg_def_it;
      // can_ids have 11 bits in standar messages and 29 bits in extended messages.
      // Truncate the can_ids to 29 bits at most. Mask for 29 bits: 0x1FFFFFFF.
      carob_decoding_map_.insert({can_id & 0x1FFFFFFF, {dec, &can_msg_def}});
    }
    else
    {
      std::string can_id_str = std::to_string(can_id);
      constexpr std::string_view str_1{"Can message '"};
      constexpr std::string_view str_2{"' not present in dbc file"};
      constexpr auto size = str_1.size() + str_2.size();

      std::string msg;
      msg.reserve(size + can_id_str.size());
      msg.append(str_1).append(can_id_str).append(str_2);

      throw std::runtime_error{msg};
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void CarobCan2RosAdapter::decode_inverter_can_msg(const dbcppp::IMessage& can_msg_def,
                                                    const can_msgs::msg::Frame& can_msg) noexcept
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    r4c_msgs::msg::CarobInverter track_inverter;

    // Inverter Le f  t  S tatusWord
    //                   V elocityRPM
    //                   T orque
    // 01234567 89 10 11 12
    // Inverter Ri g  h  t  S tatusWord
    //                      V elocityRPM
    //                      T orque
    // 01234567 89 10 11 12 13
    size_t index{12ul};  // default value to use with left inverter
    double sign{-1.0};   // default value to use with left inverter

    std_msgs::msg::Float64* track_linear_vel_ = &left_track_linear_vel_;

    rclcpp::Publisher<r4c_msgs::msg::CarobInverter>::SharedPtr
      track_inverter_pub = left_track_inverter_pub_;  // default value to use with left inverter

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      track_linear_vel_pub = left_track_linear_vel_pub_;  // default value to use with left inverter

    if(can_msg_def.Signals_Get(0).Name()[8] == 'R')
    {
      index                = 13ul;
      sign                 = 1.0;
      track_linear_vel_    = &right_track_linear_vel_;
      track_inverter_pub   = right_track_inverter_pub_;
      track_linear_vel_pub = right_track_linear_vel_pub_;
    }

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name[index] == 'S')
      {
        track_inverter.status = static_cast<uint16_t>(sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name[index] == 'V')
      {
        const auto rpm          = sig.RawToPhys(sig.Decode(raw_data));  // In rpm
        track_inverter.rpm      = static_cast<int32_t>(rpm);
        track_inverter.vel      = sign * linear_vel_from_rpm_ * rpm;
        track_linear_vel_->data = track_inverter.vel;
      }
      else  // T
      {
        track_inverter.torque_percentage = sig.RawToPhys(sig.Decode(raw_data));  // In %
        track_inverter.torque = (track_inverter.torque_percentage * 0.01) * inverter_max_torque_;
      }
    }

    twist_.linear.x  = (right_track_linear_vel_.data + left_track_linear_vel_.data) * 0.5;
    twist_.angular.z = (right_track_linear_vel_.data - left_track_linear_vel_.data) /
                       wheel_spacing_;

    track_inverter_pub->publish(track_inverter);
    track_linear_vel_pub->publish(*track_linear_vel_);
    twist_pub_->publish(twist_);

    r4c_msgs::msg::Gbsd gbsd;
    gbsd.distance = -1.0;  // The way to indicate the field distance is not valid to be used.

    gbsd.header.frame_id = "NA";
    gbsd.header.stamp    = can_msg.header.stamp;
    gbsd.speed           = twist_.linear.x;

    if(std::abs(gbsd.speed) < std::numeric_limits<double>::epsilon())  // speed = 0 m/s
    {
      gbsd.direction = r4c_msgs::msg::Gbsd::NOT_AVAILABLE;
    }
    else if(gbsd.speed > 0)  // speed is not 0 m/s and it is positive
    {
      gbsd.direction = r4c_msgs::msg::Gbsd::FORWARD;
    }
    else  // speed is not 0 m/s and it is negative
    {
      gbsd.direction = r4c_msgs::msg::Gbsd::REVERSE;
    }

    gbsd_pub_->publish(gbsd);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void CarobCan2RosAdapter::init_decoding_map()
  {
    add_entry_decoding_map(CAROB_CAN_MSG_ID::LEFT_INVERTER_STATUS,
                           &CarobCan2RosAdapter::decode_inverter_can_msg);

    add_entry_decoding_map(CAROB_CAN_MSG_ID::RIGHT_INVERTER_STATUS,
                           &CarobCan2RosAdapter::decode_inverter_can_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void CarobCan2RosAdapter::rcv_can_msg(const can_msgs::msg::Frame::ConstSharedPtr can_msg)
  {
    // Find the proper decoding function, associated with the proper can msg defition.
    // can_ids have 11 bits in standar messages and 29 bits in extended messages.
    // Truncate the can_ids to 29 bits at most. Mask for 29 bits: 0x1FFFFFFF.

    auto can_id = can_msg->id & 0x1FFFFFFF;

    auto decoding_map_entry = decoding_map_.find(can_id);

    if(decoding_map_entry != decoding_map_.end())
    {
      // decoder is a pointer to a member function that can process the incoming can_msg.
      auto decoder            = decoding_map_entry->second.first;
      const auto& can_msg_def = *decoding_map_entry->second.second;
      // Call the function with: proper can_msg definition and the can message.
      (*this.*decoder)(can_msg_def, *can_msg);

      return;
    }

    auto carob_decoding_map_entry = carob_decoding_map_.find(can_id);

    if(carob_decoding_map_entry != carob_decoding_map_.end())
    {
      auto decoder            = carob_decoding_map_entry->second.first;
      const auto& can_msg_def = *carob_decoding_map_entry->second.second;
      // Call the function with: proper can_msg definition and the can message.
      (*this.*decoder)(can_msg_def, *can_msg);
    }
  }
}  // namespace r4c_can_adapter
