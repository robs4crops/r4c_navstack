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
#include "r4c_can_adapter/tractor_can2ros_adapter.hpp"

namespace r4c_can_adapter
{
  TractorCan2RosAdapter::TractorCan2RosAdapter():
    Can2RosAdapter{},
    ublox_fix_sub_{this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("ublox_fix_topic", "mbase/fix"),
      10,
      std::bind(&TractorCan2RosAdapter::filter_ublox_fix_msg, this, std::placeholders::_1))},
    manual_mode_pub_{this->create_publisher<std_msgs::msg::Bool>(
      this->declare_parameter("manual_mode_topic", "manual_mode"),
      10)},
    front_left_wheel_angle_pub_{this->create_publisher<std_msgs::msg::Float64>(
      this->declare_parameter("front_left_wheel_angle_topic", "front_left_wheel_angle"),
      10)},
    rear_left_wheel_linear_vel_pub_{this->create_publisher<std_msgs::msg::Float64>(
      this->declare_parameter("rear_left_wheel_linear_vel_topic", "rear_left_wheel_linear_vel"),
      10)},
    rear_right_wheel_linear_vel_pub_{this->create_publisher<std_msgs::msg::Float64>(
      this->declare_parameter("rear_right_wheel_linear_vel_topic", "rear_right_wheel_linear_vel"),
      10)},
    twist_cmd_echo_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
      this->declare_parameter("twist_cmd_echo_topic", "cmd_vel_echo"),
      10)},
    ublox_filtered_fix_pub_{this->create_publisher<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("filtered_ublox_fix_topic", "mbase/fix/filtered"),
      10)},
    steering_bias_deg_{this->declare_parameter("steering_bias_deg", 0.0)},
    slope_gbsd{this->declare_parameter("slope_gbsd", 1.0)},
    intercept_gbsd{this->declare_parameter("intercept_gbsd", 0.0)}
  {
    DbcModelChecker dbc_model_checker{tractor_dbc_model_, *dbc_};

    init_decoding_map();
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void TractorCan2RosAdapter::add_entry_decoding_map(const uint32_t can_id,
                                                     tractor_can2ros_decoder dec)
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
      tractor_decoding_map_.insert({can_id & 0x1FFFFFFF, {dec, &can_msg_def}});
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

  void TractorCan2RosAdapter::decode_machine_safety_status(const dbcppp::IMessage& can_msg_def,
                                                           const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
    RCLCPP_WARN(this->get_logger(), "[%d] dlc: %d.", can_msg->id, can_msg->dlc);

    for(int i{0}; i < 8; i++)
    {
      RCLCPP_WARN(this->get_logger(), "%d - %X.", i, raw_data[i]);
    }
#endif

    std_msgs::msg::Bool manual_mode;

    // By default we consider the vehicle is in manual mode.
    manual_mode.data = true;

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
      RCLCPP_WARN(this->get_logger(), "[%d] %s.", can_msg->id, sig_name.c_str());
#endif

      if(sig_name == "StatusWordCAIUS")
      {
        // According to an e-mail sent by amar.benrais@agreenculture.fr:
        // StatusWordCAIUS
        //  0: Safe
        //  1: Manual-Safe
        //  2: Manual-Enable
        //  3: Manual-Armed
        //  4: Auto-Safe
        //  5: Auto-Enable
        //  6: Motion-Imminent
        //  7: Auto-Motion
        // Typically, by following the procedure in the user manual, you will reach state
        // '6: Motion-Imminent'. At that point, you can start sending commands greater than
        // 0.01 m/s, which will switch the tractor to '7: Auto-Motion' and allow it to move forward.
        // The brakes are released only in state 3 so that the operator can drive the tractor, and
        // in state 7.

        auto status_word = static_cast<int>(sig.RawToPhys(sig.Decode(raw_data)));
        manual_mode.data = status_word < 6 ? true : false;
      }
    }

    manual_mode_pub_->publish(manual_mode);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void TractorCan2RosAdapter::decode_vel_feedback_can_msg(const dbcppp::IMessage& can_msg_def,
                                                          const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
    RCLCPP_WARN(this->get_logger(), "[%d] dlc: %d.", can_msg->id, can_msg->dlc);

    for(int i{0}; i < 8; i++)
    {
      RCLCPP_WARN(this->get_logger(), "%d - %X.", i, raw_data[i]);
    }
#endif

    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Twist twist_cmd_echo;

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG

      RCLCPP_WARN(this->get_logger(), "[%d] %s.", can_msg->id, sig_name.c_str());
#endif

      if(sig_name == "GuidanceVelocityEstimated")
      {
        twist.linear.x = sig.RawToPhys(sig.Decode(raw_data));  // In m/s

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG

        RCLCPP_WARN(this->get_logger(), "[%d] %X %X", can_msg->id, raw_data[0], raw_data[1]);
        RCLCPP_WARN(this->get_logger(), "[%d] e lin vel: %7.3f.", can_msg->id, .twist.linear.x);
#endif
      }
      else if(sig_name == "GuidanceYawRateEstimated")
      {
        twist.angular.z = sig.RawToPhys(sig.Decode(raw_data));  // In rad/s

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
        RCLCPP_WARN(this->get_logger(), "[%d] %X %X", can_msg->id, raw_data[2], raw_data[3]);
        RCLCPP_WARN(this->get_logger(), "[%d] e ang vel: %7.3f.", can_msg->id, twist.angular.z);
#endif
      }
      else if(sig_name == "GuidanceVelocityTarget")
      {
        twist_cmd_echo.linear.x = sig.RawToPhys(sig.Decode(raw_data));  // In m/s

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
        RCLCPP_WARN(this->get_logger(), "[%d] %X %X", can_msg->id, raw_data[4], raw_data[5]);

        RCLCPP_WARN(this->get_logger(),
                    "[%d] c lin vel: %7.3f.",
                    can_msg->id,
                    twist_cmd_echo.linear.x);
#endif
      }
      else  // GuidanceYawRateTarget
      {
        twist_cmd_echo.angular.z = sig.RawToPhys(sig.Decode(raw_data));  // In rad/s

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
        RCLCPP_WARN(this->get_logger(), "[%d] %X %X", can_msg->id, raw_data[6], raw_data[7]);
        RCLCPP_WARN(this->get_logger(),
                    "[%d] c ang vel: %7.3f.",
                    can_msg->id,
                    twist_cmd_echo.angular.z);
#endif
      }
    }

    twist_pub_->publish(twist);
    twist_cmd_echo_pub_->publish(twist_cmd_echo);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void TractorCan2RosAdapter::decode_wheels_data_can_msg(const dbcppp::IMessage& can_msg_def,
                                                         const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    std_msgs::msg::Float64 rear_left_wheel_linear_vel;
    std_msgs::msg::Float64 rear_right_wheel_linear_vel;
    r4c_msgs::msg::Gbsd gbsd;

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      // AGC is sending the following magnitudes in mm/s and in deg, instead of m/s and rad.
      // I asked them to change to m/s and rad/s, to respect the international system unit, so we
      // work in a common frame. However, they were not willing to chage the units in the dbc file,
      // since they are using the following fields for their computations in the low-level
      // controllers they have developed for the linear and angular velocity, so they had to change
      // the units of these fields in several places, so they prefer not to do that. Therefore, we
      // transform in this adapter from mm/s to m/s, and from deg to rad.

      if(sig_name == "CaesarLeftWheelSpeed")
      {
        // From mm/s to m/s.
        rear_left_wheel_linear_vel.data = sig.RawToPhys(sig.Decode(raw_data)) * 0.001;

        rear_left_wheel_linear_vel_pub_->publish(rear_left_wheel_linear_vel);
      }
      else if(sig_name == "CaesarRightWheelSpeed")
      {
        // From mm/s to m/s.
        rear_right_wheel_linear_vel.data = sig.RawToPhys(sig.Decode(raw_data)) * 0.001;

        rear_right_wheel_linear_vel_pub_->publish(rear_right_wheel_linear_vel);
      }
      else  // CaesarLeftWheelAngle
      {
        // From deg to rad
        std_msgs::msg::Float64 front_left_wheel_angle;
        front_left_wheel_angle.data = angles::from_degrees(sig.RawToPhys(sig.Decode(raw_data)));

        front_left_wheel_angle.data = front_left_wheel_angle.data -
                                      angles::from_degrees(steering_bias_deg_);

        front_left_wheel_angle_pub_->publish(front_left_wheel_angle);
      }
    }

    gbsd.header.frame_id = "NA";
    gbsd.header.stamp    = can_msg.header.stamp;

    gbsd.distance  = -1.0;  // The way to indicate the field distance is not valid to be used.
    gbsd.speed     = 0.0;   // By default, the tractor is stopped
    gbsd.direction = r4c_msgs::msg::Gbsd::NOT_AVAILABLE;  // tractor stopped means dir not available

    // In a normal operation, the velocity of each rear wheel should be provided, and since the
    // tractor has an Ackermann configuration, it should be expected that both rear drive wheels
    // move at a similar speed, ideally equal.
    // Let's consider the tractor is stopped when the velocity is less than 0.5 km/h, for example.

    double linear_vel{0.0};

    constexpr double threshold{0.5 / 3.6};  // 0.5 km/h is (0.5/3.6) m/s

    if(std::abs(rear_left_wheel_linear_vel.data) > threshold &&
       std::abs(rear_right_wheel_linear_vel.data) > threshold)
    {
      // Strictly speaking the linear velocity of the tractor is computed with:
      // v = (v_right + v_left)/2
      linear_vel = (rear_left_wheel_linear_vel.data + rear_right_wheel_linear_vel.data) * 0.5;
    }
    // However, in some occasions, the AGCBox does not provide either the velocity of the rear
    // left or rear right wheel. In any case, since the tractor has an Ackermann configuration,
    // given just the velocity of one of the rear wheels, either left or right, we can consider
    // the velocity which is not reported is equal to the one reported.
    else if(std::abs(rear_left_wheel_linear_vel.data) > threshold)
    {
      // v_right not reported, v_right is considered equal to v_left.
      // v = (v_right + v_left)/2 = (v_left + v_left) / 2 = v_left
      linear_vel = rear_left_wheel_linear_vel.data;
    }
    else if(std::abs(rear_right_wheel_linear_vel.data) > threshold)
    {
      // v_left not reported, v_left is considered equal to v_right.
      // v = (v_right + v_left)/2 = (v_right + v_right) / 2 = v_right
      linear_vel = rear_right_wheel_linear_vel.data;
    }

    if(std::abs(linear_vel) > threshold)  // linear vel > threshold means the tractor isn't stopped
    {
      // Use the least-squares line.
      gbsd.speed     = (slope_gbsd * linear_vel) + intercept_gbsd;
      gbsd.direction = gbsd.speed > 0.0 ? r4c_msgs ::msg::Gbsd::FORWARD :
                                          gbsd.direction = r4c_msgs::msg::Gbsd::REVERSE;
    }

    gbsd_pub_->publish(gbsd);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void TractorCan2RosAdapter::init_decoding_map()
  {
    add_entry_decoding_map(TRACTOR_CAN_MSG_ID::VEL_FEEDBACK,
                           &TractorCan2RosAdapter::decode_vel_feedback_can_msg);

    add_entry_decoding_map(TRACTOR_CAN_MSG_ID::WHEELS_DATA,
                           &TractorCan2RosAdapter::decode_wheels_data_can_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void TractorCan2RosAdapter::rcv_can_msg(const can_msgs::msg::Frame::ConstSharedPtr can_msg)
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

    auto tractor_decoding_map_entry = tractor_decoding_map_.find(can_id);

    if(tractor_decoding_map_entry != tractor_decoding_map_.end())
    {
      auto decoder            = tractor_decoding_map_entry->second.first;
      const auto& can_msg_def = *tractor_decoding_map_entry->second.second;
      // Call the function with: proper can_msg definition and the can message.
      (*this.*decoder)(can_msg_def, *can_msg);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void TractorCan2RosAdapter::filter_ublox_fix_msg(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr ublox_fix_msg)
  {
    if(ublox_fix_msg->status.status == 2)  // Only publish message when status is STATUS_GBAS_FIX
    {
      sensor_msgs::msg::NavSatFix ublox_filtered_fix_msg;
      ublox_filtered_fix_msg = *ublox_fix_msg;
      ublox_filtered_fix_pub_->publish(ublox_filtered_fix_msg);
    }
  }

}  // namespace r4c_can_adapter
