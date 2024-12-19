#include <fstream>
#include <functional>
#include <limits>

#include "r4c_can_adapter/carob_ros2can_adapter.hpp"
#include "r4c_can_adapter/dbc_model_checker.hpp"

namespace r4c_can_adapter
{
  CarobRos2CanAdapter::CarobRos2CanAdapter():
    Ros2CanAdapter{},
    wheel_spacing_{this->declare_parameter("wheel_spacing", 0.0)}
  {
    auto num = this->declare_parameter("reduction_ratio_ceol", 1.0) *
               this->declare_parameter("sprocket_radius_carob", 1.0);
    auto den = this->declare_parameter("reduction_ratio_carob", 1.0) *
               this->declare_parameter("sprocket_radius_ceol", 1.0);

    // The factor 0.8 was obtained empirically after trial and error, verifying that
    // the current twist of the platform and the sent twist command differed in a factor.
    conversion_factor_ = 0.8 * (num / den);

    DbcModelChecker dbc_model_checker{carob_dbc_model_, *dbc_};
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void CarobRos2CanAdapter::encode_twist_cmd_can_msg(
    const geometry_msgs::msg::Twist::ConstSharedPtr twist_cmd_msg)
  {
    const auto& can_msg_def = *can_msg_defs_[CAN_MSG_ID::TWIST_CMD];

    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = this->now();
    // can_ids have 11 bits in standar messages and 29 bits in extended messages.
    // 11 bits to 1 is 0x7FF.
    // 29 bits to 1 is 0x1FFFFFFF.
    can_msg.id  = static_cast<decltype(can_msg.id)>(can_msg_def.Id() & 0x1FFFFFFF);
    can_msg.dlc = static_cast<decltype(can_msg.dlc)>(can_msg_def.MessageSize());
    // If id is bigger than 0x7FF, then we can use up to 29 bits for the can_id.
    can_msg.is_extended = can_msg.id > 0x000007FF ? true : false;

    auto sigs = can_msg_def.Signals_Size();

    // v_r = (R + wheel_spacing/2) * w =
    //     = (v/w + wheel_spacing/2) * w =
    //     = (((2 * v) + (wheel_spacing * w)) / (2 * w)) * w
    //     = ((2 * v) + (wheel_spacing * w)) / 2
    //     = v + ((wheel_spacing/2) * w)
    // v_l = (R - wheel_spacing/2) * w =
    //     = (v/w - wheel_spacing/2) * w =
    //     = (((2 * v) - (wheel_spacing * w)) / (2 * w)) * w
    //     = ((2 * v)  (wheel_spacing * w)) / 2
    //     = v - ((wheel_spacing/2) * w)

    // In m/s.
    double v_r = twist_cmd_msg->linear.x + (wheel_spacing_ * twist_cmd_msg->angular.z * 0.5);
    double v_l = twist_cmd_msg->linear.x - (wheel_spacing_ * twist_cmd_msg->angular.z * 0.5);

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "GuidanceLeftSpeedCommand")
      {
        // In mm/s
        sig.Encode(sig.PhysToRaw(conversion_factor_ * 1000.0 * v_l), can_msg.data.data());
      }
      else if(sig_name == "GuidanceRightSpeedCommand")
      {
        // In mm/s
        sig.Encode(sig.PhysToRaw(conversion_factor_ * 1000.0 * v_r), can_msg.data.data());
      }
      else  // SafetyCounterENS
      {
        sig.Encode(sig.PhysToRaw(ens_safety_counter_), can_msg.data.data());
        ++ens_safety_counter_;
      }
    }

    std_msgs::msg::Float64 trajectory_radius;

    // If angular.z is null, then robot moves in straight line, with a infinite radius.
    if(std::abs(twist_cmd_msg->angular.z) > std::numeric_limits<double>::epsilon())
    {
      trajectory_radius.data = (twist_cmd_msg->linear.x) / (twist_cmd_msg->angular.z);
    }
    {  // Vehicle does not move in a 'perfect straight line'.
      trajectory_radius.data = std::copysign(std::numeric_limits<double>::infinity(),
                                             twist_cmd_msg->angular.z);
    }

    trajectory_radius_pub_->publish(trajectory_radius);

    can_msg_pub_->publish(can_msg);
  }

}  // namespace r4c_can_adapter
