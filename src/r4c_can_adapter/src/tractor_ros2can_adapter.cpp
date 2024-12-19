#include <fstream>
#include <functional>
#include <limits>

#include "r4c_can_adapter/dbc_model_checker.hpp"
#include "r4c_can_adapter/tractor_ros2can_adapter.hpp"

namespace r4c_can_adapter
{
  TractorRos2CanAdapter::TractorRos2CanAdapter(): Ros2CanAdapter{}
  {
    DbcModelChecker dbc_model_checker{tractor_dbc_model_, *dbc_};
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void TractorRos2CanAdapter::encode_twist_cmd_can_msg(
    const geometry_msgs::msg::Twist::ConstSharedPtr twist_cmd_msg)
  {
    const auto& can_msg_def = *can_msg_defs_[CAN_MSG_ID::TWIST_CMD];

    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = this->now();
    // can_ids have 11 bits in standar messages and 29 bits in extended messages.
    // 11 bits to 1 is 0x7FF.
    // 29 bits to 1 is 0x1FFFFFFF.
    can_msg.id  = static_cast<decltype(can_msg.id)>(can_msg_def.Id() & 0x1FFFFFFF);
    can_msg.dlc          = static_cast<decltype(can_msg.dlc)>(can_msg_def.MessageSize());
    // If id is bigger than 0x7FF, then we can use up to 29 bits for the can_id.
    can_msg.is_extended = can_msg.id > 0x000007FF ? true : false;

    auto sigs = can_msg_def.Signals_Size();

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "GuidanceVelocityCommand")
      {
        sig.Encode(sig.PhysToRaw(twist_cmd_msg->linear.x), can_msg.data.data());  // In m/s
      }
      else  // GuidanceYawRateCommand
      {
        sig.Encode(sig.PhysToRaw(twist_cmd_msg->angular.z), can_msg.data.data());  // In rad/s
      }
    }

    // This publication is for debugging purposes.
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
