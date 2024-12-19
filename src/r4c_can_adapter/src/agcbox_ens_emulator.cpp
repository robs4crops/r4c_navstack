#include <chrono>
#include <fstream>
#include <functional>
#include <limits>

#include "r4c_can_adapter/agcbox_ens_emulator.hpp"

namespace r4c_can_adapter
{
  AgcboxEnsEmulator::AgcboxEnsEmulator():
    rclcpp::Node{"agcbox_ens_emulator"},
    ens_control_sub_{this->create_subscription<r4c_msgs::msg::AgcboxEnsControl>(
      this->declare_parameter("ens_control_topic", "agcbox/ens/control"),
      10,
      std::bind(&AgcboxEnsEmulator::rcv_ens_control_msg, this, std::placeholders::_1))},
    ens_status_pub_{this->create_publisher<r4c_msgs::msg::AgcboxEnsStatus>(
      this->declare_parameter("ens_status_topic", "agcbox/ens/status"),
      10)},
    manual_mode_pub_{this->create_publisher<std_msgs::msg::Bool>(
      this->declare_parameter("manual_mode_topic", "manual_mode"),
      10)}
  {
    auto manual_mode_publishing_rate = this->declare_parameter("manual_mode_publishing_rate", 10.0);
    auto manual_mode_publishing_period_ms = static_cast<int>(1000.0 *
                                                             (1.0 / manual_mode_publishing_rate));
    timer_ = create_wall_timer(std::chrono::milliseconds(manual_mode_publishing_period_ms),
                               std::bind(&AgcboxEnsEmulator::timer_cb, this));
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void AgcboxEnsEmulator::rcv_ens_control_msg(
    const r4c_msgs::msg::AgcboxEnsControl::ConstSharedPtr ens_control_msg)
  {
    // The cases presented in this function were provided by AGC in order to put the Carob robot
    // in auto mode.

    if(!ens_control_msg->go_to_auto && !ens_control_msg->motion_authorized && caius_in_auto_mode)
    {
      RCLCPP_INFO(this->get_logger(), "AGC robot in manual mode");

      snd_ens_status_msg(false, r4c_msgs::msg::AgcboxEnsStatus::STANDBY);

      caius_in_auto_mode                      = false;
      waiting_caius_in_auto_mode_confirmation = false;
    }
    else if(ens_control_msg->go_to_auto &&          //
            !ens_control_msg->motion_authorized &&  //
            !waiting_caius_in_auto_mode_confirmation)
    {
      RCLCPP_INFO(this->get_logger(), "Trying to set AGC robot in auto mode");

      snd_ens_status_msg(false, r4c_msgs::msg::AgcboxEnsStatus::AUTO_STANDBY);

      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      snd_ens_status_msg(true, r4c_msgs::msg::AgcboxEnsStatus::AUTO_STANDBY);

      waiting_caius_in_auto_mode_confirmation = true;
    }
    else if(ens_control_msg->go_to_auto &&         //
            ens_control_msg->motion_authorized &&  //
            !caius_in_auto_mode)
    {
      RCLCPP_WARN(this->get_logger(), "AGC robot is in auto mode");

      caius_in_auto_mode                      = true;
      waiting_caius_in_auto_mode_confirmation = false;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void AgcboxEnsEmulator::snd_ens_status_msg(bool request_motion, uint8_t status_word)
  {
    r4c_msgs::msg::AgcboxEnsStatus agcbox_ens_status_msg;

    agcbox_ens_status_msg.header.frame_id = "nobu_link";
    agcbox_ens_status_msg.header.stamp    = this->now();
    agcbox_ens_status_msg.request_motion  = request_motion;
    agcbox_ens_status_msg.status_word     = status_word;
    agcbox_ens_status_msg.mission_width   = 2.0;

    ens_status_pub_->publish(agcbox_ens_status_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void AgcboxEnsEmulator::timer_cb(){
    std_msgs::msg::Bool manual_mode;
    manual_mode.data = !caius_in_auto_mode;

    manual_mode_pub_->publish(manual_mode);
  }
}  // namespace r4c_can_adapter
