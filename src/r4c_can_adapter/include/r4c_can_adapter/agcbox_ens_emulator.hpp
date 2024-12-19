#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/bool.hpp>

#include <r4c_msgs/msg/agcbox_ens_control.hpp>
#include <r4c_msgs/msg/agcbox_ens_status.hpp>

namespace r4c_can_adapter
{
  class AgcboxEnsEmulator: public rclcpp::Node
  {
    public:
    AgcboxEnsEmulator();

    void rcv_ens_control_msg(const r4c_msgs::msg::AgcboxEnsControl::ConstSharedPtr ens_control_msg);

    void timer_cb();

    private:
    void snd_ens_status_msg(bool request_motion, uint8_t status_word);
    //---------------------------------

    bool caius_in_auto_mode{false};
    uint8_t agcbox_spe_node_id_;
    bool waiting_caius_in_auto_mode_confirmation{false};

    rclcpp::Subscription<r4c_msgs::msg::AgcboxEnsControl>::SharedPtr ens_control_sub_;

    rclcpp::Publisher<r4c_msgs::msg::AgcboxEnsStatus>::SharedPtr ens_status_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
  };

}  // namespace r4c_can_adapter
