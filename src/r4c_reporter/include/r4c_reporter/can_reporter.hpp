#ifndef CAN_REPORTER_H_
#define CAN_REPORTER_H_

#pragma once

#include <chrono>
#include "r4c_reporter/reporter_commons.hpp"

#include <can_msgs/msg/frame.hpp>

namespace can_reporter
{
  class CanReporter : public rclcpp::Node {
  public:
      CanReporter(const std::string& node_name);

  private:
      bool msg_received_ = false;
      double sync_threshold_;
      double timeout_;
      double check_frequency_;

      r4c_interfaces::msg::CanStatus can_status_;

      can_msgs::msg::Frame::SharedPtr can_msg_;

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_msg_sub_;

      rclcpp::Publisher<r4c_interfaces::msg::CanStatus>::SharedPtr can_status_pub_;

      void timerCallback();
      void can_msg_callback(const can_msgs::msg::Frame::SharedPtr can_msg);


  };

}  // namespace can_reporter

#endif