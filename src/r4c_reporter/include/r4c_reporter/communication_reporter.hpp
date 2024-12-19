#ifndef COMMUNICATION_REPORTER_H_
#define COMMUNICATION_REPORTER_H_

#pragma once

#include <chrono>
#include <mutex>
#include "r4c_reporter/reporter_commons.hpp"
namespace communication_reporter
{
  class CommunicationReporter: public rclcpp::Node
  {
    public:
    CommunicationReporter(const std::string& node_name);

    private:
    double check_frequency_;
    rclcpp::TimerBase::SharedPtr timer_;

    r4c_interfaces::msg::CommunicationStatus communication_status_;

    rclcpp::Subscription<r4c_interfaces::msg::InternetStatus>::SharedPtr internet_status_sub_;
    rclcpp::Subscription<r4c_interfaces::msg::CanStatus>::SharedPtr can_status_sub_;

    rclcpp::Publisher<r4c_interfaces::msg::CommunicationStatus>::SharedPtr
      communication_status_pub_;

    void timerCallback();
    void internet_status_callback(const r4c_interfaces::msg::InternetStatus::SharedPtr status_msg);
    void can_status_callback(const r4c_interfaces::msg::CanStatus::SharedPtr status_msg);
    void processStatus(r4c_interfaces::msg::CommunicationStatus& status);

    std::mutex communication_mutex_;
  };

}  // namespace communication_reporter

#endif
