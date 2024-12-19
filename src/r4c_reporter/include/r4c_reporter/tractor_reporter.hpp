#ifndef TRACTOR_REPORTER_H_
#define TRACTOR_REPORTER_H_

#pragma once

#include <chrono>
#include <mutex>
#include "r4c_reporter/reporter_commons.hpp"
namespace tractor_reporter
{
  class TractorReporter : public rclcpp::Node {
  public:
      TractorReporter(const std::string& node_name);

  private:

      double check_frequency_;
      rclcpp::TimerBase::SharedPtr timer_;

      r4c_interfaces::msg::TractorStatus tractor_status_;

      rclcpp::Subscription<r4c_interfaces::msg::SensorsStatus>::SharedPtr sensors_status_sub_;
      rclcpp::Subscription<r4c_interfaces::msg::CommunicationStatus>::SharedPtr communication_status_sub_;
      rclcpp::Subscription<r4c_interfaces::msg::LocalizationStatus>::SharedPtr localization_status_sub_;

      rclcpp::Publisher<r4c_interfaces::msg::TractorStatus>::SharedPtr tractor_status_pub_;

      void timerCallback();
      void sensors_status_callback(const r4c_interfaces::msg::SensorsStatus::SharedPtr status_msg);
      void communication_status_callback(const r4c_interfaces::msg::CommunicationStatus::SharedPtr status_msg);
      void localization_status_callback(const r4c_interfaces::msg::LocalizationStatus::SharedPtr status_msg);
      void processStatus(r4c_interfaces::msg::TractorStatus& status);

      std::mutex tractor_mutex_;

  };

}  // namespace tractor_reporter

#endif
