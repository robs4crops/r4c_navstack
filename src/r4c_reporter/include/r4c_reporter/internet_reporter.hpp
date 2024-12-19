#ifndef INTERNET_REPORTER_H_
#define INTERNET_REPORTER_H_

#pragma once

#include <chrono>
#include "r4c_reporter/reporter_commons.hpp"

namespace internet_reporter
{
  class InternetReporter : public rclcpp::Node {
  public:
      InternetReporter(const std::string& node_name);

  private:
      double timeout_;
      double check_frequency_;

      r4c_interfaces::msg::InternetStatus internet_status_;

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Publisher<r4c_interfaces::msg::InternetStatus>::SharedPtr status_pub_;

      void timerCallback();

  };

}  // namespace internet_reporter

#endif