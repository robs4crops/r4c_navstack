#ifndef SENSORS_REPORTER_H_
#define SENSORS_REPORTER_H_

#pragma once

#include <chrono>
#include <mutex>
#include "r4c_reporter/reporter_commons.hpp"
namespace sensors_reporter
{
  class SensorsReporter : public rclcpp::Node {
  public:
      SensorsReporter(const std::string& node_name);

  private:

      double check_frequency_;
      rclcpp::TimerBase::SharedPtr timer_;

      r4c_interfaces::msg::SensorsStatus sensors_status_;

      bool use_oakd_, use_rslidar_, use_livox_, use_um7_, use_ublox_;

      rclcpp::Subscription<r4c_interfaces::msg::SensorStatus>::SharedPtr oakd_status_sub_;
      rclcpp::Subscription<r4c_interfaces::msg::SensorStatus>::SharedPtr um7_status_sub_;
      rclcpp::Subscription<r4c_interfaces::msg::SensorStatus>::SharedPtr ublox_status_sub_;
      rclcpp::Subscription<r4c_interfaces::msg::SensorStatus>::SharedPtr livox_status_sub_;
      rclcpp::Subscription<r4c_interfaces::msg::SensorStatus>::SharedPtr rslidar_status_sub_;

      rclcpp::Publisher<r4c_interfaces::msg::SensorsStatus>::SharedPtr sensors_status_pub_;

      void timerCallback();
      void oakd_status_callback(const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg);
      void um7_status_callback(const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg);
      void ublox_status_callback(const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg);
      void livox_status_callback(const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg);
      void rslidar_status_callback(const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg);
      void processStatus(r4c_interfaces::msg::SensorsStatus& status);

      std::mutex sensors_mutex_;

  };

}  // namespace sensors_reporter

#endif
