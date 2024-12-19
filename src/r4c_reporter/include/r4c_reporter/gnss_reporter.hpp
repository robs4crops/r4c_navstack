#ifndef GNSS_REPORTER_H_
#define GNSS_REPORTER_H_

#pragma once

#include <chrono>
#include "r4c_reporter/reporter_commons.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <rtcm_msgs/msg/message.hpp>

namespace gnss_reporter
{
  class GNSSReporter : public rclcpp::Node {
  public:
      GNSSReporter(const std::string& node_name);

  private:
      bool report_;
      std::string sensor_name_;
      std::string topic_;
      bool msg_received_ = false;
      double sync_threshold_;
      double timeout_;
      double check_frequency_;

      r4c_interfaces::msg::SensorStatus gnss_status_;

      sensor_msgs::msg::NavSatFix::SharedPtr sensor_msg_;
      rtcm_msgs::msg::Message::SharedPtr rtcm_msg_;

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sensor_msg_sub_;
      rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_msg_sub_;

      rclcpp::Publisher<r4c_interfaces::msg::SensorStatus>::SharedPtr sensor_status_pub_;

      void timerCallback();
      void sensor_msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr sensor_msg);
      void rtcm_msg_callback(const rtcm_msgs::msg::Message::SharedPtr rtcm_msg);

      void evaluateFix(r4c_interfaces::msg::SensorStatus& sensor_status, sensor_msgs::msg::NavSatFix::SharedPtr& sensor_msg);
      void evaluateCorrections(r4c_interfaces::msg::SensorStatus& sensor_status, rtcm_msgs::msg::Message::SharedPtr& rtcm_msg, double timeout);

  };

}  // namespace gnss_reporter

#endif