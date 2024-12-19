#ifndef LIDAR_REPORTER_H_
#define LIDAR_REPORTER_H_

#pragma once

#include <chrono>
#include "r4c_reporter/reporter_commons.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_reporter
{
  class LidarReporter : public rclcpp::Node {
  public:
      LidarReporter(const std::string& node_name);

  private:
      bool report_;
      std::string sensor_name_;
      std::string topic_;
      bool msg_received_ = false;
      double sync_threshold_;
      double timeout_;
      double check_frequency_;


      r4c_interfaces::msg::SensorStatus lidar_status_;

      sensor_msgs::msg::PointCloud2::SharedPtr sensor_msg_;

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_msg_sub_;

      rclcpp::Publisher<r4c_interfaces::msg::SensorStatus>::SharedPtr sensor_status_pub_;

      void timerCallback();
      void sensor_msg_callback(const sensor_msgs::msg::PointCloud2::SharedPtr sensor_msg);

  };

}  // namespace lidar_reporter

#endif