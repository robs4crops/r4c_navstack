#ifndef CAMERA_REPORTER_H_
#define CAMERA_REPORTER_H_

#pragma once

#include <chrono>
#include "r4c_reporter/reporter_commons.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace camera_reporter
{
  class CameraReporter : public rclcpp::Node {
  public:
      CameraReporter(const std::string& node_name);

  private:
      bool report_;
      std::string sensor_name_;
      std::string topic_;
      bool msg_received_ = false;
      double sync_threshold_;
      double timeout_;
      double check_frequency_;
      double occlusion_thresh_;

      r4c_interfaces::msg::SensorStatus camera_status_;

      sensor_msgs::msg::Image::SharedPtr sensor_msg_;

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sensor_msg_sub_;

      rclcpp::Publisher<r4c_interfaces::msg::SensorStatus>::SharedPtr sensor_status_pub_;

      void timerCallback();
      void sensor_msg_callback(const sensor_msgs::msg::Image::SharedPtr sensor_msg);

      void evaluateOcclusion(r4c_interfaces::msg::SensorStatus& sensor_status, sensor_msgs::msg::Image::SharedPtr& sensor_msg);

  };

}  // namespace camera_reporter

#endif