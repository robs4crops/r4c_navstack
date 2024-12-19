#pragma once

#include <chrono>
#include "r4c_reporter/reporter_commons.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace localization_reporter
{
  class LocalizationReporter: public rclcpp::Node
  {
    public:
    LocalizationReporter(const std::string& node_name);

    private:
    std::string topic_;
    bool msg_received_{false};
    double covariance_threshold_;
    double timeout_;
    double check_frequency_;

    bool received_first_gnss_{false};
    bool received_first_heading_{false};

    r4c_interfaces::msg::LocalizationStatus localization_status_;

    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_msg_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_msg_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_msg_sub_;

    rclcpp::Publisher<r4c_interfaces::msg::LocalizationStatus>::SharedPtr localization_status_pub_;

    void timerCallback();
    void odom_msg_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void gnss_msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg);
    void heading_msg_callback(const sensor_msgs::msg::Imu::SharedPtr heading_msg);

    void checkOdometryCovariance(r4c_interfaces::msg::LocalizationStatus& localization_status,
                                 nav_msgs::msg::Odometry::SharedPtr& odom_msg,
                                 double covariance_threshold);
    void checkInitialMessages(r4c_interfaces::msg::LocalizationStatus& localization_status);
  };

}  // namespace localization_reporter