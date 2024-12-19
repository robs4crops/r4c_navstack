#include "r4c_reporter/localization_reporter.hpp"

namespace localization_reporter
{
  LocalizationReporter::LocalizationReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    covariance_threshold_{this->declare_parameter("covariance_threshold", 100.0)},
    timeout_{this->declare_parameter("timeout", 5.0)},
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    odom_msg_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
      this->declare_parameter("odom_topic", "odom"),
      10,
      std::bind(&LocalizationReporter::odom_msg_callback, this, std::placeholders::_1))},
    gnss_msg_sub_{this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("gnss_topic", "gnss"),
      10,
      std::bind(&LocalizationReporter::gnss_msg_callback, this, std::placeholders::_1))},
    heading_msg_sub_{this->create_subscription<sensor_msgs::msg::Imu>(
      this->declare_parameter("heading_topic", "heading"),
      10,
      std::bind(&LocalizationReporter::heading_msg_callback, this, std::placeholders::_1))},
    localization_status_pub_{this->create_publisher<r4c_interfaces::msg::LocalizationStatus>(
      this->declare_parameter("status_topic", "localization_status"),
      10)}
  {
    localization_status_.name = "localization";
    odom_msg_                 = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg_->header.stamp   = this->now();
    state_transitions::changeStateToInactive<r4c_interfaces::msg::LocalizationStatus>(
      localization_status_);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&LocalizationReporter::timerCallback, this));
  }

  void LocalizationReporter::odom_msg_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    msg_received_ = true;
    odom_msg_     = odom_msg;
  }

  void LocalizationReporter::gnss_msg_callback(
    const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg)
  {
    received_first_gnss_ = true;
    // Remove subscription once the first message has been received, since from that moment
    // onwards, there is no need to received more messages of this type.
    gnss_msg_sub_.reset();
  }

  void LocalizationReporter::heading_msg_callback(
    const sensor_msgs::msg::Imu::SharedPtr heading_msg)
  {
    received_first_heading_ = true;
    // Remove subscription once the first message has been received, since from that moment
    // onwards, there is no need to received more messages of this type.
    heading_msg_sub_.reset();
  }

  void LocalizationReporter::timerCallback()
  {
    // When an odometry message is received, by default, we set the localizacion status to OK.
    // After that, we check some conditions to enable warnings and or errors.

    if(msg_received_)
    {
      msg_received_ = false;
      state_transitions::changeStateToOk<r4c_interfaces::msg::LocalizationStatus>(
        localization_status_);
    }

    checkOdometryCovariance(localization_status_, odom_msg_, covariance_threshold_);
    checkInitialMessages(localization_status_);

    localization_status_.header.stamp = this->now();

    localization_status_pub_->publish(localization_status_);
  }

  void LocalizationReporter::checkOdometryCovariance(
    r4c_interfaces::msg::LocalizationStatus& localization_status,
    nav_msgs::msg::Odometry::SharedPtr& odom_msg,
    double covariance_threshold)
  {
    if((odom_msg->pose.covariance[0] > covariance_threshold) ||
       (odom_msg->pose.covariance[7] > covariance_threshold) ||
       (odom_msg->pose.covariance[35] > covariance_threshold))
    {
      state_transitions::addErrorCode<r4c_interfaces::msg::LocalizationStatus>(
        r4c_interfaces::msg::LocalizationStatus::HIGH_POSITIONING_COVARIANCE,
        localization_status);
    }
    else
    {
      state_transitions::removeErrorCode<r4c_interfaces::msg::LocalizationStatus>(
        r4c_interfaces::msg::LocalizationStatus::HIGH_POSITIONING_COVARIANCE,
        localization_status);
    }
  }

  void LocalizationReporter::checkInitialMessages(
    r4c_interfaces::msg::LocalizationStatus& localization_status)
  {
    if(received_first_gnss_)
    {
      state_transitions::removeWarningCode<r4c_interfaces::msg::LocalizationStatus>(
        r4c_interfaces::msg::LocalizationStatus::FIRST_GNSS_MSG_NOT_RECEIVED,
        localization_status);
    }
    else
    {
      state_transitions::addWarningCode<r4c_interfaces::msg::LocalizationStatus>(
        r4c_interfaces::msg::LocalizationStatus::FIRST_GNSS_MSG_NOT_RECEIVED,
        localization_status);
    }

    if(received_first_heading_)
    {
      state_transitions::removeWarningCode<r4c_interfaces::msg::LocalizationStatus>(
        r4c_interfaces::msg::LocalizationStatus::FIRST_HEADING_MSG_NOT_RECEIVED,
        localization_status);
    }
    else
    {
      state_transitions::addWarningCode<r4c_interfaces::msg::LocalizationStatus>(
        r4c_interfaces::msg::LocalizationStatus::FIRST_HEADING_MSG_NOT_RECEIVED,
        localization_status);
    }
  }
}  // namespace localization_reporter
