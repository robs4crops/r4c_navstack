#include "r4c_reporter/can_reporter.hpp"

namespace can_reporter
{
  CanReporter::CanReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    sync_threshold_{this->declare_parameter("sync_threshold", 100.0)},
    timeout_{this->declare_parameter("timeout", 5.0)},
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    can_msg_sub_{this->create_subscription<can_msgs::msg::Frame>(
      this->declare_parameter("can_topic", "/can"),
      10,
      std::bind(&CanReporter::can_msg_callback, this, std::placeholders::_1))},
    can_status_pub_{this->create_publisher<r4c_interfaces::msg::CanStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/can/status"),
      10)}
  {
    can_status_.name       = "can";
    can_msg_               = std::make_shared<can_msgs::msg::Frame>();
    can_msg_->header.stamp = this->now();
    state_transitions::changeStateToInactive<r4c_interfaces::msg::CanStatus>(can_status_);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&CanReporter::timerCallback, this));
  }

  void CanReporter::can_msg_callback(const can_msgs::msg::Frame::SharedPtr can_msg)
  {
    msg_received_ = true;
    can_msg_      = can_msg;
  }

  void CanReporter::timerCallback()
  {
    if(msg_received_)
    {
      msg_received_ = false;
      state_transitions::changeStateToOk<r4c_interfaces::msg::CanStatus>(can_status_);
    }

    rclcpp::Time current_time = this->now();

    reporter_commons::evaluateConnection<can_msgs::msg::Frame, r4c_interfaces::msg::CanStatus>(
      can_status_,
      can_msg_,
      current_time,
      timeout_);
    reporter_commons::evaluateSynchronization<can_msgs::msg::Frame, r4c_interfaces::msg::CanStatus>(
      can_status_,
      can_msg_,
      current_time,
      sync_threshold_);

    can_status_.header.stamp = this->now();

    can_status_pub_->publish(can_status_);
  }

}  // namespace can_reporter
