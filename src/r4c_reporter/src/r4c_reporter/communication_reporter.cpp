#include "r4c_reporter/communication_reporter.hpp"

namespace communication_reporter
{
  CommunicationReporter::CommunicationReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    internet_status_sub_{this->create_subscription<r4c_interfaces::msg::InternetStatus>(
      this->declare_parameter("internet_status_topic", "/internet/status"),
      10,
      std::bind(&CommunicationReporter::internet_status_callback, this, std::placeholders::_1))},
    can_status_sub_{this->create_subscription<r4c_interfaces::msg::CanStatus>(
      this->declare_parameter("can_status_topic", "/can/status"),
      10,
      std::bind(&CommunicationReporter::can_status_callback, this, std::placeholders::_1))},
    communication_status_pub_{this->create_publisher<r4c_interfaces::msg::CommunicationStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/comms/status"),
      10)}
  {
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&CommunicationReporter::timerCallback, this));
  }

  void CommunicationReporter::internet_status_callback(
    const r4c_interfaces::msg::InternetStatus::SharedPtr status_msg)
  {
    std::lock_guard<std::mutex> lock(communication_mutex_);
    communication_status_.internet_status = *status_msg;
  }

  void CommunicationReporter::can_status_callback(
    const r4c_interfaces::msg::CanStatus::SharedPtr status_msg)
  {
    std::lock_guard<std::mutex> lock(communication_mutex_);
    communication_status_.can_status = *status_msg;
  }

  void CommunicationReporter::timerCallback()
  {
    std::lock_guard<std::mutex> lock(communication_mutex_);
    {
      processStatus(communication_status_);

      communication_status_.header.stamp = this->now();

      communication_status_pub_->publish(communication_status_);
    }
  }

  void CommunicationReporter::processStatus(r4c_interfaces::msg::CommunicationStatus& status)
  {
    int warning_status = 0, error_status = 0;

    if(status.internet_status.state == r4c_interfaces::msg::InternetStatus::STATE_ERROR)
      error_status++;
    else if(status.internet_status.state == r4c_interfaces::msg::InternetStatus::STATE_WARNING)
      warning_status++;

    if(status.can_status.state == r4c_interfaces::msg::CanStatus::STATE_ERROR)
      error_status++;
    else if(status.can_status.state == r4c_interfaces::msg::CanStatus::STATE_WARNING)
      warning_status++;

    if(error_status > 0)
      status.state = r4c_interfaces::msg::CommunicationStatus::STATE_ERROR;
    else if(warning_status > 0)
      status.state = r4c_interfaces::msg::CommunicationStatus::STATE_WARNING;
    else
      status.state = r4c_interfaces::msg::CommunicationStatus::STATE_OK;
  }


}  // namespace communication_reporter