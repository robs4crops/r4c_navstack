#include "r4c_reporter/tractor_reporter.hpp"

namespace tractor_reporter
{
  TractorReporter::TractorReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    sensors_status_sub_{this->create_subscription<r4c_interfaces::msg::SensorsStatus>(
      this->declare_parameter("sensors_status_topic", "sensors/status"),
      10,
      std::bind(&TractorReporter::sensors_status_callback, this, std::placeholders::_1))},
    communication_status_sub_{this->create_subscription<r4c_interfaces::msg::CommunicationStatus>(
      this->declare_parameter("communication_status_topic", "comms/status"),
      10,
      std::bind(&TractorReporter::communication_status_callback, this, std::placeholders::_1))},
    localization_status_sub_{this->create_subscription<r4c_interfaces::msg::LocalizationStatus>(
      this->declare_parameter("localization_status_topic", "localization/status"),
      10,
      std::bind(&TractorReporter::localization_status_callback, this, std::placeholders::_1))},
    tractor_status_pub_{this->create_publisher<r4c_interfaces::msg::TractorStatus>(
      this->declare_parameter("status_topic", "platform/status"),
      10)}
  {
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&TractorReporter::timerCallback, this));
  }

  void TractorReporter::sensors_status_callback(
    const r4c_interfaces::msg::SensorsStatus::SharedPtr status_msg)
  {
    std::lock_guard<std::mutex> lock(tractor_mutex_);
    tractor_status_.sensors_status = *status_msg;
  }

  void TractorReporter::communication_status_callback(
    const r4c_interfaces::msg::CommunicationStatus::SharedPtr status_msg)
  {
    std::lock_guard<std::mutex> lock(tractor_mutex_);
    tractor_status_.communication_status = *status_msg;
  }

  void TractorReporter::localization_status_callback(
    const r4c_interfaces::msg::LocalizationStatus::SharedPtr status_msg)
  {
    std::lock_guard<std::mutex> lock(tractor_mutex_);
    tractor_status_.localization_status = *status_msg;
  }

  void TractorReporter::timerCallback()
  {
    std::lock_guard<std::mutex> lock(tractor_mutex_);
    {
      processStatus(tractor_status_);

      tractor_status_.header.stamp = this->now();

      tractor_status_pub_->publish(tractor_status_);
    }
  }

  void TractorReporter::processStatus(r4c_interfaces::msg::TractorStatus& status)
  {
    int warning_status = 0, error_status = 0;

    if(status.sensors_status.state == r4c_interfaces::msg::SensorsStatus::STATE_ERROR)
      error_status++;
    else if(status.sensors_status.state == r4c_interfaces::msg::SensorsStatus::STATE_WARNING)
      warning_status++;

    if(status.communication_status.state == r4c_interfaces::msg::CommunicationStatus::STATE_ERROR)
      error_status++;
    else if(status.communication_status.state ==
            r4c_interfaces::msg::CommunicationStatus::STATE_WARNING)
      warning_status++;

    if(status.localization_status.state == r4c_interfaces::msg::LocalizationStatus::STATE_ERROR)
      error_status++;
    else if(status.localization_status.state ==
            r4c_interfaces::msg::LocalizationStatus::STATE_WARNING)
      warning_status++;

    if(error_status > 0)
      status.state = r4c_interfaces::msg::TractorStatus::STATE_ERROR;
    else if(warning_status > 0)
      status.state = r4c_interfaces::msg::TractorStatus::STATE_WARNING;
    else
      status.state = r4c_interfaces::msg::TractorStatus::STATE_OK;
  }


}  // namespace tractor_reporter