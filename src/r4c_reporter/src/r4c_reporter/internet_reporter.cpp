#include "r4c_reporter/internet_reporter.hpp"

namespace internet_reporter
{
  InternetReporter::InternetReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    timeout_{this->declare_parameter("timeout", 5.0)},
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    status_pub_{this->create_publisher<r4c_interfaces::msg::InternetStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/connection/internet/status"),
      10)}
  {
    internet_status_.name = "internet";
    state_transitions::changeStateToInactive<r4c_interfaces::msg::InternetStatus>(internet_status_);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&InternetReporter::timerCallback, this));
  }

  void InternetReporter::timerCallback()
  {
    if(system("ping 8.8.8.8 -W 1 -c 1 > nul") == 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "INTERNET");
      state_transitions::removeErrorCode<r4c_interfaces::msg::InternetStatus>(
        r4c_interfaces::msg::InternetStatus::NO_INTERNET_CONNECTION,
        internet_status_);
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "NO INTERNET");
      state_transitions::addErrorCode<r4c_interfaces::msg::InternetStatus>(
        r4c_interfaces::msg::InternetStatus::NO_INTERNET_CONNECTION,
        internet_status_);
    }

    internet_status_.header.stamp = this->now();

    status_pub_->publish(internet_status_);
  }

}  // namespace internet_reporter
