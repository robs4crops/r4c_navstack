#include "r4c_reporter/imu_reporter.hpp"

namespace imu_reporter
{
  ImuReporter::ImuReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    report_{this->declare_parameter("report", false)},
    sensor_name_{this->declare_parameter("sensor_name", "imu")},
    sync_threshold_{this->declare_parameter("sync_threshold", 100.0)},
    timeout_{this->declare_parameter("timeout", 5.0)},
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    sensor_msg_sub_{this->create_subscription<sensor_msgs::msg::Imu>(
      this->declare_parameter("sensor_topic", "/imu"),
      10,
      std::bind(&ImuReporter::sensor_msg_callback, this, std::placeholders::_1))},
    sensor_status_pub_{this->create_publisher<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/imu/status"),
      10)}
  {
    imu_status_.name          = sensor_name_;
    sensor_msg_               = std::make_shared<sensor_msgs::msg::Imu>();
    sensor_msg_->header.stamp = this->now();
    state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(imu_status_);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&ImuReporter::timerCallback, this));
  }

  void ImuReporter::sensor_msg_callback(const sensor_msgs::msg::Imu::SharedPtr sensor_msg)
  {
    msg_received_ = true;
    sensor_msg_   = sensor_msg;
  }

  void ImuReporter::timerCallback()
  {
    if(msg_received_)
    {
      msg_received_ = false;
      state_transitions::changeStateToOk<r4c_interfaces::msg::SensorStatus>(imu_status_);
    }

    rclcpp::Time current_time = this->now();

    if(!report_)
    {
      state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(imu_status_);
    }
    else
    {
      reporter_commons::evaluateConnection<sensor_msgs::msg::Imu,
                                           r4c_interfaces::msg::SensorStatus>(imu_status_,
                                                                              sensor_msg_,
                                                                              current_time,
                                                                              timeout_);
      reporter_commons::evaluateSynchronization<sensor_msgs::msg::Imu,
                                                r4c_interfaces::msg::SensorStatus>(imu_status_,
                                                                                   sensor_msg_,
                                                                                   current_time,
                                                                                   sync_threshold_);
    }

    imu_status_.header.stamp = this->now();

    sensor_status_pub_->publish(imu_status_);
  }

}  // namespace imu_reporter
