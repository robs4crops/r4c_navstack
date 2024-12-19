#include "r4c_reporter/sensors_reporter.hpp"

namespace sensors_reporter
{
  SensorsReporter::SensorsReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    oakd_status_sub_{this->create_subscription<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("oakd_status_topic", "/oakd/status"),
      10,
      std::bind(&SensorsReporter::oakd_status_callback, this, std::placeholders::_1))},
    um7_status_sub_{this->create_subscription<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("um7_status_topic", "/um7/status"),
      10,
      std::bind(&SensorsReporter::um7_status_callback, this, std::placeholders::_1))},
    ublox_status_sub_{this->create_subscription<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("ublox_status_topic", "/ublox/status"),
      10,
      std::bind(&SensorsReporter::ublox_status_callback, this, std::placeholders::_1))},
    livox_status_sub_{this->create_subscription<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("livox_status_topic", "/livox/status"),
      10,
      std::bind(&SensorsReporter::livox_status_callback, this, std::placeholders::_1))},
    rslidar_status_sub_{this->create_subscription<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("rslidar_status_topic", "/rslidar/status"),
      10,
      std::bind(&SensorsReporter::rslidar_status_callback, this, std::placeholders::_1))},
    sensors_status_pub_{this->create_publisher<r4c_interfaces::msg::SensorsStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/sensors/status"),
      10)}
  {
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&SensorsReporter::timerCallback, this));
  }

  void SensorsReporter::oakd_status_callback(
    const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg)
  {
    std::lock_guard<std::mutex> lock(sensors_mutex_);
    sensors_status_.oakd_status = *sensor_status_msg;
  }

  void SensorsReporter::um7_status_callback(
    const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg)
  {
    std::lock_guard<std::mutex> lock(sensors_mutex_);
    sensors_status_.um7_status = *sensor_status_msg;
  }

  void SensorsReporter::ublox_status_callback(
    const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg)
  {
    std::lock_guard<std::mutex> lock(sensors_mutex_);
    sensors_status_.ublox_status = *sensor_status_msg;
  }

  void SensorsReporter::livox_status_callback(
    const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg)
  {
    std::lock_guard<std::mutex> lock(sensors_mutex_);
    sensors_status_.livox_status = *sensor_status_msg;
  }

  void SensorsReporter::rslidar_status_callback(
    const r4c_interfaces::msg::SensorStatus::SharedPtr sensor_status_msg)
  {
    std::lock_guard<std::mutex> lock(sensors_mutex_);
    sensors_status_.rslidar_status = *sensor_status_msg;
  }

  void SensorsReporter::timerCallback()
  {
    std::lock_guard<std::mutex> lock(sensors_mutex_);
    {
      processStatus(sensors_status_);

      sensors_status_.header.stamp = this->now();

      sensors_status_pub_->publish(sensors_status_);
    }
  }

  void SensorsReporter::processStatus(r4c_interfaces::msg::SensorsStatus& status)
  {
    int warning_status = 0, error_status = 0;

    for(r4c_interfaces::msg::SensorStatus sensor_status: {status.oakd_status,
                                                          status.um7_status,
                                                          status.rslidar_status,
                                                          status.livox_status,
                                                          status.ublox_status})
    {
      if(sensor_status.state == r4c_interfaces::msg::SensorStatus::STATE_ERROR)
        error_status++;
      else if(sensor_status.state == r4c_interfaces::msg::SensorStatus::STATE_WARNING)
        warning_status++;
    }

    if(error_status > 0)
      status.state = r4c_interfaces::msg::SensorsStatus::STATE_ERROR;
    else if(warning_status > 0)
      status.state = r4c_interfaces::msg::SensorsStatus::STATE_WARNING;
    else
      status.state = r4c_interfaces::msg::SensorsStatus::STATE_OK;
  }


}  // namespace sensors_reporter