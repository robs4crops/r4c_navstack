#include "r4c_reporter/lidar_reporter.hpp"

namespace lidar_reporter
{
  LidarReporter::LidarReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    report_{this->declare_parameter("report", false)},
    sensor_name_{this->declare_parameter("sensor_name", "lidar")},
    sync_threshold_{this->declare_parameter("sync_threshold", 100.0)},
    timeout_{this->declare_parameter("timeout", 5.0)},
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    sensor_msg_sub_{this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->declare_parameter("sensor_topic", "/lidar/points"),
      10,
      std::bind(&LidarReporter::sensor_msg_callback, this, std::placeholders::_1))},
    sensor_status_pub_{this->create_publisher<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/lidar/status"),
      10)}
  {
    lidar_status_.name        = sensor_name_;
    sensor_msg_               = std::make_shared<sensor_msgs::msg::PointCloud2>();
    sensor_msg_->header.stamp = this->now();
    state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(lidar_status_);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&LidarReporter::timerCallback, this));
  }

  void LidarReporter::sensor_msg_callback(const sensor_msgs::msg::PointCloud2::SharedPtr sensor_msg)
  {
    msg_received_ = true;
    sensor_msg_   = sensor_msg;
  }

  void LidarReporter::timerCallback()
  {
    if(msg_received_)
    {
      msg_received_ = false;
      state_transitions::changeStateToOk<r4c_interfaces::msg::SensorStatus>(lidar_status_);
    }

    rclcpp::Time current_time = this->now();

    if(!report_)
    {
      state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(lidar_status_);
    }
    else
    {
      reporter_commons::evaluateConnection<sensor_msgs::msg::PointCloud2>(lidar_status_,
                                                                          sensor_msg_,
                                                                          current_time,
                                                                          timeout_);
      reporter_commons::evaluateSynchronization<sensor_msgs::msg::PointCloud2>(lidar_status_,
                                                                               sensor_msg_,
                                                                               current_time,
                                                                               sync_threshold_);
    }

    lidar_status_.header.stamp = this->now();

    sensor_status_pub_->publish(lidar_status_);
  }

}  // namespace lidar_reporter
