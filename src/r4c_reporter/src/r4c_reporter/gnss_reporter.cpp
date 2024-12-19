#include "r4c_reporter/gnss_reporter.hpp"

namespace gnss_reporter
{
  GNSSReporter::GNSSReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    report_{this->declare_parameter("report", false)},
    sensor_name_{this->declare_parameter("sensor_name", "gnss")},
    sync_threshold_{this->declare_parameter("sync_threshold", 100.0)},
    timeout_{this->declare_parameter("timeout", 5.0)},
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    sensor_msg_sub_{this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("sensor_topic", "/gnss"),
      10,
      std::bind(&GNSSReporter::sensor_msg_callback, this, std::placeholders::_1))},
    rtcm_msg_sub_{this->create_subscription<rtcm_msgs::msg::Message>(
      this->declare_parameter("rtcm_topic", "/gnss"),
      10,
      std::bind(&GNSSReporter::rtcm_msg_callback, this, std::placeholders::_1))},
    sensor_status_pub_{this->create_publisher<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/gnss/status"),
      10)}
  {
    gnss_status_.name         = sensor_name_;
    sensor_msg_               = std::make_shared<sensor_msgs::msg::NavSatFix>();
    sensor_msg_->header.stamp = this->now();
    state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(gnss_status_);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&GNSSReporter::timerCallback, this));
  }

  void GNSSReporter::sensor_msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr sensor_msg)
  {
    msg_received_ = true;
    sensor_msg_   = sensor_msg;
  }

  void GNSSReporter::rtcm_msg_callback(const rtcm_msgs::msg::Message::SharedPtr rtcm_msg)
  {
    msg_received_ = true;
    rtcm_msg_     = rtcm_msg;
  }

  void GNSSReporter::timerCallback()
  {
    if(msg_received_)
    {
      msg_received_ = false;
      state_transitions::changeStateToOk<r4c_interfaces::msg::SensorStatus>(gnss_status_);
    }

    rclcpp::Time current_time = this->now();

    if(!report_)
    {
      state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(gnss_status_);
    }
    else
    {
      reporter_commons::evaluateConnection<sensor_msgs::msg::NavSatFix,
                                           r4c_interfaces::msg::SensorStatus>(gnss_status_,
                                                                              sensor_msg_,
                                                                              current_time,
                                                                              timeout_);
      reporter_commons::evaluateSynchronization<sensor_msgs::msg::NavSatFix,
                                                r4c_interfaces::msg::SensorStatus>(gnss_status_,
                                                                                   sensor_msg_,
                                                                                   current_time,
                                                                                   sync_threshold_);
      evaluateCorrections(gnss_status_, rtcm_msg_, timeout_);
    }

    gnss_status_.header.stamp = this->now();

    sensor_status_pub_->publish(gnss_status_);
  }

  void GNSSReporter::evaluateFix(r4c_interfaces::msg::SensorStatus& sensor_status,
                                 sensor_msgs::msg::NavSatFix::SharedPtr& sensor_msg)
  {
    if(sensor_status.state != r4c_interfaces::msg::SensorStatus::STATE_ERROR)
    {
      if(sensor_msg->status.status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
      {
        state_transitions::addWarningCode<r4c_interfaces::msg::SensorStatus>(
          r4c_interfaces::msg::SensorStatus::GNSS_NOT_FIX,
          sensor_status);
      }
      else
      {
        state_transitions::removeWarningCode<r4c_interfaces::msg::SensorStatus>(
          r4c_interfaces::msg::SensorStatus::GNSS_NOT_FIX,
          sensor_status);
      }
    }
  }

  void GNSSReporter::evaluateCorrections(r4c_interfaces::msg::SensorStatus& sensor_status,
                                         rtcm_msgs::msg::Message::SharedPtr& rtcm_msg,
                                         double timeout)
  {
    rclcpp::Time current_time  = this->now();
    rclcpp::Duration time_diff = current_time - rtcm_msg->header.stamp;

    double time_difference = time_diff.seconds();

    if(time_difference > timeout)
    {
      state_transitions::addWarningCode<r4c_interfaces::msg::SensorStatus>(
        r4c_interfaces::msg::SensorStatus::NO_CORRECTIONS,
        sensor_status);
    }
    else
    {
      state_transitions::removeWarningCode<r4c_interfaces::msg::SensorStatus>(
        r4c_interfaces::msg::SensorStatus::NO_CORRECTIONS,
        sensor_status);
    }
  }

}  // namespace gnss_reporter
