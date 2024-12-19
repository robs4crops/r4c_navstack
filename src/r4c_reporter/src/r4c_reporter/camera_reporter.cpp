#include "r4c_reporter/camera_reporter.hpp"

namespace camera_reporter
{
  CameraReporter::CameraReporter(const std::string& node_name):
    rclcpp::Node(node_name),
    report_{this->declare_parameter("report", false)},
    sensor_name_{this->declare_parameter("sensor_name", "camera")},
    sync_threshold_{this->declare_parameter("sync_threshold", 100.0)},
    timeout_{this->declare_parameter("timeout", 5.0)},
    check_frequency_{std::abs(this->declare_parameter("check_frequency", 1.0))},
    occlusion_thresh_{std::abs(this->declare_parameter("occlusion_thresh", 0.1))},
    sensor_msg_sub_{this->create_subscription<sensor_msgs::msg::Image>(
      this->declare_parameter("sensor_topic", "/stereo/depth"),
      10,
      std::bind(&CameraReporter::sensor_msg_callback, this, std::placeholders::_1))},
    sensor_status_pub_{this->create_publisher<r4c_interfaces::msg::SensorStatus>(
      this->declare_parameter("status_topic", "/r4c_tractor/camera/status"),
      10)}
  {
    camera_status_.name       = sensor_name_;
    sensor_msg_               = std::make_shared<sensor_msgs::msg::Image>();
    sensor_msg_->header.stamp = this->now();
    state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(camera_status_);
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(check_frequency_ * 1000)),
                               std::bind(&CameraReporter::timerCallback, this));
  }

  void CameraReporter::sensor_msg_callback(const sensor_msgs::msg::Image::SharedPtr sensor_msg)
  {
    msg_received_ = true;
    sensor_msg_   = sensor_msg;
  }

  void CameraReporter::timerCallback()
  {
    if(msg_received_)
    {
      msg_received_ = false;
      state_transitions::changeStateToOk<r4c_interfaces::msg::SensorStatus>(camera_status_);
    }

    rclcpp::Time current_time = this->now();

    if(!report_)
    {
      state_transitions::changeStateToInactive<r4c_interfaces::msg::SensorStatus>(camera_status_);
    }
    else
    {
      reporter_commons::evaluateConnection<sensor_msgs::msg::Image,
                                           r4c_interfaces::msg::SensorStatus>(camera_status_,
                                                                              sensor_msg_,
                                                                              current_time,
                                                                              timeout_);
      reporter_commons::evaluateSynchronization<sensor_msgs::msg::Image,
                                                r4c_interfaces::msg::SensorStatus>(camera_status_,
                                                                                   sensor_msg_,
                                                                                   current_time,
                                                                                   sync_threshold_);
      evaluateOcclusion(camera_status_, sensor_msg_);
    }

    camera_status_.header.stamp = this->now();

    sensor_status_pub_->publish(camera_status_);
  }

  void CameraReporter::evaluateOcclusion(r4c_interfaces::msg::SensorStatus& sensor_status,
                                         sensor_msgs::msg::Image::SharedPtr& sensor_msg)
  {
    if(sensor_status.state != r4c_interfaces::msg::SensorStatus::STATE_ERROR)
    {
      // Check is camera is occluded
      unsigned int total_pixels = sensor_msg->data.size();
      unsigned int valid_pixels = 0;

      for(unsigned int i = 0; i < total_pixels; i++)
      {
        if(sensor_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
           sensor_msg->encoding == sensor_msgs::image_encodings::MONO16)
        {
          if(sensor_msg->data[i] != 0)
          {
            valid_pixels++;
          }
          else if(sensor_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
          {
            // Needs to be tested
            if(std::isfinite(sensor_msg->data[i]))
            {
              valid_pixels++;
            }
          }
        }
      }

      // Check valid percentage
      double percentage = double(valid_pixels) / double(total_pixels);
      if(percentage <= occlusion_thresh_)
      {
        state_transitions::addErrorCode<r4c_interfaces::msg::SensorStatus>(
          r4c_interfaces::msg::SensorStatus::OBSTRUCTED,
          sensor_status);
      }
      else
      {
        state_transitions::removeErrorCode<r4c_interfaces::msg::SensorStatus>(
          r4c_interfaces::msg::SensorStatus::OBSTRUCTED,
          sensor_status);
      }
    }
  }
}  // namespace camera_reporter
