#include <cmath>
#include <functional>
#include <limits>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <robot_localization/srv/set_datum.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <r4c_msgs/msg/agcbox_gnss_metadata.hpp>

class TractorGnssInit: public rclcpp::Node
{
  public:
  TractorGnssInit():
    Node{"r4c_tractor_gnss_init"},
    agcbox_gnss_metadata_sub_{this->create_subscription<r4c_msgs::msg::AgcboxGnssMetadata>(
      this->declare_parameter("agcbox_gnss_metadata_topic", "agcbox/gnss/metadata"),
      1,
      std::bind(&TractorGnssInit::agcbox_gnss_metadata_cb, this, std::placeholders::_1))},
    agcbox_heading_imu_sub_{this->create_subscription<sensor_msgs::msg::Imu>(
      this->declare_parameter("agcbox_heading_topic", "agcbox/heading_imu"),
      1,
      std::bind(&TractorGnssInit::agcbox_heading_imu_cb, this, std::placeholders::_1))},
    gnss_wgs84_fix_sub_{this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("agcbox_gnss_wgs84_fix_topic", "agcbox/gnss/wgs84_fix"),
      1,
      std::bind(&TractorGnssInit::gnss_wgs84_fix_cb, this, std::placeholders::_1))},
    cli_{this->create_client<robot_localization::srv::SetDatum>(
      this->declare_parameter("datum_service", "datum"))},
    datum_srv_request_{std::make_shared<robot_localization::srv::SetDatum::Request>()}
  {
    while(!cli_->wait_for_service(std::chrono::milliseconds{1000}))
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Waiting for service " << cli_->get_service_name()
                                                 << " to be available ...");

      if(!rclcpp::ok())
      {
        RCLCPP_DEBUG(this->get_logger(),
                     "Datum service client interrupted while waiting for service to appear.");
        return;
      }
    }

    datum_srv_request_->geo_pose.position.latitude  = std::numeric_limits<double>::quiet_NaN();
    datum_srv_request_->geo_pose.position.longitude = std::numeric_limits<double>::quiet_NaN();
    datum_srv_request_->geo_pose.position.altitude  = std::numeric_limits<double>::quiet_NaN();
    datum_srv_request_->geo_pose.orientation.w      = std::numeric_limits<double>::quiet_NaN();
    datum_srv_request_->geo_pose.orientation.x      = std::numeric_limits<double>::quiet_NaN();
    datum_srv_request_->geo_pose.orientation.y      = std::numeric_limits<double>::quiet_NaN();
    datum_srv_request_->geo_pose.orientation.z      = std::numeric_limits<double>::quiet_NaN();

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Service " << cli_->get_service_name() << " available.");
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void agcbox_gnss_metadata_cb(
    const r4c_msgs::msg::AgcboxGnssMetadata::ConstSharedPtr agcbox_gnss_metadata_msg)
  {
    if(gnss_method_ != r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER)
    {
      gnss_method_ = agcbox_gnss_metadata_msg->gnss_method;

      if(gnss_method_ == r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER)
      {
        agcbox_gnss_metadata_sub_.reset();
        agcbox_gnss_metadata_sub_ = nullptr;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void agcbox_heading_imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr agcbox_heading_imu_msg)
  {
    if(gnss_method_ == r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER)
    {
      tf2::Quaternion q;
      tf2::fromMsg(agcbox_heading_imu_msg->orientation, q);
      q.normalize();  // Normalize, just in case it is not normalized from source

      double yaw = tf2::getYaw(q);

      // We look for some value different from 0.0, because if we receive 0.0 it probably means the
      // source of data is not sending 'real data'.
      if(std::abs(yaw) > std::numeric_limits<double>::epsilon())
      {
        datum_srv_request_->geo_pose.orientation = tf2::toMsg(q);

        agcbox_heading_imu_sub_.reset();
        agcbox_heading_imu_sub_ = nullptr;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void gnss_wgs84_fix_cb(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_wgs84_fix_msg)
  {
    if(gnss_method_ == r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER)
    {
      if(std::abs(gnss_wgs84_fix_msg->latitude) > std::numeric_limits<double>::epsilon() &&
         std::abs(gnss_wgs84_fix_msg->longitude) > std::numeric_limits<double>::epsilon())
      {
        datum_srv_request_->geo_pose.position.latitude  = gnss_wgs84_fix_msg->latitude;
        datum_srv_request_->geo_pose.position.longitude = gnss_wgs84_fix_msg->longitude;
        datum_srv_request_->geo_pose.position.altitude  = 0.0;

        gnss_wgs84_fix_sub_.reset();
        gnss_wgs84_fix_sub_ = nullptr;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  bool gnss_wgs84_rtk_fix_received() const noexcept
  {
    return (gnss_method_ == r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER &&
            std::isfinite(datum_srv_request_->geo_pose.position.latitude) &&
            std::isfinite(datum_srv_request_->geo_pose.position.longitude) &&
            std::isfinite(datum_srv_request_->geo_pose.position.altitude) &&
            std::isfinite(datum_srv_request_->geo_pose.orientation.w) &&
            std::isfinite(datum_srv_request_->geo_pose.orientation.x) &&
            std::isfinite(datum_srv_request_->geo_pose.orientation.y) &&
            std::isfinite(datum_srv_request_->geo_pose.orientation.z));
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  auto async_set_datum()
  {
    return cli_->async_send_request(datum_srv_request_);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  private:
  uint8_t gnss_method_{r4c_msgs::msg::AgcboxGnssMetadata::NO_GNSS};
  rclcpp::Subscription<r4c_msgs::msg::AgcboxGnssMetadata>::SharedPtr agcbox_gnss_metadata_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr agcbox_heading_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_wgs84_fix_sub_;
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr cli_;
  robot_localization::srv::SetDatum::Request::SharedPtr datum_srv_request_;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto tractor_gnss_init = std::make_shared<TractorGnssInit>();

  RCLCPP_INFO_STREAM(tractor_gnss_init->get_logger(),
                     "Node " << tractor_gnss_init->get_name() << " ready");

  rclcpp::Rate rate{10.0};

  while(rclcpp::ok())
  {
    rclcpp::spin_some(tractor_gnss_init);

    if(tractor_gnss_init->gnss_wgs84_rtk_fix_received())
    {
      auto future = tractor_gnss_init->async_set_datum();

      // Having into account that the response for the SetDatum service is empty, we do not have to
      // wait for the response to be received from the server after the client has sent the request.
      // We just send the request and exit. The following piece of code is optional, and is left
      // here as a reference:

      // if(rclcpp::spin_until_future_complete(tractor_gnss_init,
      //                                       future,
      //                                       std::chrono::milliseconds{5000}) ==
      //    rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   RCLCPP_DEBUG(tractor_gnss_init->get_logger(), "future is complete");

      //   break;
      // }

      break;
    }

    rate.sleep();
  }

  RCLCPP_INFO_STREAM(tractor_gnss_init->get_logger(),
                     "Shutting down node " << tractor_gnss_init->get_name());

  rclcpp::shutdown();
}