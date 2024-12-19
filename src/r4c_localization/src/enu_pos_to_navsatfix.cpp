#include <GeographicLib/LocalCartesian.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <r4c_msgs/msg/gnss_enu_vector.hpp>
namespace r4c::localization
{

  class EnuPostoNavSatFix: public rclcpp::Node
  {
    public:
    EnuPostoNavSatFix(): rclcpp::Node("enu_to_navsatfix")
    {
      // Declare/get datum parameters. Defaults to Serrater base station (est.)
      // NOTE: height is not considered at the moment.
      this->declare_parameter<std::vector<double>>("datum", datum_);
      this->get_parameter("datum", datum_);

      // Setup the converter
      converter_.Reset(datum_[0], datum_[1]);

      // Setup publisher and subscriber
      gnss_enu_pos_sub_ = this->create_subscription<r4c_msgs::msg::GnssEnuVector>(
        "/input_enu_pos",
        10,
        std::bind(&EnuPostoNavSatFix::rcv_gnss_enu_pos, this, std::placeholders::_1));

      gnss_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/output_fix", 10);
    }

    // Converts ENU messages to NavSatFix given the datum of the base station.
    // GeographicLib "LocalCartesian" object is used to convert from geodetic
    // to cartesian coordinates and vice-versa.
    void rcv_gnss_enu_pos(r4c_msgs::msg::GnssEnuVector::SharedPtr gnss_enu_pos_msg)
    {
      // Create fix message and update header
      // NOTE: other information (covariance, status, etc.) not included.
      sensor_msgs::msg::NavSatFix gnss_fix_msg;
      gnss_fix_msg.header = gnss_enu_pos_msg->header;

      // Perform actual conversion
      converter_.Reverse(gnss_enu_pos_msg->e,
                         gnss_enu_pos_msg->n,
                         gnss_enu_pos_msg->u,
                         gnss_fix_msg.latitude,
                         gnss_fix_msg.longitude,
                         gnss_fix_msg.altitude);

      // Publish message
      gnss_fix_pub_->publish(gnss_fix_msg);
    }

    private:
    rclcpp::Subscription<r4c_msgs::msg::GnssEnuVector>::SharedPtr gnss_enu_pos_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_fix_pub_;
    std::vector<double> datum_{42.1631285, 3.0872069};
    GeographicLib::LocalCartesian converter_;
  };

}  // namespace r4c::localization

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r4c::localization::EnuPostoNavSatFix>());
  rclcpp::shutdown();

  return 0;
}