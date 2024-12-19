#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace r4c::localization
{

class PubDatumFixNode: public rclcpp::Node
{
    public:
    PubDatumFixNode() : rclcpp::Node("pub_datum_fix")
    {
        // Declare/get datum parameters. Defaults to Serrater base station (est.)
        // NOTE: height is not considered at the moment.
        this->declare_parameter<std::vector<double>>("datum", datum_);
        this->get_parameter("datum", datum_);

        // Setup publisher
        pub_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/datum/fix", 10);

        // Setup timer with a frequency of 1.0 Hz.
        timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration(1.0, 0.0), 
            std::bind(&PubDatumFixNode::publishFix, this)); 
    }

    // Timer event that periodically publishes a fixum based on datum info.
    void publishFix()
    {
        // Create fix message
        sensor_msgs::msg::NavSatFix fix_msg;
        fix_msg.header.frame_id = "map";
        fix_msg.header.stamp = this->now();
        fix_msg.latitude  = datum_[0];
        fix_msg.longitude = datum_[1];

        // Publish message
        pub_fix_->publish(fix_msg);
    }

    private:
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
    std::vector<double> datum_{42.1631285, 3.0872069};
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace r4c::localization

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r4c::localization::PubDatumFixNode>());
  rclcpp::shutdown();

  return 0;
}