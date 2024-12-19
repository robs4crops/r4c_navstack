#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/gnss_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gnss_reporter::GNSSReporter>("gnss_reporter"));
    rclcpp::shutdown();
    return 0;
}
