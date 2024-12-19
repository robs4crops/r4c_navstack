#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/can_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<can_reporter::CanReporter>("can_reporter"));
    rclcpp::shutdown();
    return 0;
}
