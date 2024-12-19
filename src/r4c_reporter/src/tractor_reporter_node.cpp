#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/tractor_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tractor_reporter::TractorReporter>("tractor_reporter"));
    rclcpp::shutdown();
    return 0;
}
