#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/sensors_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors_reporter::SensorsReporter>("sensors_reporter"));
    rclcpp::shutdown();
    return 0;
}
