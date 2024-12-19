#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/internet_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<internet_reporter::InternetReporter>("internet_reporter"));
    rclcpp::shutdown();
    return 0;
}
