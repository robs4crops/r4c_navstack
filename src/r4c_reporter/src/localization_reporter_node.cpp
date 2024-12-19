#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/localization_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<localization_reporter::LocalizationReporter>("localization_reporter"));
    rclcpp::shutdown();
    return 0;
}
