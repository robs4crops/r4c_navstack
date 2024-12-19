#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/imu_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_reporter::ImuReporter>("imu_reporter"));
    rclcpp::shutdown();
    return 0;
}
