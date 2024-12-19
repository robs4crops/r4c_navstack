#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/lidar_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_reporter::LidarReporter>("lidar_reporter"));
    rclcpp::shutdown();
    return 0;
}
