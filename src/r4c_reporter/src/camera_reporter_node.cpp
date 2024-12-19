#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/camera_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_reporter::CameraReporter>("camera_reporter"));
    rclcpp::shutdown();
    return 0;
}
