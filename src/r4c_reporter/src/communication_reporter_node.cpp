#include "rclcpp/rclcpp.hpp"
#include "r4c_reporter/communication_reporter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<communication_reporter::CommunicationReporter>("communication_reporter"));
    rclcpp::shutdown();
    return 0;
}
