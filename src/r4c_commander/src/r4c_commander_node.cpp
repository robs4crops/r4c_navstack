#include "rclcpp/rclcpp.hpp"
#include "r4c_commander/r4c_commander.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<r4c_commander::R4CCommander>("r4c_commander"));
    rclcpp::shutdown();
    return 0;
}
