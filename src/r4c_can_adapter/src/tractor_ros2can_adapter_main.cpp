#include "r4c_can_adapter/tractor_ros2can_adapter.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<r4c_can_adapter::TractorRos2CanAdapter>());
  }
  catch(std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tractor_ros2can_adapter_main_error"), "Error: %s.", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
