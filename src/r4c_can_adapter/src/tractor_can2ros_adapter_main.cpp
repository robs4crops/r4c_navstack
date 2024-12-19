#include "r4c_can_adapter/tractor_can2ros_adapter.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<r4c_can_adapter::TractorCan2RosAdapter>());
  }
  catch(std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tractor_can2ros_adapter_main_error"), "Error: %s.", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
