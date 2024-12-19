#include "r4c_can_adapter/agcbox_ens_emulator.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<r4c_can_adapter::AgcboxEnsEmulator>());
  }
  catch(std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("agcbox_ens_emulator_main_error"), "Error: %s.", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
