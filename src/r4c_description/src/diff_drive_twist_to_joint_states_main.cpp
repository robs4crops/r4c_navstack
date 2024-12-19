#include "r4c_description/diff_drive_twist_to_joint_states.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<diff_drive_twist_to_joint_states::DiffDriveTwistToJointStates>(
    "diff_drive_twist_to_joint_states"));
  rclcpp::shutdown();
  return 0;
}
