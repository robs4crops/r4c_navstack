#include "r4c_description/diff_drive_joint_states_to_twist.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<diff_drive_joint_states_to_twist::DiffDriveJointStatesToTwist>(
    "diff_drive_joint_states_to_twist"));
  rclcpp::shutdown();
  return 0;
}
