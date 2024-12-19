#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace diff_drive_joint_states_to_twist
{
  class DiffDriveJointStatesToTwist: public rclcpp::Node
  {
    public:
    DiffDriveJointStatesToTwist(const std::string& node_name,
                                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);

    private:
    int get_joint_index(const std::vector<std::string>& joint_names,
                        const std::string& joint_name) noexcept;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    std::string left_wheel_joint_name_;
    std::string right_wheel_joint_name_;
    double wheel_radius_;
    double wheel_spacing_;
  };

}  // namespace diff_drive_joint_states_to_twist
