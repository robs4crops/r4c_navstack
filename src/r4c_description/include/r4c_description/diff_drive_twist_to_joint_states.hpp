#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace diff_drive_twist_to_joint_states
{
  class DiffDriveTwistToJointStates: public rclcpp::Node
  {
    public:
    DiffDriveTwistToJointStates(const std::string& node_name,
                                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
    void twist_cb(const geometry_msgs::msg::Twist::ConstSharedPtr twist_msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    std::string left_wheel_joint_name_;
    std::string right_wheel_joint_name_;
    double wheel_radius_;
    double wheel_spacing_;

    rclcpp::Time prev_twist_time_;

    double left_wheel_pos_{0.0};
    double right_wheel_pos_{0.0};
  };

}  // namespace diff_drive_twist_to_joint_states
