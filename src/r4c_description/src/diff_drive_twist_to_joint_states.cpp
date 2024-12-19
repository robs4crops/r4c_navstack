#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <rclcpp/wait_for_message.hpp>
#include <angles/angles.h>

#include "r4c_description/diff_drive_twist_to_joint_states.hpp"

namespace diff_drive_twist_to_joint_states
{
  DiffDriveTwistToJointStates::DiffDriveTwistToJointStates(const std::string& node_name,
                                                           const rclcpp::NodeOptions& options):
    rclcpp::Node{node_name, options},
    twist_sub_{this->create_subscription<geometry_msgs::msg::Twist>(
      this->declare_parameter("twist_topic", "twist"),
      10,
      std::bind(&DiffDriveTwistToJointStates::twist_cb, this, std::placeholders::_1))},
    joint_state_pub_{this->create_publisher<sensor_msgs::msg::JointState>(
      this->declare_parameter("joint_states_topic", "joint_states"),
      10)},
    left_wheel_joint_name_{this->declare_parameter("left_wheel_joint_name", "left_axle")},
    right_wheel_joint_name_{this->declare_parameter("right_wheel_joint_name", "right_axle")},
    wheel_radius_{this->declare_parameter("wheel_radius", 0.65)},
    wheel_spacing_{this->declare_parameter("wheel_spacing", 1.176)}
  {
    // Wait for the first twist message to arrive.
    // The first twist message is the origin for the odometry computation.
    uint64_t timeout_ms = static_cast<uint64_t>(
      std::round(1000.0 * std::abs(this->declare_parameter("first_twist_timeout", 5.0))));

    auto twist = std::make_shared<geometry_msgs::msg::Twist>();

    if(!rclcpp::wait_for_message(*twist,
                                 twist_sub_,
                                 this->get_node_options().context(),
                                 std::chrono::duration<uint64_t, std::milli>{timeout_ms}))
    {
      throw std::runtime_error{"No first twist message received"};
    }

    prev_twist_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Twist to joint_states node initialized successfully");
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void DiffDriveTwistToJointStates::twist_cb(
    const geometry_msgs::msg::Twist::ConstSharedPtr twist_msg)
  {
    auto now = this->now();
    auto dt  = (now - prev_twist_time_).seconds();

    // By default we consider the vehicle moving in a straight line.
    double v_left  = twist_msg->linear.x;
    double v_right = twist_msg->linear.x;

    // If the angular_vel is different from zero, vels for each wheel are different.
    if(std::abs(twist_msg->angular.z) > std::numeric_limits<double>::epsilon())
    {
      auto R                  = twist_msg->linear.x / twist_msg->angular.z;
      auto half_wheel_spacing = 0.5 * wheel_spacing_;
      auto R_left             = R - half_wheel_spacing;
      auto R_right            = R + half_wheel_spacing;
      v_left                  = twist_msg->angular.z * R_left;
      v_right                 = twist_msg->angular.z * R_right;
    }

    double w_left  = v_left / wheel_radius_;
    double w_right = v_right / wheel_radius_;

    left_wheel_pos_ += (w_left * dt);
    right_wheel_pos_ += (w_right * dt);

    left_wheel_pos_  = angles::normalize_angle_positive(left_wheel_pos_);
    right_wheel_pos_ = angles::normalize_angle_positive(right_wheel_pos_);

    sensor_msgs::msg::JointState joint_state_msg;

    joint_state_msg.header.stamp = this->now();

    joint_state_msg.name.push_back(left_wheel_joint_name_);
    joint_state_msg.position.push_back(left_wheel_pos_);
    joint_state_msg.velocity.push_back(w_left);

    joint_state_msg.name.push_back(right_wheel_joint_name_);
    joint_state_msg.position.push_back(right_wheel_pos_);
    joint_state_msg.velocity.push_back(w_right);

    // Leaving effort field empty as it is not necessary.

    joint_state_pub_->publish(joint_state_msg);
  }
}  // namespace diff_drive_twist_to_joint_states
