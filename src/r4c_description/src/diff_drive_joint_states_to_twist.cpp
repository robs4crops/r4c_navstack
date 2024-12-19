#include <cmath>

#include "r4c_description/diff_drive_joint_states_to_twist.hpp"

namespace diff_drive_joint_states_to_twist
{
  DiffDriveJointStatesToTwist::DiffDriveJointStatesToTwist(const std::string& node_name,
                                                           const rclcpp::NodeOptions& options):
    rclcpp::Node{node_name, options},
    joint_state_sub_{this->create_subscription<sensor_msgs::msg::JointState>(
      this->declare_parameter("joint_states_topic", "joint_states"),
      10,
      std::bind(&DiffDriveJointStatesToTwist::joint_state_cb, this, std::placeholders::_1))},
    twist_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
      this->declare_parameter("twist_topic", "r4c_tractor/vel"),
      10)},
    left_wheel_joint_name_{this->declare_parameter("left_wheel_joint_name", "left_axle")},
    right_wheel_joint_name_{this->declare_parameter("right_wheel_joint_name", "right_axle")},
    wheel_radius_{std::abs(this->declare_parameter("wheel_radius", 0.65))},
    wheel_spacing_{std::abs(this->declare_parameter("wheel_spacing", 2.1804))}
  {
    RCLCPP_INFO(this->get_logger(), "Velocities to twist node initialized successfully");
  }

  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void DiffDriveJointStatesToTwist::joint_state_cb(
    const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)
  {
    auto left_wheel_joint_idx  = get_joint_index(joint_state->name, left_wheel_joint_name_);
    auto right_wheel_joint_idx = get_joint_index(joint_state->name, right_wheel_joint_name_);

    if(left_wheel_joint_idx == -1 || right_wheel_joint_idx == -1)
    {
      return;
    }

    double left_wheel_joint_vel  = joint_state->velocity[left_wheel_joint_idx];
    double right_wheel_joint_vel = joint_state->velocity[right_wheel_joint_idx];

    // https://www.cs.columbia.edu/~allen/F19/NOTES/icckinematics.pdf

    // v_left = w * ( R - wheel_spacing/2)
    // v_right = w * ( R + wheel_spacing/2)
    // v = w * R

    // v_left + v_right = 2wR = 2*v => v = (v_left + v_right)/2
    // v_right - v_left = w * wheel_spacing => w = (v_right - v_left)/wheel_spacing
    // R = (v_left + v_right)/(2w) = v/w

    geometry_msgs::msg::Twist twist_msg;

    double v_left  = left_wheel_joint_vel * wheel_radius_;
    double v_right = right_wheel_joint_vel * wheel_radius_;

    twist_msg.linear.x  = (v_left + v_right) * 0.5;
    twist_msg.angular.z = (v_right - v_left) / wheel_spacing_;
    // auto R = v / w;

    twist_pub_->publish(twist_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  int DiffDriveJointStatesToTwist::get_joint_index(const std::vector<std::string>& joint_names,
                                                   const std::string& joint_name) noexcept
  {
    auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);

    if(it != joint_names.end())
    {
      return static_cast<int>(it - joint_names.begin());
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Could not find joint %s in joint_states topic",
                 joint_name.c_str());

    return -1;
  }

}  // namespace diff_drive_joint_states_to_twist
