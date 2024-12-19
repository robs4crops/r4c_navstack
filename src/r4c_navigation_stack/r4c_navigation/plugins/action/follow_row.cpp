#include <r4c_navigation/action/follow_row.hpp>

FollowRow::FollowRow(const std::string& name, const BT::NodeConfiguration& config):
  BT::StatefulActionNode(name, config)
{
  // Get ROS parameters
  node_ = std::make_shared<rclcpp::Node>("follow_row");
  node_->declare_parameter("global_frame", global_frame_);
  node_->declare_parameter("robot_frame", robot_frame_);
  node_->declare_parameter("path_navigation_action_name", path_navigation_action_name_);
  node_->declare_parameter("pure_pursuit_local_planner", pure_pursuit_local_planner_);
  node_->declare_parameter("reactive_navigation_topic", reactive_navigation_topic_);
  node_->declare_parameter("reactive_navigation_max_vel_lin", reactive_navigation_max_vel_lin_);
  node_->declare_parameter("reactive_navigation_max_vel_ang", reactive_navigation_max_vel_ang_);
  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_frame", robot_frame_);
  node_->get_parameter("path_navigation_action_name", path_navigation_action_name_);
  node_->get_parameter("pure_pursuit_local_planner", pure_pursuit_local_planner_);
  node_->get_parameter("reactive_navigation_topic", reactive_navigation_topic_);
  node_->get_parameter("reactive_navigation_max_vel_lin", reactive_navigation_max_vel_lin_);
  node_->get_parameter("reactive_navigation_max_vel_ang", reactive_navigation_max_vel_ang_);

  buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  ls_     = std::make_shared<tf2_ros::TransformListener>(buffer_.get());
}

BT::NodeStatus FollowRow::onStart()
{
  if(!getInput<geometry_msgs::msg::Pose2D>("src_node", src_node_))
  {
    throw BT::RuntimeError("Missing required input [src_node]");
  }

  if(!getInput<geometry_msgs::msg::Pose2D>("dst_node", dst_node_))
  {
    throw BT::RuntimeError("Missing required input [dst_node]");
  }

  // Configure goal
//   goal_.header.frame_id       = robot_frame_;
//   goal_.header.stamp          = ros::Time::now();
//   goal_.global_plan_topic     = reactive_navigation_topic_;
//   goal_.local_planner         = pure_pursuit_local_planner_;
//   goal_.path_type             = gntc_navigation::PathNavigationGoal::PATH_TYPE_ROW;
//   goal_.path_dir              = dir_; // -1: backward | +1: forward
//   goal_.nav_type              = gntc_navigation::PathNavigationGoal::NAV_TYPE_REACTIVE;
//   goal_.global_frame          = robot_frame_;
//   goal_.robot_base_frame      = robot_frame_;
//   goal_.max_vel_lin           = reactive_navigation_max_vel_lin_;
//   goal_.max_vel_th            = reactive_navigation_max_vel_ang_;
//   goal_.check_goal_reached    = true;
//   goal_.tolerance_from_action = false;
//   goal_.lookahead_distance    = 1.2;

  end_reached_ = false;
  send_goal_   = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowRow::onRunning()
{
  if(send_goal_)
  {
    RCLCPP_INFO(node_->get_logger(), "[FollowRow] Sending goal.");
    // ac_->sendGoal(goal_);
    send_goal_ = false;
  }

  // Check whether action is running or robot is inside line
  if(!end_reached_)
  {
    // if(ac_->getState().isDone() && ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //   RCLCPP_ERROR(node_->get_logger(), "[FollowRow] Row following action failed!");
    //   return BT::NodeStatus::FAILURE;
    // }

    // Get robot pose
    // TODO: action must fail after a while if TF is not found. Must review this
    if(!buffer_->canTransform(global_frame_, robot_frame_, rclcpp::Time(0), rclcpp::Duration(1, 0)))
    {
      RCLCPP_WARN(node_->get_logger(), "[FollowRow] Cannot look up transform! Continuing action.");
      return BT::NodeStatus::RUNNING;
    }
    auto robot_pose = buffer_->lookupTransform(global_frame_, robot_frame_, rclcpp::Time(0));

    // If "end of line" is reached (pose within a threshold), finish action.
    // From the end node coordinates, derive a virtual line through which the
    // robot must pass for the action to be concluded.
    // Note: assuming end node orientation is outwards.
    // TODO: the orientation can be implicitly calculated, analyze all cases!
    const Eigen::Vector2d u(dst_node_.x   - src_node_.x, dst_node_.y   - src_node_.y);
    const Eigen::Vector2d v(robot_pose.transform.translation.x - dst_node_.x, 
                            robot_pose.transform.translation.y - dst_node_.y);
    
    if(v.norm() > 1e-6)
    {
      const double angle = acos(u.dot(v) / (u.norm() * v.norm()));
      end_reached_ = std::fabs(angle) < M_PI_2;
    }

    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "[FollowRow] End of row reached.");
//   ac_->cancelGoal();
  return BT::NodeStatus::SUCCESS;
}

// Interrupt action if halt is requested.
void FollowRow::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[FollowRow] Halt");
//   ac_->cancelGoal();
  send_goal_ = true;
}

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<FollowRow>("FollowRow");
}
