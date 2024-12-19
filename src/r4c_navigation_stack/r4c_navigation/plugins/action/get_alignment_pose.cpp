#include <r4c_navigation/action/get_alignment_pose.hpp>

GetAlignmentPose::GetAlignmentPose(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("get_alignment_pose");

  buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  ls_     = std::make_shared<tf2_ros::TransformListener>(*buffer_);

  alignment_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("alignment_pose", 1);
}

BT::NodeStatus GetAlignmentPose::tick()
{
  geometry_msgs::msg::PoseStamped parcel_origin;
  if(!getInput("parcel_origin", parcel_origin))
  {
    throw BT::RuntimeError("Missing required input [parcel_origin]");
  }

  geometry_msgs::msg::PoseStamped parcel_end;
  if(!getInput("parcel_end", parcel_end))
  {
    throw BT::RuntimeError("Missing required input [parcel_end]");
  }

  double vert_alignment_dist;
  if(!getInput("vert_alignment_dist", vert_alignment_dist))
  {
    throw BT::RuntimeError("Missing required input [vert_alignment_dist]");
  }

  double lat_alignment_dist;
  if(!getInput("lat_alignment_dist", lat_alignment_dist))
  {
    throw BT::RuntimeError("Missing required input [lat_alignment_dist]");
  }

  double alignment_orientation;
  if(!getInput("alignment_orientation", alignment_orientation))
  {
    throw BT::RuntimeError("Missing required input [alignment_orientation]");
  }

  if(!buffer_->canTransform("map", "base_link", rclcpp::Time(0), rclcpp::Duration(1, 0)))
  {
    RCLCPP_WARN(node_->get_logger(), "[GetAlignmentPose] Cannot look up transform!");
    return BT::NodeStatus::FAILURE;
  }
  auto base_tf = buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));

  //Determine if approach if from the right or the left of the vector of line
  double AB_x = parcel_end.pose.position.x - parcel_origin.pose.position.x;
  double AB_y = parcel_end.pose.position.y - parcel_origin.pose.position.y;
  double AR_x = base_tf.transform.translation.x - parcel_origin.pose.position.x;
  double AR_y = base_tf.transform.translation.y - parcel_origin.pose.position.y;

  RCLCPP_WARN_STREAM(node_->get_logger(),"Robot is at "<<base_tf.transform.translation.x<<","<<base_tf.transform.translation.y);
  
  // Calculate cross product of AB and AR
  double cross_product = AB_x * AR_y - AB_y * AR_x;

  bool direction;
  // Determine position based on cross product
  if (cross_product > 0) {
      direction = 0; // R is to the left of AB
      RCLCPP_WARN(node_->get_logger(), "Coming from left");
  } 
  else if (cross_product < 0) {
      direction = 1; // R is to the right of AB
      RCLCPP_WARN(node_->get_logger(), "Coming from right");
  }

  double vector_x = parcel_end.pose.position.x - parcel_origin.pose.position.x;
  double vector_y = parcel_end.pose.position.y - parcel_origin.pose.position.y; 

  double magn = sqrt(pow(vector_x, 2) + pow(vector_y, 2));

  double norm_vector_x = vector_x/magn;
  double norm_vector_y = vector_y/magn;

  double perp_vector_x, perp_vector_y;

  if (!direction) 
  {
    // Extend laterally to the right
    perp_vector_x = -norm_vector_y;
    perp_vector_y = norm_vector_x;
  } else 
  {
    // Extend laterally to the left
    perp_vector_x = norm_vector_y;
    perp_vector_y = -norm_vector_x;
  }

  geometry_msgs::msg::PoseStamped alignment_pose;
  alignment_pose = parcel_origin;
  alignment_pose.pose.position.x = parcel_origin.pose.position.x - norm_vector_x * vert_alignment_dist + perp_vector_x * lat_alignment_dist;
  alignment_pose.pose.position.y = parcel_origin.pose.position.y - norm_vector_y * vert_alignment_dist + perp_vector_y * lat_alignment_dist;

  double dx = parcel_end.pose.position.x - parcel_origin.pose.position.x;
  double dy = parcel_end.pose.position.y - parcel_origin.pose.position.y;
  double orientation = atan2(dy,dx); //* 180.0 / 3.1415926;

  RCLCPP_WARN_STREAM(node_->get_logger(),"Original orientation was "<<orientation);

  // Apply orientation offset depending on side coming from
  if(!direction)
  {
    orientation -= alignment_orientation * (3.1415926/180.0);
  }
  else
  {
    orientation += alignment_orientation * (3.1415926/180.0);
  }

  RCLCPP_WARN_STREAM(node_->get_logger(),"Now it is "<<orientation);

  tf2::Quaternion quat;
  quat.setRPY(0, 0, orientation); // Roll and pitch are zero, only yaw is set

  // Set orientation quaternion
  alignment_pose.pose.orientation.x = quat.x();
  alignment_pose.pose.orientation.y = quat.y();
  alignment_pose.pose.orientation.z = quat.z();
  alignment_pose.pose.orientation.w = quat.w();

  if(!setOutput<geometry_msgs::msg::PoseStamped>("alignment_pose", alignment_pose))
  {
    throw BT::RuntimeError("Could not set output [alignment_pose]");
  }

  alignment_pose_publisher_->publish(alignment_pose);

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetAlignmentPose>("GetAlignmentPose");
}