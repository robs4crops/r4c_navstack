#include <r4c_navigation/decorators/do_while_in_parcel.hpp>

DoWhileInParcel::DoWhileInParcel(const std::string& name, const BT::NodeConfiguration& config):
  BT::DecoratorNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("do_while_in_parcel_node");
  
  pub_bounds_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("/parcel", 10);

  buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  ls_     = std::make_shared<tf2_ros::TransformListener>(*buffer_);
}

BT::NodeStatus DoWhileInParcel::tick()
{
  // Initialization
  if(status() != BT::NodeStatus::RUNNING)
  {
    // Get inputs
    geometry_msgs::msg::PoseStamped src_node, dst_node;
    double row_width;

    // Entry/exit points of a row.
    if(!getInput<geometry_msgs::msg::PoseStamped>("src_node", src_node))
    {
      RCLCPP_ERROR(node_->get_logger(), "[DoWhileInParcel] Missing required input [src_node]");
      return BT::NodeStatus::FAILURE;
    }
    if(!getInput<geometry_msgs::msg::PoseStamped>("dst_node", dst_node))
    {
      RCLCPP_ERROR(node_->get_logger(), "[DoWhileInParcel] Missing required input [dst_node]");
      return BT::NodeStatus::FAILURE;
    }

    // Row width.
    if(!getInput<double>("row_width", row_width))
    {
      RCLCPP_ERROR(node_->get_logger(), "[DoWhileInParcel] Missing required input [row_width]");
      return BT::NodeStatus::FAILURE;
    }

    // Whether to invert operation (i.e., do while OUTSIDE parcel)
    if(!getInput<bool>("invert_logic", invert_logic_))
    {
      RCLCPP_ERROR(node_->get_logger(), "[DoWhileInParcel] Missing required input [invert_logic]");
      return BT::NodeStatus::FAILURE;
    }


    // Get position from nodes
    const geometry_msgs::msg::Point src_pos = src_node.pose.position;
    const geometry_msgs::msg::Point dst_pos = dst_node.pose.position;

    // Calculate parcel vertices from given src/dst nodes and row width
    const double theta = atan2(dst_pos.y - src_pos.y, dst_pos.x - src_pos.x);
    const double delta = row_width / 2.0;

    coords_.clear();
    coords_.emplace_back(Eigen::Vector2d(src_pos.x  + delta*cos(theta + M_PI_2), src_pos.y + delta*sin(theta + M_PI_2)));
    coords_.emplace_back(Eigen::Vector2d(dst_pos.x  + delta*cos(theta + M_PI_2), dst_pos.y + delta*sin(theta + M_PI_2)));
    coords_.emplace_back(Eigen::Vector2d(dst_pos.x  + delta*cos(theta - M_PI_2), dst_pos.y + delta*sin(theta - M_PI_2)));
    coords_.emplace_back(Eigen::Vector2d(src_pos.x  + delta*cos(theta - M_PI_2), src_pos.y + delta*sin(theta - M_PI_2)));

    if(invert_logic_)
    {
      RCLCPP_INFO(node_->get_logger(), "[DoWhileInParcel] Running actions while outside parcel: [(%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)]", 
        coords_[0].x(), coords_[0].y(), coords_[1].x(), coords_[1].y(), coords_[2].x(), coords_[2].y(), coords_[3].x(), coords_[3].y());
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "[DoWhileInParcel] Running actions while inside parcel: [(%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)]", 
        coords_[0].x(), coords_[0].y(), coords_[1].x(), coords_[1].y(), coords_[2].x(), coords_[2].y(), coords_[3].x(), coords_[3].y());
    }

    // Publish stamped polygon for visualization
    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = node_->now();
    polygon_msg.header.frame_id = global_frame_;
    for(const auto& v : coords_)
    {
      geometry_msgs::msg::Point32 p;
      p.x = v.x();
      p.y = v.y();
      polygon_msg.polygon.points.push_back(p);
    }
    pub_bounds_->publish(polygon_msg);
  }

  // Run child node
  if(child_node_->executeTick() == BT::NodeStatus::FAILURE)
  {
    RCLCPP_WARN(node_->get_logger(), "[DoWhileInParcel] Child action failed!");
    resetChild();
    return BT::NodeStatus::FAILURE;
  }

  // Get robot position
  if(!buffer_->canTransform(global_frame_, base_frame_, rclcpp::Time(0), rclcpp::Duration(1, 0)))
  {
    RCLCPP_WARN(node_->get_logger(), "[DoWhileInParcel] Cannot look up transform! Aborting actions.");
    resetChild();
    return BT::NodeStatus::FAILURE;
  }
  const auto  base_tf = buffer_->lookupTransform(global_frame_, base_frame_, rclcpp::Time(0));
  const double base_x = base_tf.transform.translation.x;
  const double base_y = base_tf.transform.translation.y;

  // Check if looked up transform position is inside polygon bounds defined by vertices
  // Using PNPoly algorithm: https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
  bool inside{false};

  for(size_t i = 0, j = coords_.size() - 1; i < coords_.size(); j = i++)
  {
    if(((coords_[i].y() > base_y) != (coords_[j].y() > base_y)) &&
       (base_x < (coords_[j].x() - coords_[i].x()) * (base_y - coords_[i].y()) / (coords_[j].y() - coords_[i].y()) + coords_[i].x()))
    {
      inside = !inside;
    }
  }

  // If robot is outside parcel, halt and return success
  if(!invert_logic_ && !inside)
  {
    RCLCPP_INFO(node_->get_logger(), "[DoWhileInParcel] Robot is outside parcel. Halting child");
    resetChild();
    return BT::NodeStatus::SUCCESS;
  }
  else if(invert_logic_ && inside) // If robot is inside parcel (for the inverted case), halt and return success
  {
    RCLCPP_INFO(node_->get_logger(), "[DoWhileInParcel] Robot is inside parcel. Halting child");
    resetChild();
    return BT::NodeStatus::SUCCESS;
  }

  // Otherwise continue
  return BT::NodeStatus::RUNNING;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<DoWhileInParcel>("DoWhileInParcel");
}
