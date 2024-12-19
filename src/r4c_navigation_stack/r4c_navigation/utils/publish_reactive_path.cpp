#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <eut_nav_utils/nav_utils.h>

class PathPublisher: public rclcpp::Node
{
  public:
  PathPublisher(const nav_msgs::msg::Path& path): Node("publish_reactive_path"), initial_path_(path)
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers
    initial_path_publisher_  = this->create_publisher<nav_msgs::msg::Path>("initial_path", 10);
    reactive_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("central_line", 10);

    // Define frames
    global_frame_     = initial_path_.header.frame_id;
    robot_base_frame_ = "base_footprint";

    double transform_tolerance = 0.5;
    transform_tolerance_       = tf2::durationFromSec(transform_tolerance);

    double rate         = 20.0;        // Hz
    double timer_period = 1.0 / rate;  // seconds
    timer_              = this->create_wall_timer(std::chrono::duration<double>(timer_period),
                                     std::bind(&PathPublisher::timer_callback, this));
  }

  std::vector<geometry_msgs::msg::PoseStamped>::iterator findClosestWaypoint(
    std::vector<geometry_msgs::msg::PoseStamped>& plan,
    const geometry_msgs::msg::PoseStamped& pose)
  {
    double min_dist{1e5};
    std::vector<geometry_msgs::msg::PoseStamped>::iterator it, min_dist_waypoint_it;
    for(it = plan.begin(); it < plan.end(); it++)
    {
      double robot_to_waypoint_dist = nav_utils::euclideanDistance(pose, *it);
      if(robot_to_waypoint_dist < min_dist)
      {
        min_dist_waypoint_it = it;
        min_dist             = robot_to_waypoint_dist;
      }
    }
    return min_dist_waypoint_it;
  }

  private:
  void timer_callback()
  {
    // Update initial path stamp header
    initial_path_.header.stamp = get_clock()->now();

    // Transform path
    nav_msgs::msg::Path transformed_path;
    if(!nav_utils::transformGlobalPlan(tf_buffer_,
                                       initial_path_,
                                       robot_base_frame_,
                                       transform_tolerance_,
                                       transformed_path))
    {
      RCLCPP_WARN(this->get_logger(),
                  "Could not transform %s to %s",
                  global_frame_.c_str(),
                  robot_base_frame_.c_str());
      return;
    }

    // Find closest waypoint to the robot
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.pose.position.x    = 0;
    robot_pose.pose.position.y    = 0;
    robot_pose.pose.orientation.w = 1;
    auto it_robot                 = findClosestWaypoint(transformed_path.poses, robot_pose);

    nav_msgs::msg::Path reactive_path;
    reactive_path.header = transformed_path.header;
    reactive_path.poses  = {it_robot, transformed_path.poses.end()};


    // Publish initial path
    initial_path_.header.stamp = this->get_clock()->now();
    initial_path_publisher_->publish(initial_path_);

    // Publish reactive path
    reactive_path_publisher_->publish(reactive_path);
  }
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr initial_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reactive_path_publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path initial_path_;
  tf2::Duration transform_tolerance_;
  std::string robot_base_frame_, global_frame_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Define initial path
  double interpolation_distance = 0.5;  // in meters
  nav_msgs::msg::Path initial_path;
  initial_path.header.frame_id = "odom";

  // Fill up poses
  geometry_msgs::msg::PoseStamped ps;
  ps.header             = initial_path.header;
  ps.pose.orientation.w = 1.0;
  // P1
  ps.pose.position.x = 0.0;
  ps.pose.position.y = 0.0;
  initial_path.poses.push_back(ps);
  // P2
  ps.pose.position.x = 10.0;
  ps.pose.position.y = 0.0;
  initial_path.poses.push_back(ps);
  // P3
  ps.pose.position.x = 30.0;
  ps.pose.position.y = 2.0;
  initial_path.poses.push_back(ps);

  // Interpolate path
  initial_path = nav_utils::interpolatePath(initial_path, interpolation_distance);

  // Create path publisher object
  try
  {
    rclcpp::spin(std::make_shared<PathPublisher>(initial_path));
  }
  catch(const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("publish_reactive_path"), "%s.", e.what());
  }

  rclcpp::shutdown();

  return 0;
}