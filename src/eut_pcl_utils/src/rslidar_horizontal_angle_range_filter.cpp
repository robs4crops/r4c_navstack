#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <eut_pcl_utils/pcl_utils.hpp>

// Global variables
double min_angle, max_angle;
double min_range, max_range;
rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;

// Lidar callback
void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert from ROS to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud = pcl_utils::filterCloud(cloud, min_angle, max_angle, min_range, max_range);

  sensor_msgs::msg::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(*filtered_cloud, pcl_ros_msg);
  pcl_ros_msg.header = msg->header;
  cloud_pub->publish(pcl_ros_msg);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("rslidar_filter");

  // Load parameters
  min_range = static_cast<double>(node->declare_parameter("min_range", 0.0));
  max_range = static_cast<double>(node->declare_parameter("max_range", 100.0));

  min_angle = static_cast<double>(node->declare_parameter("min_angle", -M_PI));
  max_angle = static_cast<double>(node->declare_parameter("max_angle", M_PI));

  RCLCPP_INFO(node->get_logger(),
              "[filterRSLidar] Range: [%.3f, %.3f], Angle: [%.3f, %.3f]",
              min_range,
              max_range,
              min_angle,
              max_angle);

  // Publishers and subscribers
  auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points",
                                                                            100,
                                                                            lidarCallback);
  cloud_pub      = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_points/filtered",
                                                                    100);

  RCLCPP_INFO(node->get_logger(), "[filterRSLidar] Inititalized successfully");

  // Spin node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}