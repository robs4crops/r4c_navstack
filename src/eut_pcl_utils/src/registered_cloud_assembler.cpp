#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include <sensor_msgs/msg/point_cloud2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <queue>
#include <list>

// Script that accumulates multiple scans of a PointCloud2 topic up to a given number
// and publishes as another PointCloud2. It is used in this project to provide denser
// lidar information for perception tasks, given a reliable odometry source.
// Assuming cloud and odometry information are synchronized.
// Assuming as well the cloud is already published in the map frame.

// Instructions: Launch eut_fast_lio first, then this node!

rclcpp::Node::SharedPtr node_ = nullptr;
int buffer_size{10};
std::list<sensor_msgs::msg::PointCloud2> clouds;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr assembled_cloud_pub;

void syncCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
                  const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  // Append to queue and pop if size exceeds a limit
  clouds.push_back(*cloud_msg);
  if(clouds.size() > buffer_size)
  {
    // RCLCPP_WARN(node_->get_logger(),"[REGISTERED CLOUD ASSEMBLER] Cleaning cause %d",
    // clouds.size());
    clouds.pop_front();
  }

  // Then sum all clouds and transform to base frame using odometry information
  pcl::PointCloud<pcl::PointXYZ> assembled_cloud_pcl;
  for(auto cloud: clouds)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
    pcl::fromROSMsg(cloud, cloud_pcl);
    assembled_cloud_pcl += cloud_pcl;
  }

  Eigen::Isometry3d odom_pose;
  tf2::fromMsg(odom_msg->pose.pose, odom_pose);
  pcl::transformPointCloud(assembled_cloud_pcl,
                           assembled_cloud_pcl,
                           odom_pose.matrix().cast<float>().inverse());

  // Convert back to ROS and publish assembled cloud
  sensor_msgs::msg::PointCloud2 assembled_cloud;
  pcl::toROSMsg(assembled_cloud_pcl, assembled_cloud);
  assembled_cloud.header.frame_id = odom_msg->child_frame_id;
  assembled_cloud.header.stamp    = cloud_msg->header.stamp;
  assembled_cloud_pub->publish(assembled_cloud);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node_ = rclcpp::Node::make_shared("registered_cloud_assembler");

  node_->declare_parameter("buffer_size", 10);
  node_->get_parameter("buffer_size", buffer_size);

  // define quality of service: all messages that you want to receive must have the same
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  // create subscribers to the topics of interest
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub(node_,
                                                                       "cloud_registered",
                                                                       custom_qos_profile);
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub(node_,
                                                                "Odometry",
                                                                custom_qos_profile);

  assembled_cloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>("assembled_cloud",
                                                                               10);

  // message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>
  // syncExact(cloud_sub, odom_sub, 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                          nav_msgs::msg::Odometry>
    approximate_policy;
  message_filters::Synchronizer<approximate_policy> syncApproximate(approximate_policy(10),
                                                                    cloud_sub,
                                                                    odom_sub);

  // register the exact time callback
  // syncExact.registerCallback(syncCallback);
  syncApproximate.registerCallback(syncCallback);

  RCLCPP_INFO(node_->get_logger(), "[REGISTERED CLOUD ASSEMBLER] Inititalized successfully");

  rclcpp::spin(node_);

  return 0;
}