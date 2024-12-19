#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace pcl_utils
{
  /**
   * @brief Filter pcl::Pointcloud by x, y and z fields
   * @param[in] input Input pcl::PointXYZ pointer to filter
   * @param[in] min_x Minimum x value to filter the pointcloud
   * @param[in] max_x Maximum x value to filter the pointcloud
   * @param[in] min_y Minimum y value to filter the pointcloud
   * @param[in] max_y Maximum y value to filter the pointcloud
   * @param[in] min_z Minimum z value to filter the pointcloud
   * @param[in] max_z Maximum z value to filter the pointcloud
   * @return filtered pcl::PointXYZ pointer
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                                  double min_x,
                                                  double max_x,
                                                  double min_y,
                                                  double max_y,
                                                  double min_z,
                                                  double max_z);

  /**
   * @brief Filter pcl::Pointcloud by angle and range
   * @param[in] input Input pcl::PointXYZ pointer to filter
   * @param[in] min_angle Minimum angle
   * @param[in] max_angle Maximum angle
   * @param[in] min_range Minimum range
   * @param[in] max_range Maximum range
   * @return filtered pcl::PointXYZ pointer
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                                  double min_angle,
                                                  double max_angle,
                                                  double min_range,
                                                  double max_range);

  /**
   * @brief Filter pcl::Pointcloud by angle
   * @param[in] input Input pcl::PointXYZ pointer to filter
   * @param[in] min_angle Minimum angle
   * @param[in] max_angle Maximum angles
   * @return filtered pcl::PointXYZ pointer
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloudAngle(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    double min_angle,
    double max_angle);

  /**
   * @brief Get the orthogonal signed distance from a point to a plane
   * @param[in] point tf2::Vector3 point
   * @param[in] plane pcl::ModelCoefficients plane
   * @return orthogonal signed distance
   */
  double distancePointToPlane(const tf2::Vector3& point, const pcl::ModelCoefficients& plane);

  /**
   * @brief Get the orthogonal projection of a point into a plane
   * @param[in] point tf2::Vector3 point
   * @param[in] plane pcl::ModelCoefficients plane
   * @return projected tf2::Vector3 point into the plane
   */
  tf2::Vector3 projectPointOntoPlane(const tf2::Vector3& point,
                                     const pcl::ModelCoefficients& plane);

  /**
   * @brief Publish a sensor_msgs::PointCloud2 from a pcl::PointXYZ in a given frame
   * @param[in] cloud The cloud to publish
   * @param[in] header The header to use
   * @param[in] pub The publisher to use
   */
  void publishCloudMessage(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                           const std_msgs::msg::Header header,
                           const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub);

  /**
   * @brief Publish a PointCloud2 message from a templated pointcloud in a given frame
   * @param[in] cloud Pointcloud to publish
   * @param[in] tf_matrix A transform matrix
   * @param[in] header The header to use
   * @param[in] pub The publisher to use
   */
  template<typename PointT>
  void publishCloudMessage(const pcl::PointCloud<PointT>& cloud,
                           const Eigen::Isometry3d& tf_matrix,
                           const std_msgs::msg::Header& header,
                           const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_transformed(new typename pcl::PointCloud<PointT>);
    pcl::transformPointCloud<PointT>(cloud, *cloud_transformed, tf_matrix.matrix());

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_transformed, cloud_msg);
    cloud_msg.header = header;
    pub->publish(cloud_msg);
  }

  /**
   * @brief Get the normalized quaternion that represents the 3D rotation between two vectors.
   * Handles special cases of parallel vectors.
   * @param[in] v1 First vector
   * @param[in] v2 Second vector
   * @return normalized quaternion rotation
   */
  tf2::Quaternion getQuaternionRotation(tf2::Vector3& v1, tf2::Vector3& v2);

  /**
   * @brief Publish geometry_msgs::PolygonStamped message based on the model coefficients of a plane
   * @param[in] plane Coefficients of the plane
   * @param[in] header The header to use
   * @param[in] length Length of the sides of the polygon
   * @param[in] pub The publisher to use
   */
  void publishPlanePolygon(
    const pcl::ModelCoefficients& plane,
    const std_msgs::msg::Header& header,
    const double& length,
    const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr& pub);
};  // namespace pcl_utils
