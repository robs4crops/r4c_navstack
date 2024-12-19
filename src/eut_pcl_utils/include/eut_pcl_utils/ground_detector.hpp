#pragma once

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eut_pcl_utils/pcl_utils.hpp>

namespace pcl_utils
{
  /**
   * @brief Ground Detector Class constructor
   * @param params GroundParam struct
   */
  class GroundDetector
  {
    public:
    struct GroundParams
    {
      tf2::Vector3 direction{tf2::Vector3(0, 0, 1)};
      double min_x{-15.0};
      double max_x{15.0};
      double min_y{-5.0};
      double max_y{5.0};
      double min_z{0.2};
      double max_z{1.2};
      int ransac_max_iter{100};
      double ransac_dist_thresh{0.1};
      float voxel_size{0.05f};
      int min_ground_size{1000};
      double ground_thresh{0.9};
      int ground_max_iters{10};
      double max_dist_to_ground{0.2};
    };

    /**
     * @brief Constructor for the GroundDetector
     * @param params ground params
     */
    GroundDetector(const GroundParams& params);

    /**
     * @brief Find the ground plane of a given pointcloud. If no ground is found, the plane
     * coefficients default to the defined ground direction
     * @param cloud A pcl::PointXYZ Ptr to find the ground
     * @return whether the algorithm has been able to find the ground plane
     */
    bool findPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Get the plane coefficients of the ground
     * @return pcl::ModelCoefficients with the plane coefficients
     */
    pcl::ModelCoefficients getPlaneCoefs();

    /**
     * @brief Get the ground plane pointcloud
     * @return pcl::PointXYZ with the resulting pcl
     */
    pcl::PointCloud<pcl::PointXYZ> getGroundPcl();

    /**
     * @brief Given a plane represented in a source frame, this functions calculates the transform
     * between the source frame and the plane described by the coefficients
     * @return tf2::Transform from source frame to ground frame
     */
    tf2::Transform getPlaneTransform();

    private:
    GroundParams params_;
    pcl::ModelCoefficients coef_;
    pcl::PointCloud<pcl::PointXYZ> ground_pcl_;
  };
};  // namespace pcl_utils