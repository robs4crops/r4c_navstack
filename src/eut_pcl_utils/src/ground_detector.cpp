#include <eut_pcl_utils/ground_detector.hpp>

namespace pcl_utils
{
  GroundDetector::GroundDetector(const GroundParams& params): params_{params} {}

  bool GroundDetector::findPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Filter cloud ////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = pcl_utils::filterCloud(cloud,
                                                                                params_.min_x,
                                                                                params_.max_x,
                                                                                params_.min_y,
                                                                                params_.max_y,
                                                                                params_.min_z,
                                                                                params_.max_z);

    //////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Downsample cloud //////////////////////////////
    //////////////////////////////////////////////////////////////////////////////

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
    vg.filter(*cloud_ds);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);

    bool ground_found = false;

    for(int iter = 0; iter < params_.ground_max_iters; iter++)
    {
      //////////////////////////////////////////////////////////////////////////////
      /////////////////////////////// Apply RANSAC /////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////

      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(params_.ransac_max_iter);
      seg.setDistanceThreshold(params_.ransac_dist_thresh);
      seg.setInputCloud(cloud_ds);
      seg.segment(*inliers, *coef);

      // Coefficients describe a plane in the form Ax + By + Cz + D = 0
      tf2::Vector3 plane_direction(coef->values[0], coef->values[1], coef->values[2]);

      // Find distance from plane to robot
      tf2::Vector3 pt(0, 0, 0);
      double dist_to_plane = std::fabs(distancePointToPlane(pt, *coef));

      //////////////////////////////////////////////////////////////////////////////
      ///////////////////////// Check ground requirements //////////////////////////
      //////////////////////////////////////////////////////////////////////////////
      bool ground_conditions_met = true;
      int ground_size            = static_cast<int>(inliers->indices.size());
      double thresh              = std::fabs(tf2::tf2Dot(params_.direction, plane_direction));
      if(dist_to_plane >= params_.max_dist_to_ground)
      {
        ground_conditions_met = false;
        RCLCPP_DEBUG(rclcpp::get_logger("GroundDetector"),
                     " [Iter: %i] Distance to plane %.2f exceeded maximum %.2f",
                     iter,
                     dist_to_plane,
                     params_.max_dist_to_ground);
      }

      if(thresh <= params_.ground_thresh)
      {
        ground_conditions_met = false;
        RCLCPP_DEBUG(rclcpp::get_logger("GroundDetector"),
                     " [Iter: %i] Ground threshold %.2f exceeded maximum %.2f",
                     iter,
                     thresh,
                     params_.ground_thresh);
      }
      if(ground_size <= params_.min_ground_size)
      {
        ground_conditions_met = false;
        RCLCPP_DEBUG(rclcpp::get_logger("GroundDetector"),
                     " [Iter: %i] Ground size %i did not reach minimum %i",
                     iter,
                     ground_size,
                     params_.min_ground_size);
      }

      //////////////////////////////////////////////////////////////////////////////
      //////////////////////////////// Find ground /////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////

      if(ground_conditions_met)
      {
        RCLCPP_DEBUG(rclcpp::get_logger("GroundDetector"),
                     " [Iter: %i] Ground plane found of size: %u",
                     iter,
                     ground_size);

        // Save ground pcl
        ground_found = true;
        extract.setInputCloud(cloud_ds);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(ground_pcl_);

        coef_ = *coef;
        break;
      }
      else
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud2);
        pcl::copyPointCloud(*cloud2, *cloud_ds);
      }
    }

    return ground_found;
  }

  pcl::ModelCoefficients GroundDetector::getPlaneCoefs()
  {
    return coef_;
  }

  pcl::PointCloud<pcl::PointXYZ> GroundDetector::getGroundPcl()
  {
    return ground_pcl_;
  }

  tf2::Transform GroundDetector::getPlaneTransform()
  {
    float A = coef_.values[0];
    float B = coef_.values[1];
    float C = coef_.values[2];

    tf2::Vector3 origin(0, 0, 0);
    tf2::Vector3 new_origin = pcl_utils::projectPointOntoPlane(origin, coef_);

    tf2::Vector3 v_z(0, 0, 1);
    tf2::Vector3 v_z_new(A, B, C);
    v_z_new = v_z_new.normalized();

    tf2::Quaternion q = pcl_utils::getQuaternionRotation(v_z, v_z_new);

    tf2::Transform plane_trans;
    plane_trans.setOrigin(new_origin);
    plane_trans.setRotation(q);

    return plane_trans.inverse();
  }


};  // namespace pcl_utils