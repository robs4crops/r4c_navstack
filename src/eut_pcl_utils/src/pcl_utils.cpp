#include <eut_pcl_utils/pcl_utils.hpp>

namespace pcl_utils
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                                  double min_x,
                                                  double max_x,
                                                  double min_y,
                                                  double max_y,
                                                  double min_z,
                                                  double max_z)
  {
    // https://github.com/PointCloudLibrary/pcl/blob/master/filters/src/passthrough.cpp
    // https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/passthrough.h

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    int nr_points = input->width * input->height;

    output->height   = 1;
    output->is_dense = true;
    output->points.reserve(nr_points);

    int nr_p = 0;

    pcl::PointXYZ pt;

    // Go over all points;
    for(int cp = 0; cp < nr_points; ++cp)
    {
      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy(&pt.x, &input->points[cp].x, sizeof(float));
      memcpy(&pt.y, &input->points[cp].y, sizeof(float));
      memcpy(&pt.z, &input->points[cp].z, sizeof(float));

      // Check if the point is invalid
      if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        continue;
      }

      // Use a threshold for cutting out points which are too close/far away
      if(pt.x > max_x || pt.x < min_x || pt.y > max_y || pt.y < min_y || pt.z > max_z ||
         pt.z < min_z)
      {
        continue;
      }

      // Copy all the fields
      output->points.push_back(pt);

      memcpy(&output->points[nr_p].x, &pt.x, sizeof(float));
      memcpy(&output->points[nr_p].y, &pt.y, sizeof(float));
      memcpy(&output->points[nr_p].z, &pt.z, sizeof(float));
      nr_p++;
    }
    output->width = nr_p;

    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                                  double min_angle,
                                                  double max_angle,
                                                  double min_range,
                                                  double max_range)
  {
    // https://github.com/PointCloudLibrary/pcl/blob/master/filters/src/passthrough.cpp
    // https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/passthrough.h

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    int nr_points = input->width * input->height;

    output->height   = 1;
    output->is_dense = true;
    output->points.reserve(nr_points);

    int nr_p = 0;

    pcl::PointXYZ pt;

    // Go over all points;
    for(int cp = 0; cp < nr_points; ++cp)
    {
      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy(&pt.x, &input->points[cp].x, sizeof(float));
      memcpy(&pt.y, &input->points[cp].y, sizeof(float));
      memcpy(&pt.z, &input->points[cp].z, sizeof(float));

      // Check if the point is invalid
      if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        continue;
      }

      // Filter by angle
      double angle = atan2(pt.y, pt.x);
      if(angle < min_angle || angle > max_angle)
      {
        continue;
      }

      // Filter by range
      double range = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if(range < min_range || range > max_range)
      {
        continue;
      }

      // Copy all the fields
      output->points.push_back(pt);

      memcpy(&output->points[nr_p].x, &pt.x, sizeof(float));
      memcpy(&output->points[nr_p].y, &pt.y, sizeof(float));
      memcpy(&output->points[nr_p].z, &pt.z, sizeof(float));
      nr_p++;
    }
    output->width = nr_p;

    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloudAngle(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    double min_angle,
    double max_angle)
  {
    // https://github.com/PointCloudLibrary/pcl/blob/master/filters/src/passthrough.cpp
    // https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/passthrough.h

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    int nr_points = input->width * input->height;

    output->height   = 1;
    output->is_dense = true;
    output->points.reserve(nr_points);

    int nr_p = 0;

    pcl::PointXYZ pt;

    // Go over all points;
    for(int cp = 0; cp < nr_points; ++cp)
    {
      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy(&pt.x, &input->points[cp].x, sizeof(float));
      memcpy(&pt.y, &input->points[cp].y, sizeof(float));
      memcpy(&pt.z, &input->points[cp].z, sizeof(float));

      // Check if the point is invalid
      if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        continue;
      }

      // Filter by angle
      double angle = atan2(pt.y, pt.x);
      if(angle < min_angle || angle > max_angle)
      {
        continue;
      }

      // Copy all the fields
      output->points.push_back(pt);

      memcpy(&output->points[nr_p].x, &pt.x, sizeof(float));
      memcpy(&output->points[nr_p].y, &pt.y, sizeof(float));
      memcpy(&output->points[nr_p].z, &pt.z, sizeof(float));
      nr_p++;
    }
    output->width = nr_p;

    return output;
  }

  double distancePointToPlane(const tf2::Vector3& point, const pcl::ModelCoefficients& plane)
  {
    // Signed distance
    double A = plane.values[0];
    double B = plane.values[1];
    double C = plane.values[2];
    double D = plane.values[3];
    // distance from point to plane
    return (A * point.x() + B * point.y() + C * point.z() + D) / sqrt(A * A + B * B + C * C);
  }

  tf2::Vector3 projectPointOntoPlane(const tf2::Vector3& point, const pcl::ModelCoefficients& plane)
  {
    // https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d
    // https://www.cuemath.com/geometry/distance-between-point-and-plane/
    double A = plane.values[0];
    double B = plane.values[1];
    double C = plane.values[2];
    // distance from point to plane
    double dist_to_plane = distancePointToPlane(point, plane);
    tf2::Vector3 v_n(A, B, C);
    return point - dist_to_plane * v_n;
  }

  void publishCloudMessage(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                           const std_msgs::msg::Header header,
                           const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header = header;
    pub->publish(cloud_msg);
  }

  tf2::Quaternion getQuaternionRotation(tf2::Vector3& v1, tf2::Vector3& v2)
  {
    // Source:
    // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another

    // Normalize vectors
    v1 = v1.normalized();
    v2 = v2.normalized();

    // Check if vectors are parallel
    if(tf2::tf2Dot(v1, v2) > 0.999999)
    {
      tf2::Quaternion q(0, 0, 0, 1);
      return q;
    }
    else if(tf2::tf2Dot(v1, v2) < -0.999999)
    {
      // Calculate 180 degree rotation around any orthogonal vector
      // This implementation uses the cross product with the most orthogonal basis vector.
      double x = fabs(v1.getX());
      double y = fabs(v1.getY());
      double z = fabs(v1.getZ());

      tf2::Vector3 other = x < y ? (x < z ? tf2::Vector3(1, 0, 0) : tf2::Vector3(0, 0, 1)) :
                                   (y < z ? tf2::Vector3(0, 1, 0) : tf2::Vector3(0, 0, 1));
      tf2::Vector3 orth  = tf2::tf2Cross(v1, other);

      tf2::Quaternion q(orth.getX(), orth.getY(), orth.getZ(), 0);
      q.normalize();
      return q;
    }

    tf2::Vector3 q_xyz = tf2::tf2Cross(v1, v2);
    tf2Scalar q_w      = sqrt((v1.length2()) * (v2.length2())) + tf2::tf2Dot(v1, v2);
    tf2::Quaternion q(q_xyz.getX(), q_xyz.getY(), q_xyz.getZ(), q_w);
    q.normalize();
    return q;
  }

  void publishPlanePolygon(
    const pcl::ModelCoefficients& plane,
    const std_msgs::msg::Header& header,
    const double& length,
    const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr& pub)
  {
    float A = plane.values[0];
    float B = plane.values[1];
    float C = plane.values[2];
    float D = plane.values[3];

    geometry_msgs::msg::Polygon polygon;
    geometry_msgs::msg::PolygonStamped polygon_stamped;
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(length);
    pt.y = static_cast<float>(length);
    pt.z = -(A * pt.x + B * pt.y + D) / C;
    polygon.points.push_back(pt);

    pt.x = -static_cast<float>(length);
    pt.y = static_cast<float>(length);
    pt.z = -(A * pt.x + B * pt.y + D) / C;
    polygon.points.push_back(pt);

    pt.x = -static_cast<float>(length);
    pt.y = -static_cast<float>(length);
    pt.z = -(A * pt.x + B * pt.y + D) / C;
    polygon.points.push_back(pt);

    pt.x = static_cast<float>(length);
    pt.y = -static_cast<float>(length);
    pt.z = -(A * pt.x + B * pt.y + D) / C;
    polygon.points.push_back(pt);

    polygon_stamped.header  = header;
    polygon_stamped.polygon = polygon;

    pub->publish(polygon_stamped);
  }
};  // namespace pcl_utils
