// ROS2 node that processes RS-Helios 5515 scans, converting them from XYZI to XYZIRT
// format. The following operations are performed:
// - Add "ring" channel by matching vertical angle to ring information from table 5 in device manual
// - Add "time" channel by checking ring and horizontal angle according to information from section 7.2 of manual
// Source: https://cdn-reichelt.de/documents/datenblatt/C900/RS-HELIOS_5515_USER_GUIDE_V3.0.1_EN.pdf

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

struct PointXYZIRT 
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, time, time))

constexpr float v_time_array[32] = 
{0.00, 1.57, 3.15, 4.72, 6.3, 7.87, 9.45, 11.36, 13.26, 15.17, 17.08, 18.99, 20.56, 22.14,
 23.71, 25.29, 26.53, 27.77, 29.01, 30.25, 31.49, 32.73, 33.98, 35.22, 36.46, 37.7, 38.94,
 40.18, 41.42, 42.67, 43.91, 45.15};
constexpr float TIME_OFFSET = 55.56;

class RSHeliosConverterNode : public rclcpp::Node 
{
  public:
  RSHeliosConverterNode() : Node("RSHeliosConverterNode") 
  {
    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_converted", 10);
    sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 10,
                std::bind(&RSHeliosConverterNode::callback, this, std::placeholders::_1));
  }

  // Could not find a better way to remap to ring. Mapping is not linear (although some fractions are)
  // Note that we start the ring number on '0' instead of '1'.
  uint16_t getRing(const double angle)
  {
    if     (angle >  14.0) return 0;
    else if(angle >  12.0) return 1;
    else if(angle >  10.0) return 2;
    else if(angle >  8.0 ) return 3;
    else if(angle >  6.25) return 4;
    else if(angle >  4.75) return 5;
    else if(angle >  3.33) return 6;
    else if(angle >  2.0 ) return 7;
    else if(angle >  0.67) return 8;
    else if(angle > -0.67) return 9;
    else if(angle > -2.0 ) return 10;
    else if(angle > -3.33) return 11;
    else if(angle > -4.67) return 12;
    else if(angle > -6.0 ) return 13;
    else if(angle > -7.33) return 14;
    else if(angle > -9.0 ) return 15;
    else if(angle > -11.5) return 16;
    else if(angle > -14.5) return 17;
    else if(angle > -17.5) return 18;
    else if(angle > -20.5) return 19;
    else if(angle > -23.5) return 20;
    else if(angle > -26.5) return 21;
    else if(angle > -29.5) return 22;
    else if(angle > -32.5) return 23;
    else if(angle > -35.5) return 24;
    else if(angle > -38.5) return 25;
    else if(angle > -41.5) return 26;
    else if(angle > -44.5) return 27;
    else if(angle > -47.5) return 28;
    else if(angle > -50.5) return 29;
    else if(angle > -53.5) return 30;
    else return 31;
  }

  inline void processPoint(PointXYZIRT& pt)
  {
    // Calculate vertical angle and deduce ring from it
    const double v_ang = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180.0 / M_PI;
    pt.ring = getRing(v_ang);

    // Calculate horizontal angle
    const double h_ang = atan2(pt.y, pt.x) * 180.0 / M_PI;

    // 1800 points per ring: horizontal resolution of 0.2 deg
    // Assuming initial angle of 0 degrees.
    // Note that the sensor rotates clockwise, so the index sign is inverted.
    int h_index = -floor(h_ang/0.2);
    if(h_index < 0) h_index += 1800; 

    // Deduce timestamp from ring and horizontal index
    // Assmuming single return mode. See Table 13 for information
    pt.time = 1e-6*(v_time_array[pt.ring] + TIME_OFFSET*h_index);
  }

  void callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) 
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<PointXYZIRT> cloud_processed;
    pcl::fromROSMsg(*msg, cloud);
    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(cloud, cloud, ind);

    for(auto& pt : cloud.points)
    {
      // Discard nan points
      PointXYZIRT pt_processed;
      pt_processed.x = pt.x;
      pt_processed.y = pt.y;
      pt_processed.z = pt.z;
      pt_processed.intensity = pt.intensity;
      processPoint(pt_processed);
      cloud_processed.points.push_back(pt_processed);
    }

    sensor_msgs::msg::PointCloud2 msg_converted;
    pcl::toROSMsg<PointXYZIRT>(cloud_processed, msg_converted);
    msg_converted.header = msg->header;
    pub->publish(msg_converted);
  }

  protected:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
};

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RSHeliosConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

