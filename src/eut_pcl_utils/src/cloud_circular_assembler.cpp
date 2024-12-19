
#include <stdexcept>
#include <string>

#include <boost/circular_buffer.hpp>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2_eigen/tf2_eigen.h>

// ROS2 node that accumulates N (buffer_size) pointclouds and publishes the result.

class CloudAssembler: public rclcpp::Node
{
  public:
  explicit CloudAssembler():
    rclcpp::Node{"CloudAssembler"},
    buffer_size_{this->declare_parameter("buffer_size", 10)},  // how many pointclouds to store
    cloud_sub_{this, "cloud_registered", rmw_qos_profile_default},
    odom_sub_{this, "Odometry", rmw_qos_profile_default},
    assembled_cloud_pub_{
      this->create_publisher<sensor_msgs::msg::PointCloud2>("assembled_cloud", 1UL)},
    sync_policy_{cloud_assembler_sync_policy{10}, cloud_sub_, odom_sub_}
  {
    if(buffer_size_ < 1)
    {
      throw std::invalid_argument{"Buffer size must be a positive number for a CloudAssmbler"};
    }

    sync_policy_.registerCallback(&CloudAssembler::syncCallback, this);

    pcl_cloud_buffer_.set_capacity(static_cast<size_t>(buffer_size_));
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  void syncCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
                    const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    // Important: The frame in which the input ros clouds are referred must be the same as the frame
    // in which the odometry data is referred, i.e:
    // cloud_msg->header.frame_id must be the same as odom_msg->header.frame_id.
    // If this requirement is not met, then we cannot convert the assembled cloud into the
    // child_frame_id indicated in the odometry message, because we won't have the whole chain of
    // transformations to achieve the conversion of data.

    if(cloud_msg->header.frame_id != odom_msg->header.frame_id)
    {
      std::string msg;
      // auto total = 45 + cloud_msg->header.frame_id.length() + 54 +
      //              odom_msg->header.frame_id.length() + 35 = 134 + ...

      msg.reserve(134 + cloud_msg->header.frame_id.length() + odom_msg->header.frame_id.length());
      msg.append("The input cloud is referred w.r.t the frame '")
        .append(cloud_msg->header.frame_id)
        .append("', but the odometry pose is referred w.r.t the frame '")
        .append(odom_msg->header.frame_id)
        .append("'!.No cloud assembling is performed");

      RCLCPP_WARN_STREAM(this->get_logger(), msg);

      return;
    }

    // Transform the ros cloud into a pcl cloud, store it in the circular buffer, so, in next step,
    // when assembling the clouds, we do not have to converter the same ros cloud into a pcl cloud
    // in several call round of this function.
    pcl::PointCloud<pcl::PointXYZ> input_pcl_cloud;
    pcl::moveFromROSMsg(*cloud_msg, input_pcl_cloud);
    pcl_cloud_buffer_.push_back(input_pcl_cloud);

    // Accumulate the clouds in the circular buffer and then convert the resulting cloud into the
    // the frame odom_msg->header.child_frame_id.
    pcl::PointCloud<pcl::PointXYZ> pcl_assembled_cloud;

    for(const auto& pcl_cloud: pcl_cloud_buffer_)
    {
      pcl_assembled_cloud += pcl_cloud;
    }

    // The pose from the odometry message represents the orientation and the translation of the
    // frame odom_msg->header.child_frame_id w.r.t the frame odom_msg->header.frame, which in fact
    // is a homogeneus transformation matrix, represented here with the symbol T:
    //             T
    // odom_frame ---> child_frame
    //            <---
    //             T^(-1)
    // T is used to transform data, referred w.r.t the frame child_frame, to data referred w.r.t the
    // frame odom_frame (yes the direction of the arrow is right).
    // On the other hand, T^(-1), is used to transform data referred w.r.t the frame odom_frame,
    // to data referred w.r.t to the frame chil_frame.
    Eigen::Isometry3d T;
    tf2::fromMsg(odom_msg->pose.pose, T);
    auto T_inv = T.matrix().cast<float>().inverse();
    pcl::transformPointCloud(pcl_assembled_cloud, pcl_assembled_cloud, T_inv);

    // Transform the assembled cloud from pcl format to ros format, and publish it.
    sensor_msgs::msg::PointCloud2 assembled_cloud;
    pcl::toROSMsg(pcl_assembled_cloud, assembled_cloud);
    assembled_cloud.header.frame_id = odom_msg->child_frame_id;
    assembled_cloud.header.stamp    = cloud_msg->header.stamp;
    assembled_cloud_pub_->publish(assembled_cloud);
  }

  private:
  long int buffer_size_;
  boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_buffer_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr assembled_cloud_pub_;

  using cloud_assembler_sync_policy = message_filters::sync_policies::
    ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

  message_filters::Synchronizer<cloud_assembler_sync_policy> sync_policy_;
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<CloudAssembler> cloud_assembler;

  try
  {
    cloud_assembler = std::make_shared<CloudAssembler>();
    RCLCPP_INFO(rclcpp::get_logger("CloudAssembler"), "CloudAssembler initialized successfully");
  }
  catch(const std::bad_alloc& ex)
  {
    RCLCPP_FATAL(rclcpp::get_logger("CloudAssembler"), "[CloudAssembler] Memory exhausted");
  }
  catch(const std::invalid_argument& ex)
  {
    RCLCPP_FATAL(rclcpp::get_logger("CloudAssembler"), "[CloudAssembler] %s.", ex.what());
  }

  rclcpp::spin(cloud_assembler);
  rclcpp::shutdown();

  return 0;
}