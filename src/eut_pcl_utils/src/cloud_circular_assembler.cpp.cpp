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
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>


// ROS2 node that accumulates N (buffer_size) pointclouds and publishes the result.

class CloudAssembler: public rclcpp::Node
{
  public:
  explicit CloudAssembler():
    rclcpp::Node{"CloudAssembler"},
    fixed_frame_{this->declare_parameter("fixed_frame", "map")},
    target_frame_{this->declare_parameter("target_frame", "base_link")},
    tf_buffer_{this->get_clock()},
    tf_listener_{tf_buffer_},
    pointcloud_sub_filter_{this,
                           this->declare_parameter("input_pointcloud_topic", "cloud"),
                           rmw_qos_profile_default},
    pointcloud_assembler_tf_filter_{pointcloud_sub_filter_,
                                    tf_buffer_,
                                    fixed_frame_,
                                    10,
                                    this->shared_from_this()},
    assembled_cloud_pub_{
      this->create_publisher<sensor_msgs::msg::PointCloud2>("assembled_cloud", 1UL)}
  {
    // Assure buffer_size >= 0
    auto buffer_size = std::max(0l, this->declare_parameter("buffer_size", 10));

    if(buffer_size_ < 1)
    {
      RCLCPP_WARN(this->get_logger(), "Buffer size is 0");
    }

    pcl_buffer_.set_capacity(static_cast<size_t>(buffer_size));

    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    pointcloud_assembler_tf_filter_.registerCallback(
      std::bind(&CloudAssembler::accumulate_pointcloud, this, std::placeholders::_1));
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  void accumulate_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl;

    if(cloud_msg->header.frame_id != fixed_frame_)
    {
      // Since this callback function has been called, then the
      // T_<fixed_frame>_<cloud.header.frame_id> exists, so no need to use a try{}catch{} block.
      auto T_fixed_fr_cloud_fr = tf_buffer_.lookupTransform(fixed_frame_,
                                                            cloud_msg->header.frame_id,
                                                            tf2::TimePointZero);

      sensor_msgs::msg::PointCloud2 transformed_cloud_msg;

      tf2::doTransform(*cloud_msg, transformed_cloud_msg, T_fixed_fr_cloud_fr);

      pcl::moveFromROSMsg(transformed_cloud_msg, pcl);
    }
    else
    {
      pcl::fromROSMsg(*cloud_msg, pcl); // since cloud_msg is const we cannot move, just copy.
    }

    pcl_buffer_.push_back(std::move(pcl));

    // Accumulate the clouds in the circular buffer and then convert the resulting cloud into the
    // the frame odom_msg->header.child_frame_id.
    pcl::PointCloud<pcl::PointXYZ> assembled_pcl;

    for(const auto& pcl: pcl_buffer_)
    {
      assembled_pcl += pcl;
    }

    sensor_msgs::msg::PointCloud2 assembled_cloud_msg;
    pcl::toROSMsg(assembled_pcl, assembled_cloud_msg);

    assembled_cloud_msg.header.frame_id = target_frame_;
    assembled_cloud_msg.header.stamp    = this->now();

    if(target_frame_ != fixed_frame_)
    {
      geometry_msgs::msg::TransformStamped T_target_fr_fixed_fr;

      try
      {
        T_target_fr_fixed_fr = tf_buffer_.lookupTransform(target_frame_,
                                                          fixed_frame_,
                                                          tf2::TimePointZero);
      }
      catch(tf2::TransformException& ex)
      {
        RCLCPP_WARN(this->get_logger(), "%s.", ex.what());

        return;
      }

      sensor_msgs::msg::PointCloud2 output_cloud_msg;

      tf2::doTransform(assembled_cloud_msg, output_cloud_msg, T_target_fr_fixed_fr);

      output_cloud_msg.header = assembled_cloud_msg.header;

      assembled_cloud_pub_->publish(output_cloud_msg);
    }
    else
    {
      assembled_cloud_pub_->publish(assembled_cloud_msg);
    }
  }

  private:
  long int buffer_size_;
  std::string fixed_frame_;
  std::string target_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  // listen tf and stores them in the tf_buffersI
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_filter_;
  tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> pointcloud_assembler_tf_filter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr assembled_cloud_pub_;

  boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>> pcl_buffer_;
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