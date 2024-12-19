#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>

namespace r4c::localization
{

  class AgcBoxImuFilterNode: public rclcpp::Node
  {
    public:
    AgcBoxImuFilterNode():
      rclcpp::Node("enu_to_navsatfix"),
      agcbox_imu_frame_{
        this->declare_parameter<std::string>("agcbox_imu_frame", "agcbox_imu_link")},
      sub_imu_{this->create_subscription<sensor_msgs::msg::Imu>(
        "/input_imu",
        10,
        std::bind(&AgcBoxImuFilterNode::IMUCallback, this, std::placeholders::_1))},
      pub_imu_{this->create_publisher<sensor_msgs::msg::Imu>("/output_imu", 10)}
    {}

    // Only publish if IMU message comes from AGC Box (by checking is frame_id).
    void IMUCallback(sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
      if(imu_msg->header.frame_id.compare(agcbox_imu_frame_) == 0)
      {
        // Correct orientation (currently reversed and with a +/-15 deg. offset)
        sensor_msgs::msg::Imu imu_msg_fixed(*imu_msg);
        Eigen::Quaterniond q(imu_msg_fixed.orientation.w,
                             imu_msg_fixed.orientation.x,
                             imu_msg_fixed.orientation.y,
                             imu_msg_fixed.orientation.z);
        q *= Eigen::Quaterniond(Eigen::AngleAxisd((-0.2617994), Eigen::Vector3d(0, 0, 1)));
        imu_msg_fixed.orientation.x = q.x();
        imu_msg_fixed.orientation.y = q.y();
        imu_msg_fixed.orientation.z = q.z();
        imu_msg_fixed.orientation.w = q.w();
        pub_imu_->publish(imu_msg_fixed);
      }
    }

    private:
    std::string agcbox_imu_frame_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  };

}  // namespace r4c::localization

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r4c::localization::AgcBoxImuFilterNode>());
  rclcpp::shutdown();

  return 0;
}