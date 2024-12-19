#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "dbcppp/Message.h"
#include "dbcppp/Network.h"

#include <r4c_msgs/msg/agcbox_ens_control.hpp>
#include <r4c_msgs/msg/agcbox_gnss_cog_sog.hpp>
#include <r4c_msgs/msg/agcbox_gnss_metadata.hpp>
#include <r4c_msgs/msg/agcbox_heartbeat.hpp>
#include <r4c_msgs/msg/gbsd.hpp>
#include <r4c_msgs/msg/gnss_enu_vector.hpp>

#include "r4c_can_adapter/can_msg_model.hpp"
namespace r4c_can_adapter
{
  class Can2RosAdapter: public rclcpp::Node
  {
    public:
    Can2RosAdapter();

    protected:
    using can2ros_decoder =
      void (r4c_can_adapter::Can2RosAdapter::*)(const dbcppp::IMessage& can_msg_def,
                                                const can_msgs::msg::Frame& can_msg);

    //---------------------------------

    void add_entry_decoding_map(const uint32_t can_id, can2ros_decoder dec);

    //---------------------------------

    void decode_agcbox_ens_control_can_msg(const dbcppp::IMessage& can_msg_def,
                                           const can_msgs::msg::Frame& can_msg);

    //---------------------------------

    void decode_agcbox_ens_heartbeat_can_msg(const dbcppp::IMessage& can_msg_def,
                                             const can_msgs::msg::Frame& can_msg);

    //---------------------------------

    void decode_agcbox_gnss_cog_sog_can_msg(const dbcppp::IMessage& can_msg_def,
                                            const can_msgs::msg::Frame& can_msg);

    // --------------------------------

    void decode_agcbox_gnss_e_pos_can_msg(const dbcppp::IMessage& can_msg_def,
                                          const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_gnss_n_pos_can_msg(const dbcppp::IMessage& can_msg_def,
                                          const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_gnss_u_pos_can_msg(const dbcppp::IMessage& can_msg_def,
                                          const can_msgs::msg::Frame& can_msg);

    void decode_enu_pos(const dbcppp::IMessage& can_msg_def,
                        const can_msgs::msg::Frame& can_msg,
                        double& axis_pos);

    //---------------------------------

    void decode_agcbox_gnss_e_vel_can_msg(const dbcppp::IMessage& can_msg_def,
                                          const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_gnss_n_vel_can_msg(const dbcppp::IMessage& can_msg_def,
                                          const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_gnss_u_vel_can_msg(const dbcppp::IMessage& can_msg_def,
                                          const can_msgs::msg::Frame& can_msg);

    void decode_enu_vel(const dbcppp::IMessage& can_msg_def,
                        const can_msgs::msg::Frame& can_msg,
                        double& axis_vel);

    // --------------------------------

    void decode_agcbox_gnss_metadata_can_msg(const dbcppp::IMessage& can_msg_def,
                                             const can_msgs::msg::Frame& can_msg);

    // --------------------------------

    void decode_agcbox_gnss_wgs84_fix_can_msg(const dbcppp::IMessage& can_msg_def,
                                              const can_msgs::msg::Frame& can_msg);

    // --------------------------------

    void decode_agcbox_heading_can_msg(const dbcppp::IMessage& can_msg_def,
                                       const can_msgs::msg::Frame& can_msg);

    // --------------------------------

    void decode_agcbox_imu_angular_x_can_msg(const dbcppp::IMessage& can_msg_def,
                                             const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_imu_angular_y_can_msg(const dbcppp::IMessage& can_msg_def,
                                             const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_imu_angular_z_can_msg(const dbcppp::IMessage& can_msg_def,
                                             const can_msgs::msg::Frame& can_msg);

    void decode_imu_angular_data(const dbcppp::IMessage& can_msg_def,
                                 const can_msgs::msg::Frame& can_msg,
                                 double& axis_angle,
                                 double& axis_angular_vel);

    void decode_agcbox_imu_linear_x_can_msg(const dbcppp::IMessage& can_msg_def,
                                            const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_imu_linear_y_can_msg(const dbcppp::IMessage& can_msg_def,
                                            const can_msgs::msg::Frame& can_msg);

    void decode_agcbox_imu_linear_z_can_msg(const dbcppp::IMessage& can_msg_def,
                                            const can_msgs::msg::Frame& can_msg);

    void decode_imu_linear_data(const dbcppp::IMessage& can_msg_def,
                                const can_msgs::msg::Frame& can_msg,
                                double& axis_linear_acceleration);

    // --------------------------------

    void decode_mw_tim_can_msg(const dbcppp::IMessage& can_msg_def,
                               const can_msgs::msg::Frame& can_msg);

    // --------------------------------

    void init_decoding_map();

    // --------------------------------

    void publish_null_twist();

    // --------------------------------

    virtual void rcv_can_msg(const can_msgs::msg::Frame::ConstSharedPtr can_msg) = 0;

    // --------------------------------

    std::unique_ptr<dbcppp::INetwork> dbc_;

    std::unordered_map<uint32_t, std::pair<can2ros_decoder, const dbcppp::IMessage*>> decoding_map_;

    std::string agcbox_base_frame_;
    std::string agcbox_gnss_frame_;
    std::string agcbox_imu_frame_;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_msg_sub_;

    rclcpp::Publisher<r4c_msgs::msg::AgcboxEnsControl>::SharedPtr agcbox_ens_control_pub_;

    rclcpp::Publisher<r4c_msgs::msg::AgcboxHeartbeat>::SharedPtr agcbox_ens_heartbeat_pub_;

    rclcpp::Publisher<r4c_msgs::msg::AgcboxGnssCogSog>::SharedPtr agcbox_gnss_cog_sog_pub_;

    rclcpp::Publisher<r4c_msgs::msg::GnssEnuVector>::SharedPtr agcbox_gnss_enu_pos_pub_;
    r4c_msgs::msg::GnssEnuVector agcbox_gnss_enu_pos_msg_;

    rclcpp::Publisher<r4c_msgs::msg::GnssEnuVector>::SharedPtr agcbox_gnss_enu_vel_pub_;
    r4c_msgs::msg::GnssEnuVector agcbox_gnss_enu_vel_msg_;

    rclcpp::Publisher<r4c_msgs::msg::AgcboxGnssMetadata>::SharedPtr agcbox_gnss_metadata_pub_;
    r4c_msgs::msg::AgcboxGnssMetadata agcbox_gnss_metadata_msg_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr agcbox_gnss_wgs84_fix_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr agcbox_heading_imu_pub_;
    double wgnss84_position_coordinate_variance_;
    double heading_imu_yaw_variance_;
    double heading_default_value_;
    bool heading_reception_active_{false};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr agcbox_imu_pub_;
    sensor_msgs::msg::Imu agcbox_imu_msg_;
    geometry_msgs::msg::Vector3 agcbox_imu_rpy_;

    rclcpp::Publisher<r4c_msgs::msg::Gbsd>::SharedPtr gbsd_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mw_soft_stop_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mw_twist_recommendation_pub_;

    rclcpp::TimerBase::SharedPtr mw_soft_stop_timer_{nullptr};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    enum CAN_MSG_ID
    {
      // Messages received from the can bus.
      AGCBOX_ENS_CONTROL         = 529,
      AGCBOX_ENS_HEARTBEAT       = 1809,
      AGCBOX_GNSS_COG_SOG        = 2314732060,  // 0x89F8021C
      AGCBOX_GNSS_EAST_POSITION  = 399,
      AGCBOX_GNSS_NORTH_POSITION = 655,
      AGCBOX_GNSS_UP_POSITION    = 911,
      AGCBOX_GNSS_EAST_VELOCITY  = 1551,
      AGCBOX_GNSS_NORTH_VELOCITY = 1679,
      AGCBOX_GNSS_METADATA       = 2583168284,  // 0x99F8051C

      // old id = 2314731804, 0x89F8011C
      // new id = 2314731788, 0x89F8010C
      AGCBOX_GNSS_WGS84    = 2314731788,
      AGCBOX_HEADING       = 914,
      AGCBOX_IMU_ANGULAR_X = 1167,
      AGCBOX_IMU_ANGULAR_Y = 1423,
      AGCBOX_IMU_ANGULAR_Z = 527,
      AGCBOX_IMU_LINEAR_X  = 783,
      AGCBOX_IMU_LINEAR_Y  = 1039,
      AGCBOX_IMU_LINEAR_Z  = 1295,
      MW_TIM_SPRAYER       = 2619670516,  // 0x9C24FFF4
    };

    const std::unordered_map<uint64_t, CanMsgModel> dbc_model_{
      {CAN_MSG_ID::AGCBOX_ENS_CONTROL,
       {CAN_MSG_ID::AGCBOX_ENS_CONTROL,
        "ENSControl",
        {"SpeedLimitationENS", "GoToAuto", "MotionAuthorized"}}},
      {CAN_MSG_ID::AGCBOX_ENS_HEARTBEAT,
       {CAN_MSG_ID::AGCBOX_ENS_HEARTBEAT, "HeartbeatENS", {"CANOpenStatusENS"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_COG_SOG,
       {CAN_MSG_ID::AGCBOX_GNSS_COG_SOG,
        "CogSog",
        {"SpeedOverGround", "CourseOverGround", "CogReference", "SID"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_EAST_POSITION,
       {CAN_MSG_ID::AGCBOX_GNSS_EAST_POSITION,
        "GNSSPositionEast",
        {"GNSSPositionEast", "GNSSTimeStamp", "ValidityFlag"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_NORTH_POSITION,
       {CAN_MSG_ID::AGCBOX_GNSS_NORTH_POSITION,
        "GNSSPositionNorth",
        {"GNSSPositionNorth", "GNSSTimeStamp", "ValidityFlag"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_UP_POSITION,
       {CAN_MSG_ID::AGCBOX_GNSS_UP_POSITION,
        "GNSSPositionUp",
        {"GNSSPositionUp", "GNSSTimeStamp", "ValidityFlag"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_EAST_VELOCITY,
       {CAN_MSG_ID::AGCBOX_GNSS_EAST_VELOCITY,
        "GNSSVelocityEast",
        {"GNSSVelocityEast", "GNSSTimeStamp", "ValidityFlag"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_NORTH_VELOCITY,
       {CAN_MSG_ID::AGCBOX_GNSS_NORTH_VELOCITY,
        "GNSSVelocityNorth",
        {"GNSSVelocityNorth", "GNSSTimeStamp", "ValidityFlag"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_METADATA,
       {CAN_MSG_ID::AGCBOX_GNSS_METADATA,
        "GnssPosData",
        {"GnssMethod",
         "ReferenceStationID1",
         "NumberOfReferenceStations",
         "PDOP",
         "HDOP",
         "NumberOfSVs"}}},
      {CAN_MSG_ID::AGCBOX_GNSS_WGS84,
       {CAN_MSG_ID::AGCBOX_GNSS_WGS84, "GnssWgs84", {"Longitude", "Latitude"}}},
      {CAN_MSG_ID::AGCBOX_HEADING,
       {CAN_MSG_ID::AGCBOX_HEADING, "StatePsi", {"Psi", "StateTickTime"}}},
      {CAN_MSG_ID::AGCBOX_IMU_ANGULAR_X,
       {CAN_MSG_ID::AGCBOX_IMU_ANGULAR_X, "IMUAnglesX", {"IMURotX", "IMURoll", "IMUTimestamp"}}},
      {CAN_MSG_ID::AGCBOX_IMU_ANGULAR_Y,
       {CAN_MSG_ID::AGCBOX_IMU_ANGULAR_Y, "IMUAnglesY", {"IMURotY", "IMUPitch", "IMUTimestamp"}}},
      {CAN_MSG_ID::AGCBOX_IMU_ANGULAR_Z,
       {CAN_MSG_ID::AGCBOX_IMU_ANGULAR_Z, "IMUAnglesZ", {"IMURotZ", "IMUYaw", "IMUTimestamp"}}},
      {CAN_MSG_ID::AGCBOX_IMU_LINEAR_X,
       {CAN_MSG_ID::AGCBOX_IMU_LINEAR_X, "IMUAccX", {"IMUAccX", "IMUDVX", "IMUTimestamp"}}},
      {CAN_MSG_ID::AGCBOX_IMU_LINEAR_Y,
       {CAN_MSG_ID::AGCBOX_IMU_LINEAR_Y, "IMUAccY", {"IMUAccY", "IMUDVY", "IMUTimestamp"}}},
      {CAN_MSG_ID::AGCBOX_IMU_LINEAR_Z,
       {CAN_MSG_ID::AGCBOX_IMU_LINEAR_Z, "IMUAccZ", {"IMUAccZ", "IMUDVZ", "IMUTimestamp"}}},
      {CAN_MSG_ID::MW_TIM_SPRAYER,
       {CAN_MSG_ID::MW_TIM_SPRAYER,
        "TimSprayer",
        {"VehicleSpeedRequest", "PTOSpeedRequest", "Process_Malfunction", "AuxValveFlowReq"}}}};
  };

}  // namespace r4c_can_adapter
