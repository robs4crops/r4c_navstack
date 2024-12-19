#include <chrono>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <angles/angles.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "r4c_can_adapter/dbc_model_checker.hpp"
#include "r4c_can_adapter/can2ros_adapter.hpp"

namespace r4c_can_adapter
{
  Can2RosAdapter::Can2RosAdapter():
    rclcpp::Node{"can2ros_adapter"},
    agcbox_base_frame_{this->declare_parameter("agcbox_base_frame", "agcbox_base_link")},
    agcbox_gnss_frame_{this->declare_parameter("agcbox_gnss_frame", "agcbox_gnss_link")},
    agcbox_imu_frame_{this->declare_parameter("agcbox_imu_frame", "agcbox_imu_link")},
    can_msg_sub_{this->create_subscription<can_msgs::msg::Frame>(
      this->declare_parameter("input_can_msg_topic", "can_msg_from_can_bus"),
      10,
      std::bind(&Can2RosAdapter::rcv_can_msg, this, std::placeholders::_1))},
    agcbox_ens_control_pub_{this->create_publisher<r4c_msgs::msg::AgcboxEnsControl>(
      this->declare_parameter("agcbox_ens_control_topic", "agcbox/ens/control"),
      10)},
    agcbox_ens_heartbeat_pub_{this->create_publisher<r4c_msgs::msg::AgcboxHeartbeat>(
      this->declare_parameter("agcbox_ens_heartbeat_topic", "agcbox/ens/heartbeat"),
      10)},
    agcbox_gnss_cog_sog_pub_{this->create_publisher<r4c_msgs::msg::AgcboxGnssCogSog>(
      this->declare_parameter("agcbox_gnss_cog_sog_topic", "agcbox/gnss/cog_sog"),
      10)},
    agcbox_gnss_enu_pos_pub_{this->create_publisher<r4c_msgs::msg::GnssEnuVector>(
      this->declare_parameter("agcbox_gnss_enu_pos_topic", "agcbox/gnss/enu/pos"),
      10)},
    agcbox_gnss_enu_vel_pub_{this->create_publisher<r4c_msgs::msg::GnssEnuVector>(
      this->declare_parameter("agcbox_gnss_enu_vel_topic", "agcbox/gnss/enu/vel"),
      10)},
    agcbox_gnss_metadata_pub_{this->create_publisher<r4c_msgs::msg::AgcboxGnssMetadata>(
      this->declare_parameter("agcbox_gnss_metadata_topic", "agcbox/gnss/metadata"),
      10)},
    agcbox_gnss_wgs84_fix_pub_{this->create_publisher<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("agcbox_gnss_wgs84_fix_topic", "agcbox/gnss/wgs84_fix"),
      10)},
    agcbox_heading_imu_pub_{this->create_publisher<sensor_msgs::msg::Imu>(
      this->declare_parameter("agcbox_heading_topic", "agcbox/heading_imu"),
      10)},
    wgnss84_position_coordinate_variance_{
      this->declare_parameter("wgnss84_position_coordinate_variance", 0.0001)},
    heading_imu_yaw_variance_{this->declare_parameter("heading_imu_yaw_variance", 0.0001)},
    heading_default_value_{this->declare_parameter("heading_default_value", M_PI_2)},
    agcbox_imu_pub_{this->create_publisher<sensor_msgs::msg::Imu>(
      this->declare_parameter("agcbox_imu_topic", "agcbox/imu"),
      10)},
    gbsd_pub_{
      this->create_publisher<r4c_msgs::msg::Gbsd>(this->declare_parameter("gbsd_topic", "gbsd"),
                                                  10)},
    mw_soft_stop_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
      this->declare_parameter("mw_soft_stop_topic", "mw/soft_stop"),
      10)},
    mw_twist_recommendation_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
      this->declare_parameter("mw_twist_recommendation_topic", "mw/vel_recommendation"),
      10)},
    twist_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
      this->declare_parameter("twist_topic", "vel"),
      10)}
  {
    const auto dbc_file = declare_parameter("dbc_file", "");

    if(!std::filesystem::exists(dbc_file))
    {
      constexpr std::string_view str_1{"The file '"};
      constexpr std::string_view str_2{"' doest not exist in disk"};
      constexpr auto size = str_1.size() + str_2.size();

      std::string message;
      message.reserve(size + dbc_file.length());
      message.append(str_1).append(dbc_file).append(str_2);

      throw std::invalid_argument{message};
    }

    if(std::filesystem::is_empty(dbc_file))
    {
      constexpr std::string_view str_1{"The file '"};
      constexpr std::string_view str_2{"' is empty"};
      constexpr auto size = str_1.size() + str_2.size();

      std::string message;
      message.reserve(size + dbc_file.length());
      message.append(str_1).append(dbc_file).append(str_2);

      throw std::invalid_argument{message};
    }

    std::ifstream dbc_fs{dbc_file};
    dbc_ = dbcppp::INetwork::LoadDBCFromIs(dbc_fs);

    DbcModelChecker dbc_model_checker{dbc_model_, *dbc_};

    init_decoding_map();

    agcbox_gnss_enu_pos_msg_.header.frame_id  = agcbox_gnss_frame_;
    agcbox_gnss_enu_vel_msg_.header.frame_id  = agcbox_gnss_frame_;
    agcbox_gnss_metadata_msg_.header.frame_id = agcbox_gnss_frame_;
    agcbox_imu_msg_.header.frame_id           = agcbox_imu_frame_;
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::add_entry_decoding_map(const uint32_t can_id, can2ros_decoder dec)
  {
    auto predicate = [&can_id](const dbcppp::IMessage& imessage) {
      return imessage.Id() == can_id;
    };

    auto dbc_iterable = dbc_->Messages();

    auto can_msg_def_it = std::find_if(dbc_iterable.begin(), dbc_iterable.end(), predicate);

    if(can_msg_def_it != dbc_iterable.end())
    {
      const auto& can_msg_def = *can_msg_def_it;
      // can_ids have 11 bits in standar messages and 29 bits in extended messages.
      // Truncate the can_ids to 29 bits at most. Mask for 29 bits: 0x1FFFFFFF.
      decoding_map_.insert({can_id & 0x1FFFFFFF, {dec, &can_msg_def}});
    }
    else
    {
      std::string can_id_str = std::to_string(can_id);
      constexpr std::string_view str_1{"Can message '"};
      constexpr std::string_view str_2{"' not present in dbc file"};
      constexpr auto size = str_1.size() + str_2.size();

      std::string msg;
      msg.reserve(size + can_id_str.size());
      msg.append(str_1).append(can_id_str).append(str_2);

      throw std::runtime_error{msg};
    }
  }
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_ens_control_can_msg(const dbcppp::IMessage& can_msg_def,
                                                         const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    r4c_msgs::msg::AgcboxEnsControl agcbox_ens_control_msg;
    agcbox_ens_control_msg.header.frame_id = agcbox_base_frame_;
    agcbox_ens_control_msg.header.stamp    = can_msg.header.stamp;

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "MotionAuthorized")
      {
        agcbox_ens_control_msg.motion_authorized = static_cast<bool>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name == "GoToAuto")
      {
        agcbox_ens_control_msg.go_to_auto = static_cast<bool>(sig.RawToPhys(sig.Decode(raw_data)));
      }
      else  // SpeedLimitationENS
      {
        agcbox_ens_control_msg.velocity_limitation = static_cast<uint8_t>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
    }

    agcbox_ens_control_pub_->publish(agcbox_ens_control_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_ens_heartbeat_can_msg(const dbcppp::IMessage& can_msg_def,
                                                           const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    // The message 1809 HeartbeatSpe only has one signal, CanopenStatusEns.
    const auto& sig = can_msg_def.Signals_Get(0);

    r4c_msgs::msg::AgcboxHeartbeat agcbox_ens_heartbeat_msg;
    agcbox_ens_heartbeat_msg.header.frame_id = agcbox_base_frame_;
    agcbox_ens_heartbeat_msg.header.stamp    = can_msg.header.stamp;
    agcbox_ens_heartbeat_msg.status = static_cast<uint8_t>(sig.RawToPhys(sig.Decode(raw_data)));

    agcbox_ens_heartbeat_pub_->publish(agcbox_ens_heartbeat_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_cog_sog_can_msg(const dbcppp::IMessage& can_msg_def,
                                                          const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    r4c_msgs::msg::AgcboxGnssCogSog agcbox_gnss_cog_sog_msg;
    agcbox_gnss_cog_sog_msg.header.frame_id = agcbox_gnss_frame_;
    agcbox_gnss_cog_sog_msg.header.stamp    = can_msg.header.stamp;

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "SID")
      {
        agcbox_gnss_cog_sog_msg.sequence_id = static_cast<uint8_t>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name == "CogReference")
      {
        agcbox_gnss_cog_sog_msg.cog_reference = static_cast<uint8_t>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name == "CourseOverGround")
      {
        agcbox_gnss_cog_sog_msg.course_over_ground = sig.RawToPhys(sig.Decode(raw_data));
      }
      else  // SpeedOverGround
      {
        agcbox_gnss_cog_sog_msg.speed_over_ground = sig.RawToPhys(sig.Decode(raw_data));
      }
    }

    agcbox_gnss_cog_sog_pub_->publish(agcbox_gnss_cog_sog_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_e_pos_can_msg(const dbcppp::IMessage& can_msg_def,
                                                        const can_msgs::msg::Frame& can_msg)
  {
    decode_enu_pos(can_msg_def, can_msg, agcbox_gnss_enu_pos_msg_.e);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_n_pos_can_msg(const dbcppp::IMessage& can_msg_def,
                                                        const can_msgs::msg::Frame& can_msg)
  {
    decode_enu_pos(can_msg_def, can_msg, agcbox_gnss_enu_pos_msg_.n);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_u_pos_can_msg(const dbcppp::IMessage& can_msg_def,
                                                        const can_msgs::msg::Frame& can_msg)
  {
    decode_enu_pos(can_msg_def, can_msg, agcbox_gnss_enu_pos_msg_.u);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_enu_pos(const dbcppp::IMessage& can_msg_def,
                                      const can_msgs::msg::Frame& can_msg,
                                      double& axis_pos)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs_size       = can_msg_def.Signals_Size();

    bool validity{false};
    double tstamp{0.0};

    for(size_t i{0}; i < sigs_size; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "ValidityFlag")
      {
        validity = static_cast<bool>(sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name == "GNSSTimeStamp")
      {
        tstamp = sig.RawToPhys(sig.Decode(raw_data)) * 0.001;  // ms to s
      }
      else  //  "GNSSPosition[East|North|Up]"
      {
        axis_pos = sig.RawToPhys(sig.Decode(raw_data));  // In m
      }
    }

    if(!validity)
    {
      axis_pos = std::numeric_limits<double>::quiet_NaN();
    }

    agcbox_gnss_enu_pos_msg_.header.stamp = can_msg.header.stamp;

    auto secs_integral_part                     = std::floor(tstamp);
    agcbox_gnss_enu_pos_msg_.gnss_stamp.sec     = static_cast<int32_t>(secs_integral_part);
    agcbox_gnss_enu_pos_msg_.gnss_stamp.nanosec = static_cast<int32_t>(
      (tstamp - secs_integral_part) * 1e9);

    agcbox_gnss_enu_pos_pub_->publish(agcbox_gnss_enu_pos_msg_);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_e_vel_can_msg(const dbcppp::IMessage& can_msg_def,
                                                        const can_msgs::msg::Frame& can_msg)
  {
    decode_enu_vel(can_msg_def, can_msg, agcbox_gnss_enu_vel_msg_.e);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_n_vel_can_msg(const dbcppp::IMessage& can_msg_def,
                                                        const can_msgs::msg::Frame& can_msg)
  {
    decode_enu_vel(can_msg_def, can_msg, agcbox_gnss_enu_vel_msg_.n);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_u_vel_can_msg(const dbcppp::IMessage& can_msg_def,
                                                        const can_msgs::msg::Frame& can_msg)
  {
    decode_enu_vel(can_msg_def, can_msg, agcbox_gnss_enu_vel_msg_.u);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_enu_vel(const dbcppp::IMessage& can_msg_def,
                                      const can_msgs::msg::Frame& can_msg,
                                      double& axis_vel)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs_size       = can_msg_def.Signals_Size();

    bool validity{false};
    double tstamp{0.0};

    for(size_t i{0}; i < sigs_size; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "ValidityFlag")
      {
        validity = static_cast<bool>(sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name == "GNSSTimeStamp")
      {
        tstamp = sig.RawToPhys(sig.Decode(raw_data)) * 0.001;  // ms to s
      }
      else  // GnssVelocity{East|North}
      {
        axis_vel = sig.RawToPhys(sig.Decode(raw_data));  // IN m/s
      }
    }

    if(!validity)
    {
      axis_vel = std::numeric_limits<double>::quiet_NaN();
    }

    agcbox_gnss_enu_vel_msg_.header.stamp = can_msg.header.stamp;

    auto secs_integral_part                     = std::floor(tstamp);
    agcbox_gnss_enu_vel_msg_.gnss_stamp.sec     = static_cast<int32_t>(secs_integral_part);
    agcbox_gnss_enu_vel_msg_.gnss_stamp.nanosec = static_cast<int32_t>(
      (tstamp - secs_integral_part) * 1e9);

    agcbox_gnss_enu_vel_pub_->publish(agcbox_gnss_enu_vel_msg_);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_metadata_can_msg(const dbcppp::IMessage& can_msg_def,
                                                           const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    agcbox_gnss_metadata_msg_.header.stamp = can_msg.header.stamp;

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "NumberOfSVs")
      {
        agcbox_gnss_metadata_msg_.num_satellites = static_cast<uint8_t>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name == "HDOP")
      {
        agcbox_gnss_metadata_msg_.hdop = sig.RawToPhys(sig.Decode(raw_data));
      }
      else if(sig_name == "PDOP")
      {
        agcbox_gnss_metadata_msg_.pdop = sig.RawToPhys(sig.Decode(raw_data));
      }
      else if(sig_name == "NumberOfReferenceStations")
      {
        agcbox_gnss_metadata_msg_.num_reference_stations = static_cast<uint8_t>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
      else if(sig_name == "ReferenceStationID1")
      {
        agcbox_gnss_metadata_msg_.reference_station_id = static_cast<uint16_t>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
      else  // GnssMethod
      {
        agcbox_gnss_metadata_msg_.gnss_method = static_cast<uint8_t>(
          sig.RawToPhys(sig.Decode(raw_data)));
      }
    }

    agcbox_gnss_metadata_pub_->publish(agcbox_gnss_metadata_msg_);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_gnss_wgs84_fix_can_msg(const dbcppp::IMessage& can_msg_def,
                                                            const can_msgs::msg::Frame& can_msg)
  {
    // if(agcbox_gnss_metadata_msg_.gnss_method !=
    //      r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER &&
    //    agcbox_gnss_metadata_msg_.gnss_method != r4c_msgs::msg::AgcboxGnssMetadata::DGNSS_FIX)

    if(agcbox_gnss_metadata_msg_.gnss_method !=
       r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER)
    {
      return;
    }

    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    sensor_msgs::msg::NavSatFix agcbox_gnss_wgs84_fix_msg;
    agcbox_gnss_wgs84_fix_msg.header.frame_id = agcbox_gnss_frame_;
    agcbox_gnss_wgs84_fix_msg.header.stamp    = can_msg.header.stamp;

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "Latitude")
      {
        agcbox_gnss_wgs84_fix_msg.latitude = sig.RawToPhys(sig.Decode(raw_data));  // deg
      }
      else if(sig_name == "Longitude")
      {
        agcbox_gnss_wgs84_fix_msg.longitude = sig.RawToPhys(sig.Decode(raw_data));  // deg
      }
    }

    agcbox_gnss_wgs84_fix_msg.altitude = 0.0;

    // How to approximate the covariance from the HDOP obtained from:
    // https://github.com/ros-drivers/nmea_navsat_driver/blob/indigo-devel/src/libnmea_navsat_driver/driver.py#L110-L114
    // agcbox_gnss_wgs84_fix_msg.position_covariance[0] = agcbox_gnss_metadata_msg_.hdop *
    //                                                    agcbox_gnss_metadata_msg_.hdop;
    // agcbox_gnss_wgs84_fix_msg.position_covariance[4] = agcbox_gnss_wgs84_fix_msg
    //                                                      .position_covariance[0];
    // auto double_hdop = (2.0 * agcbox_gnss_metadata_msg_.hdop);
    // agcbox_gnss_wgs84_fix_msg.position_covariance[8] = double_hdop * double_hdop;

    agcbox_gnss_wgs84_fix_msg.position_covariance[0] = wgnss84_position_coordinate_variance_;
    agcbox_gnss_wgs84_fix_msg.position_covariance[4] = wgnss84_position_coordinate_variance_;
    agcbox_gnss_wgs84_fix_msg.position_covariance[8] = wgnss84_position_coordinate_variance_;

    agcbox_gnss_wgs84_fix_msg
      .position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    agcbox_gnss_wgs84_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

    // 0 because we do not know which service (BEIDOU, GALILEO, GPS, GLONASS) AGC is using.
    agcbox_gnss_wgs84_fix_msg.status.service = 0;

    agcbox_gnss_wgs84_fix_pub_->publish(agcbox_gnss_wgs84_fix_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_heading_can_msg(const dbcppp::IMessage& can_msg_def,
                                                     const can_msgs::msg::Frame& can_msg)
  {
    // If gnss method is not a 'RTK FIXED', then the heading is not publish.
    // This behavior is also applied when publishing of the WGS84 GNSS solution.

    if(agcbox_gnss_metadata_msg_.gnss_method !=
       r4c_msgs::msg::AgcboxGnssMetadata::RTK_FIXED_INTEGER)
    {
      return;
    }

    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    double heading{heading_default_value_};

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "Psi")
      {
        heading = sig.RawToPhys(sig.Decode(raw_data));
        // Heading coming from AGCBox is referenced to North.
        // By adding PI/2 the new heading is referenced with respect east, which is
        // the standard followed by ROS (REP 105 -- Coordinate Frames for Mobile Platforms).
        heading = -heading + M_PI_2;

        break;
      }
    }

    // When the AGCBox switches on and it has not computed the heading yet, likey because the
    // vehicle has not moved yet, it provides a default value of 'heading_default_value_'.
    // To publish the heading first we impose the condition of receiving an RTK fix.
    // Next, we need a way to verify that the received heading is valid, i.e., we are not receiving
    // the default value. For that matter, while we are in a state in which we consider the
    // reception of valid headings has not started yet ('heading_reception_active_ = false), we
    // check if the received heading is a little different from the default value. If that is the
    // case we consider we started receiving valid headings.
    // There is always the chance that the vehicle, by coincidende, is aligned in the real
    // environment in the orientation of 'heading_default_value_'. In that case moving the vehicle
    // a little bit will solve the issue, since during the motion it'll probably exceed that
    // threshold.
    constexpr double threshold{(1.0 / 180.0) * M_PI};  // 1 degree

    if(!heading_reception_active_ && std::abs(heading - heading_default_value_) < threshold)
    {
      return;
    }

    heading_reception_active_ = true;

    heading = angles::normalize_angle(heading);

    sensor_msgs::msg::Imu agcbox_heading_imu_msg;
    agcbox_heading_imu_msg.header.frame_id = agcbox_imu_frame_;
    agcbox_heading_imu_msg.header.stamp    = can_msg.header.stamp;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading);

    agcbox_heading_imu_msg.orientation               = tf2::toMsg(q);
    agcbox_heading_imu_msg.orientation_covariance[8] = heading_imu_yaw_variance_;

    agcbox_heading_imu_pub_->publish(agcbox_heading_imu_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_imu_angular_x_can_msg(const dbcppp::IMessage& can_msg_def,
                                                           const can_msgs::msg::Frame& can_msg)
  {
    decode_imu_angular_data(can_msg_def,
                            can_msg,
                            agcbox_imu_rpy_.x,
                            agcbox_imu_msg_.angular_velocity.x);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_imu_angular_y_can_msg(const dbcppp::IMessage& can_msg_def,
                                                           const can_msgs::msg::Frame& can_msg)
  {
    decode_imu_angular_data(can_msg_def,
                            can_msg,
                            agcbox_imu_rpy_.y,
                            agcbox_imu_msg_.angular_velocity.y);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_imu_angular_z_can_msg(const dbcppp::IMessage& can_msg_def,
                                                           const can_msgs::msg::Frame& can_msg)
  {
    decode_imu_angular_data(can_msg_def,
                            can_msg,
                            agcbox_imu_rpy_.z,
                            agcbox_imu_msg_.angular_velocity.z);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_imu_angular_data(const dbcppp::IMessage& can_msg_def,
                                               const can_msgs::msg::Frame& can_msg,
                                               double& axis_angle,
                                               double& axis_angular_vel)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    [[maybe_unused]] double tstamp{0.0};

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "IMUTimestamp")
      {
        tstamp = sig.RawToPhys(sig.Decode(raw_data)) * 0.001;  // ms to s
      }
      else if(0 == sig_name.compare(0, 6, "IMURot"))  // IMURot{X|Y|Z}
      {
        axis_angular_vel = sig.RawToPhys(sig.Decode(raw_data));  // In rad/s
      }
      else  // IMURoll, IMUPitch, IMUYaw
      {
        axis_angle = sig.RawToPhys(sig.Decode(raw_data));  // In rad
      }
    }

    agcbox_imu_msg_.header.stamp = can_msg.header.stamp;

    // auto secs_integral_part              = std::floor(tstamp);
    // agcbox_imu_msg_.header.stamp.sec     = static_cast<int32_t>(secs_integral_part);
    // agcbox_imu_msg_.header.stamp.nanosec = static_cast<int32_t>((tstamp - secs_integral_part) *
    //                                                             1e9);

    tf2::Quaternion q;
    q.setRPY(agcbox_imu_rpy_.x, agcbox_imu_rpy_.y, agcbox_imu_rpy_.z);
    agcbox_imu_msg_.orientation = tf2::toMsg(q);

    agcbox_imu_pub_->publish(agcbox_imu_msg_);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_imu_linear_x_can_msg(const dbcppp::IMessage& can_msg_def,
                                                          const can_msgs::msg::Frame& can_msg)
  {
    decode_imu_linear_data(can_msg_def, can_msg, agcbox_imu_msg_.linear_acceleration.x);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_imu_linear_y_can_msg(const dbcppp::IMessage& can_msg_def,
                                                          const can_msgs::msg::Frame& can_msg)
  {
    decode_imu_linear_data(can_msg_def, can_msg, agcbox_imu_msg_.linear_acceleration.y);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_agcbox_imu_linear_z_can_msg(const dbcppp::IMessage& can_msg_def,
                                                          const can_msgs::msg::Frame& can_msg)
  {
    decode_imu_linear_data(can_msg_def, can_msg, agcbox_imu_msg_.linear_acceleration.z);
  }
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_imu_linear_data(const dbcppp::IMessage& can_msg_def,
                                              const can_msgs::msg::Frame& can_msg,
                                              double& axis_linear_acceleration)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

    [[maybe_unused]] double tstamp{0.0};
    [[maybe_unused]] double dv{0.0};  // I do not know what is this. Ask AGC.

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

      if(sig_name == "IMUTimestamp")
      {
        tstamp = sig.RawToPhys(sig.Decode(raw_data)) * 0.001;  // ms to s
      }
      else if(0 == sig_name.compare(0, 5, "IMUDV"))  // IMUDV{X|Y|Z}
      {
        dv = sig.RawToPhys(sig.Decode(raw_data));  // In m/s
      }
      else  //  IMUAcc{X|Y|Z}
      {
        axis_linear_acceleration = sig.RawToPhys(sig.Decode(raw_data));  // In m/s^2
      }
    }

    agcbox_imu_msg_.header.stamp = can_msg.header.stamp;

    // auto secs_integral_part              = std::floor(tstamp);
    // agcbox_imu_msg_.header.stamp.sec     = static_cast<int32_t>(secs_integral_part);
    // agcbox_imu_msg_.header.stamp.nanosec = static_cast<int32_t>((tstamp - secs_integral_part) *
    //                                                             1e9);

    agcbox_imu_pub_->publish(agcbox_imu_msg_);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::decode_mw_tim_can_msg(const dbcppp::IMessage& can_msg_def,
                                             const can_msgs::msg::Frame& can_msg)
  {
    const auto* raw_data = can_msg.data.data();
    auto sigs            = can_msg_def.Signals_Size();

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
    RCLCPP_WARN(this->get_logger(), "[%d] dlc: %d.", can_msg->id, can_msg->dlc);

    for(int i{0}; i < 8; i++)
    {
      RCLCPP_WARN(this->get_logger(), "%d - %X.", i, raw_data[i]);
    }
#endif

    for(size_t i{0}; i < sigs; ++i)
    {
      const auto& sig      = can_msg_def.Signals_Get(i);
      const auto& sig_name = sig.Name();

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG

      RCLCPP_WARN(this->get_logger(), "[%d] %s.", can_msg->id, sig_name.c_str());
#endif

      if(sig_name == "VehicleSpeedRequest")
      {
        // If the timer pointer is not nullptr, then we are publishing a soft stop, so we do not
        // have to process the velocity recommendation. The velocity recommendation should be zero
        // while the process_malfuntion signal is on, but in anyways by not processing the velocity
        // we are good.
        if(mw_soft_stop_timer_)
        {
          continue;
        }

        auto linear_vel_rec = sig.RawToPhys(sig.Decode(raw_data));

        // If the velocity recommendation is 0.0 m/s (or negative) then do not use this value as a
        // recommendation and that 0.0 m/s is transformed into a nan.
        if(linear_vel_rec > std::numeric_limits<double>::epsilon())
        {
          geometry_msgs::msg::Twist mw_twist_recommendation;
          mw_twist_recommendation.linear.x = linear_vel_rec;

          mw_twist_recommendation_pub_->publish(mw_twist_recommendation);
        }

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG

        RCLCPP_WARN(this->get_logger(), "[%d] %X %X", can_msg->id, raw_data[0], raw_data[1]);
        RCLCPP_WARN(this->get_logger(),
                    "[%d] linear vel recommendation: %7.3f.",
                    can_msg->id,
                    twist_.twist.linear.x);
#endif
      }
      else if(sig_name == "Process_Malfunction")
      {
        int process_malfunction = static_cast<int>(sig.RawToPhys(sig.Decode(raw_data)));

        // Do the body of the if-block only if we the process_malfunction signal is on and the timer
        // (that manages the publication of the soft_stop command) does not exist.
        if(process_malfunction == 1 && !mw_soft_stop_timer_)
        {
          mw_soft_stop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&Can2RosAdapter::publish_null_twist, this));
        }
        // Do the body of the else-if-block only if we the process_malfunction signal is off and the
        // timer (that manages the publication of the soft_stop command) does exist.
        else if(process_malfunction != 1 && mw_soft_stop_timer_)
        {
          mw_soft_stop_timer_->cancel();
          mw_soft_stop_timer_.reset();
          mw_soft_stop_timer_ = nullptr;
        }

#ifdef R4C_CAN_ADAPTER_EXTRA_LOG
        RCLCPP_WARN(this->get_logger(), "[%d] %X %X", can_msg->id, raw_data[2], raw_data[3]);
        RCLCPP_WARN(this->get_logger(),
                    "[%d] critical event: %d.",
                    can_msg->id,
                    middleware_soft_stop_request.data);
#endif
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::init_decoding_map()
  {
    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_ENS_CONTROL,
                           &Can2RosAdapter::decode_agcbox_ens_control_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_ENS_HEARTBEAT,
                           &Can2RosAdapter::decode_agcbox_ens_heartbeat_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_COG_SOG,
                           &Can2RosAdapter::decode_agcbox_gnss_cog_sog_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_EAST_POSITION,
                           &Can2RosAdapter::decode_agcbox_gnss_e_pos_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_NORTH_POSITION,
                           &Can2RosAdapter::decode_agcbox_gnss_n_pos_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_UP_POSITION,
                           &Can2RosAdapter::decode_agcbox_gnss_u_pos_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_EAST_VELOCITY,
                           &Can2RosAdapter::decode_agcbox_gnss_e_vel_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_NORTH_VELOCITY,
                           &Can2RosAdapter::decode_agcbox_gnss_n_vel_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_METADATA,
                           &Can2RosAdapter::decode_agcbox_gnss_metadata_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_GNSS_WGS84,
                           &Can2RosAdapter::decode_agcbox_gnss_wgs84_fix_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_HEADING,
                           &Can2RosAdapter::decode_agcbox_heading_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_IMU_ANGULAR_X,
                           &Can2RosAdapter::decode_agcbox_imu_angular_x_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_IMU_ANGULAR_Y,
                           &Can2RosAdapter::decode_agcbox_imu_angular_y_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_IMU_ANGULAR_Z,
                           &Can2RosAdapter::decode_agcbox_imu_angular_z_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_IMU_LINEAR_X,
                           &Can2RosAdapter::decode_agcbox_imu_linear_x_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_IMU_LINEAR_Y,
                           &Can2RosAdapter::decode_agcbox_imu_linear_y_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::AGCBOX_IMU_LINEAR_Z,
                           &Can2RosAdapter::decode_agcbox_imu_linear_z_can_msg);

    add_entry_decoding_map(CAN_MSG_ID::MW_TIM_SPRAYER, &Can2RosAdapter::decode_mw_tim_can_msg);
  }

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  void Can2RosAdapter::publish_null_twist()
  {
    geometry_msgs::msg::Twist mw_soft_stop;
    mw_soft_stop.linear.x  = 0.0;
    mw_soft_stop.angular.z = 0.0;

    mw_soft_stop_pub_->publish(mw_soft_stop);
  }
}  // namespace r4c_can_adapter
