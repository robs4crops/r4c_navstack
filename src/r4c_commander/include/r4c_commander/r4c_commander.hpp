#ifndef R4C_COMMANDER_HPP_
#define R4C_COMMANDER_HPP_

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "r4c_interfaces/action/nav.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/path.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "geographic_msgs/msg/geo_point.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "robot_localization/srv/from_ll.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_2d_utils/conversions.hpp>

#include <r4c_msgs/msg/nav_task.hpp>
#include <r4c_msgs/msg/nav_mission.hpp>
#include <r4c_interfaces/msg/tractor_status.hpp>
#include <r4c_interfaces/msg/localization_status.hpp>
#include <r4c_interfaces/msg/tractor_status.hpp>

#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "std_srvs/srv/trigger.hpp"


namespace r4c_commander
{
  class R4CCommander : public rclcpp::Node {
  public:
      explicit R4CCommander(const std::string& node_name);

  private:
      std::string mission_server_name_, navsat_service_name_, nav2_client_name_;
      std::string inrow_bt_name_, maneuver_bt_name_, free_bt_name_;
      std::string mission_frame_;
      std::string r4c_nav_pkg_share_dir_;

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr zero_nav_cmd_pub_;

      bool nav_task_finished_ = false, nav_task_failed_ = false;

      bool is_executing_ = false;

      bool status_ok_ = false;

      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_sub_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
      rclcpp::Subscription<r4c_interfaces::msg::TractorStatus>::SharedPtr status_sub_;
      rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr navsat_transform_client_;

      void navsatfix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr navsatfix_msg);
      void velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr velocity_msg);
      void status_callback(const r4c_interfaces::msg::TractorStatus::ConstSharedPtr status_msg);

      rclcpp_action::Server<r4c_interfaces::action::Nav>::SharedPtr action_server_;
      rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const r4c_interfaces::action::Nav::Goal> goal);
      rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<r4c_interfaces::action::Nav>> goal_handle);
      void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<r4c_interfaces::action::Nav>> goal_handle);
      void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<r4c_interfaces::action::Nav>> goal_handle);

      rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav2_client_;
      void nav2_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr> future);
      void nav2_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
      void nav2_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);

      geometry_msgs::msg::Point getMapPoint(double lat, double lon);
      double calculateTheta(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2);
      std::vector<geometry_msgs::msg::PoseStamped> getMissionPoses(const nav_msgs::msg::Path& path_msg);
      r4c_msgs::msg::NavTask createNavTask(const geometry_msgs::msg::PoseStamped& pose_a, const geometry_msgs::msg::PoseStamped& pose_b, const geometry_msgs::msg::PoseStamped& pose_c, const geometry_msgs::msg::PoseStamped& pose_d, int type) ;
      r4c_msgs::msg::NavMission getMission(const std::vector<geometry_msgs::msg::PoseStamped>& poses);

      r4c_interfaces::action::Nav::Feedback::SharedPtr mission_feedback_;
      r4c_interfaces::action::Nav::Result::SharedPtr mission_result_;

      geometry_msgs::msg::Twist zero_nav_cmd_vel;

      bool start_mission_ = false, pause_mission_ = true, stop_mission_ = false;

      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_srv_, pause_mission_srv_, stop_mission_srv_;

      bool startMissionCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
      bool pauseMissionCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
      bool stopMissionCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

      void timerCallback();

  };

} //namespace r4c_commander

#endif // R4C_COMMANDER_HPP_
