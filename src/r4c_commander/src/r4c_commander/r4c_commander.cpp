#include "r4c_commander/r4c_commander.hpp"
using namespace std::chrono_literals;

namespace r4c_commander
{
  R4CCommander::R4CCommander(const std::string& node_name):
    rclcpp::Node(node_name),
    mission_server_name_{this->declare_parameter("mission_server_name", "/mission")},
    navsat_service_name_{this->declare_parameter("navsat_service_name", "/navsat")},
    nav2_client_name_{this->declare_parameter("nav2_client_name", "/navigate")},
    inrow_bt_name_{this->declare_parameter("inrow_bt_name", "inrow.xml")},
    maneuver_bt_name_{this->declare_parameter("maneuver_bt_name", "maneuver.xml")},
    free_bt_name_{this->declare_parameter("free_bt_name", "free.xml")},
    mission_frame_{this->declare_parameter("mission_frame", "map")},
    zero_nav_cmd_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
      this->declare_parameter("navigation_zero_speed_topic", "/r4c_tractor/zero_nav_cmd_vel"),
      10)},
    navsatfix_sub_{this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->declare_parameter("navsatfix_topic_name", "/fix"),
      10,
      std::bind(&R4CCommander::navsatfix_callback, this, std::placeholders::_1))},
    velocity_sub_{this->create_subscription<geometry_msgs::msg::Twist>(
      this->declare_parameter("velocity_topic_name", "/vel"),
      10,
      std::bind(&R4CCommander::velocity_callback, this, std::placeholders::_1))},
    status_sub_{this->create_subscription<r4c_interfaces::msg::TractorStatus>(
      this->declare_parameter("status_topic_name", "/status"),
      10,
      std::bind(&R4CCommander::status_callback, this, std::placeholders::_1))},
    start_mission_srv_{this->create_service<std_srvs::srv::Trigger>(
      this->declare_parameter("start_mission_service", "/r4c_tractor/start"),
      std::bind(&R4CCommander::startMissionCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2))},
    pause_mission_srv_{this->create_service<std_srvs::srv::Trigger>(
      this->declare_parameter("pause_mission_service", "/r4c_tractor/pause"),
      std::bind(&R4CCommander::pauseMissionCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2))},
    stop_mission_srv_{this->create_service<std_srvs::srv::Trigger>(
      this->declare_parameter("stop_mission_service", "/r4c_tractor/stop"),
      std::bind(&R4CCommander::stopMissionCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2))}
  {
    r4c_nav_pkg_share_dir_ = ament_index_cpp::get_package_share_directory("r4c_navigation");

    // Navsat_transform service
    navsat_transform_client_ = this->create_client<robot_localization::srv::FromLL>(
      navsat_service_name_);

    while(!navsat_transform_client_->wait_for_service(std::chrono::milliseconds{5000}))
    {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Service " << navsat_transform_client_->get_service_name()
                                    << " is not available");
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Service " << navsat_transform_client_->get_service_name() << " found");

    // Nav2 action client
    this->nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      this,
      nav2_client_name_);

    if(!this->nav2_client_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Navigation action server not available after waiting");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Navigation action server found");

    // R4C Mission action server

    mission_feedback_ = std::make_shared<r4c_interfaces::action::Nav::Feedback>();
    mission_result_   = std::make_shared<r4c_interfaces::action::Nav::Result>();

    action_server_ = rclcpp_action::create_server<r4c_interfaces::action::Nav>(
      this,
      mission_server_name_,
      std::bind(&R4CCommander::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&R4CCommander::handle_cancel, this, std::placeholders::_1),
      std::bind(&R4CCommander::handle_accepted, this, std::placeholders::_1));

    // Timer for publishing zero-vel navigation command for start and pause
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(100)),
                               std::bind(&R4CCommander::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Initialized");
  }

  // Timer for zero velocity command publishing
  void R4CCommander::timerCallback()
  {
    if(pause_mission_)
    {
      zero_nav_cmd_pub_->publish(zero_nav_cmd_vel);
    }
  }

  // Mission services for start, pause, stop

  bool R4CCommander::startMissionCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                                          std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    RCLCPP_WARN(this->get_logger(), "Received request to start mission");
    if(true)
    {
      start_mission_ = true;
      pause_mission_ = false;
      res->success   = true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Mission cannot be started");
    }
    return true;
  }

  bool R4CCommander::pauseMissionCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                                          std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    RCLCPP_WARN(this->get_logger(), "Received request to pause mission");
    start_mission_ = false;
    pause_mission_ = true;
    res->success   = true;
    return true;
  }

  bool R4CCommander::stopMissionCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                                         std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    RCLCPP_WARN(this->get_logger(), "Received request to stop mission");
    stop_mission_ = true;
    RCLCPP_WARN(this->get_logger(), "Completing current task and finishing mission");
    res->success = true;
    return true;
  }

  // General status callback

  void R4CCommander::status_callback(const r4c_interfaces::msg::TractorStatus::ConstSharedPtr status_msg)
  {
    if(status_msg->localization_status.state == r4c_interfaces::msg::LocalizationStatus::STATE_OK)
    {
      status_ok_ = true;
    }
    else
    {
      status_ok_ = false;
    }
  }

  // Callbacks for feedback information

  void R4CCommander::navsatfix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr navsatfix_msg)
  {
    mission_result_->current_position   = *navsatfix_msg;
    mission_feedback_->current_position = *navsatfix_msg;
  }

  void R4CCommander::velocity_callback(const geometry_msgs::msg::Twist::ConstSharedPtr velocity_msg)
  {
    mission_result_->current_vel   = *velocity_msg;
    mission_feedback_->current_vel = *velocity_msg;
  }

  // Mission action server functions

  rclcpp_action::GoalResponse R4CCommander::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    const std::shared_ptr<const r4c_interfaces::action::Nav::Goal>)
  {
    RCLCPP_INFO(this->get_logger(), "Received mission request");
    (void)uuid;
    if(true)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "Mission request was rejected. Please, check system status before continuing!");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse R4CCommander::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<r4c_interfaces::action::Nav>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel mission");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void R4CCommander::handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<r4c_interfaces::action::Nav>> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&R4CCommander::execute, this, _1), goal_handle}.detach();
  }

  void R4CCommander::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<r4c_interfaces::action::Nav>> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    nav_msgs::msg::Path goal_path         = goal->mission;
    r4c_msgs::msg::NavMission nav_mission = getMission(getMissionPoses(goal_path));

    if(nav_mission.tasks.size() <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "Mission generation failed. Aborting action.");
      RCLCPP_INFO(this->get_logger(), "Mission canceled");
      return;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Computed mission from received goal");
    }

    int index = 0;
    rclcpp::Rate rate(10);

    for(const auto& nav_task: nav_mission.tasks)
    {
      index++;

      nav_task_finished_ = false;
      nav_task_failed_   = false;

      bool last_task = (index == nav_mission.tasks.size());

      // Create goal message for navigation action with task.
      // Send goal for starting navigation with desired BT.

      auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
      goal_msg.poses.push_back(nav_task.pose_a);
      goal_msg.poses.push_back(nav_task.pose_b);
      goal_msg.poses.push_back(nav_task.pose_c);
      goal_msg.poses.push_back(nav_task.pose_d);

      if(nav_task.type == r4c_msgs::msg::NavTask::TASK_INROW_NAVIGATION)
      {
        goal_msg.behavior_tree = r4c_nav_pkg_share_dir_ + "/behavior_trees/" + inrow_bt_name_;
      }
      else if(nav_task.type == r4c_msgs::msg::NavTask::TASK_MANEUVER_NAVIGATION)
      {
        goal_msg.behavior_tree = r4c_nav_pkg_share_dir_ + "/behavior_trees/" + maneuver_bt_name_;
      }
      else if(nav_task.type == r4c_msgs::msg::NavTask::TASK_FREE_NAVIGATION)
      {
        goal_msg.behavior_tree = r4c_nav_pkg_share_dir_ + "/behavior_trees/" + free_bt_name_;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "NavType received is not defined");
        RCLCPP_INFO(this->get_logger(), "Mission canceled");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Sending task goal");

      auto send_goal_options = rclcpp_action::Client<
        nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
      // send_goal_options.goal_response_callback =
      // std::bind(&R4CCommander::nav2_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&R4CCommander::nav2_feedback_callback,
                                                      this,
                                                      std::placeholders::_1,
                                                      std::placeholders::_2);
      send_goal_options.result_callback   = std::bind(&R4CCommander::nav2_result_callback,
                                                    this,
                                                    std::placeholders::_1);

      auto nav_goal_handle_future = this->nav2_client_->async_send_goal(goal_msg,
                                                                        send_goal_options);
      nav_goal_handle_future.wait();
      auto nav_goal_handle = nav_goal_handle_future.get();

      // nav_task_finished_ or nav_task_failed_ will be set in the action client callbacks
      while(rclcpp::ok() && !nav_task_finished_ && !nav_task_failed_)
      {
        // Check if there is a cancel request
        if(goal_handle->is_canceling())
        {
          goal_handle->canceled(mission_result_);
          this->nav2_client_->async_cancel_goal(nav_goal_handle);
          RCLCPP_INFO(this->get_logger(), "Mission canceled");
          return;
        }

        mission_feedback_->index_wp_reached = index - 1;
        mission_result_->index_wp_reached   = index;

        // Publish feedback
        goal_handle->publish_feedback(mission_feedback_);

        rate.sleep();  // inside the while loop
      }

      if(nav_task_failed_)
      {
        stop_mission_  = false;
        pause_mission_ = true;
        RCLCPP_INFO(this->get_logger(), "Mission failed");
        return;
      }
      else if(nav_task_finished_ && (last_task || stop_mission_))
      {
        stop_mission_  = false;
        pause_mission_ = true;
        goal_handle->succeed(mission_result_);
        RCLCPP_INFO(this->get_logger(), "Mission finished successfully");
        return;
      }
    }
  }

  // Nav2 action client functions

  void R4CCommander::nav2_goal_response_callback(
    std::shared_future<
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr>)
  {
    RCLCPP_DEBUG(this->get_logger(), "Goal response");
  }

  void R4CCommander::nav2_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback>)
  {
    RCLCPP_DEBUG(this->get_logger(), "Feedback");
  }

  void R4CCommander::nav2_result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult&
      result)
  {
    switch(result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        nav_task_finished_ = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        nav_task_failed_ = true;
        RCLCPP_WARN(this->get_logger(), "Navigation failed");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        nav_task_failed_ = true;
        RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
        return;
      default:
        nav_task_failed_ = true;
        RCLCPP_WARN(this->get_logger(), "Unknown result code");
        return;
    }
  }

  // Commander functions for transforming received path to navigation poses

  geometry_msgs::msg::Point R4CCommander::getMapPoint(double lat, double lon)
  {
    geometry_msgs::msg::Point nav_pose;

    RCLCPP_WARN_STREAM(this->get_logger(), "Lat, lon: " << lat << ", " << lon);

    auto request                = std::make_shared<robot_localization::srv::FromLL::Request>();
    request->ll_point.latitude  = lat;
    request->ll_point.longitude = lon;

    auto shared_this   = shared_from_this();
    auto result_future = navsat_transform_client_->async_send_request(request);

    std::future_status status = result_future.wait_for(10s);

    if(status == std::future_status::ready)
    {
      auto result = result_future.get();  // Retrieve the result
      nav_pose.x  = result->map_point.x;
      nav_pose.y  = result->map_point.y;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to transform from lat,lon to x,y,theta");
    }

    RCLCPP_WARN_STREAM(this->get_logger(), "x, y: " << nav_pose.x << ", " << nav_pose.y);

    return nav_pose;
  }

  double R4CCommander::calculateTheta(const geometry_msgs::msg::Point& point1,
                                      const geometry_msgs::msg::Point& point2)
  {
    // Calculate the difference in x and y coordinates
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;

    // Calculate the angle (theta) using atan2 function
    double theta = std::atan2(dy, dx);

    // Convert the angle to degrees
    theta = theta * 180.0 / M_PI;

    // Normalize the angle to be within the range [-180, 180]
    if(theta > 180.0)
    {
      theta -= 360.0;
    }
    else if(theta < -180.0)
    {
      theta += 360.0;
    }

    return theta * M_PI / 180.0;
  }

  std::vector<geometry_msgs::msg::PoseStamped> R4CCommander::getMissionPoses(
    const nav_msgs::msg::Path& path_msg)
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;

    if(path_msg.poses.size() % 2 != 0)
    {
      RCLCPP_WARN(this->get_logger(), "Number of poses in goal message is not even.");
      return poses;
    }

    // Loop through the poses in pairs
    for(size_t i = 0; i < path_msg.poses.size() - 1; i += 2)
    {
      auto pose1 = path_msg.poses[i];
      auto pose2 = path_msg.poses[i + 1];

      // Extract latitude and longitude from the poses
      double lat1 = pose1.pose.position.x;
      double lon1 = pose1.pose.position.y;
      double lat2 = pose2.pose.position.x;
      double lon2 = pose2.pose.position.y;

      // Convert latitude and longitude to map coordinates
      geometry_msgs::msg::Point point1 = getMapPoint(lat1, lon1);
      geometry_msgs::msg::Point point2 = getMapPoint(lat2, lon2);

      // Calculate theta between the points
      double theta = calculateTheta(point1, point2);

      // Create Pose2D messages for both poses
      geometry_msgs::msg::Pose2D pose_msg1;
      pose_msg1.x     = point1.x;
      pose_msg1.y     = point1.y;
      pose_msg1.theta = theta;

      geometry_msgs::msg::Pose pose_1 = nav_2d_utils::pose2DToPose(pose_msg1);

      geometry_msgs::msg::PoseStamped pose_1_stamped;
      pose_1_stamped.pose            = pose_1;
      pose_1_stamped.header.frame_id = mission_frame_;
      pose_1_stamped.header.stamp    = this->now();

      geometry_msgs::msg::Pose2D pose_msg2;
      pose_msg2.x     = point2.x;
      pose_msg2.y     = point2.y;
      pose_msg2.theta = theta;

      geometry_msgs::msg::Pose pose_2 = nav_2d_utils::pose2DToPose(pose_msg2);

      geometry_msgs::msg::PoseStamped pose_2_stamped;
      pose_2_stamped.pose            = pose_2;
      pose_2_stamped.header.frame_id = mission_frame_;
      pose_2_stamped.header.stamp    = this->now();

      poses.push_back(pose_1_stamped);
      poses.push_back(pose_2_stamped);
    }

    return poses;
  }


  // TODO: Change how poses are passed in this two functions to unify it. Now its kinda mess.
  r4c_msgs::msg::NavTask R4CCommander::createNavTask(const geometry_msgs::msg::PoseStamped& pose_a,
                                                     const geometry_msgs::msg::PoseStamped& pose_b,
                                                     const geometry_msgs::msg::PoseStamped& pose_c,
                                                     const geometry_msgs::msg::PoseStamped& pose_d,
                                                     int type)
  {
    r4c_msgs::msg::NavTask nav_task;

    nav_task.type = type;

    nav_task.pose_a = pose_a;
    nav_task.pose_b = pose_b;
    nav_task.pose_c = pose_c;
    nav_task.pose_d = pose_d;

    return nav_task;
  }

  r4c_msgs::msg::NavMission R4CCommander::getMission(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses)
  {
    r4c_msgs::msg::NavMission mission;

    if(poses.size() <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "Empty poses passed to mission generation.");
      return mission;
    }

    // Initial FreeNavigation task
    mission.tasks.push_back(createNavTask(poses[0],
                                          poses[0],
                                          poses[0],
                                          poses[1],
                                          r4c_msgs::msg::NavTask::TASK_FREE_NAVIGATION));
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Free navigation to: " << poses[0].pose.position.x << ","
                                              << poses[0].pose.position.y << "at frame "
                                              << poses[0].header.frame_id);
    for(size_t i = 0; i < poses.size(); i += 2)
    {
      // Create InRow task
      mission.tasks.push_back(createNavTask(poses[i],
                                            poses[i + 1],
                                            poses[i],
                                            poses[i + 1],
                                            r4c_msgs::msg::NavTask::TASK_INROW_NAVIGATION));
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Straight from: "
                           << poses[i].pose.position.x << "," << poses[i].pose.position.y << " to "
                           << poses[i + 1].pose.position.x << "," << poses[i + 1].pose.position.y);
      // Create Maneuver task between rows if not the last pair
      if(i + 2 < poses.size())
      {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Maneuver from: " << poses[i + 1].pose.position.x << ","
                                             << poses[i + 1].pose.position.y << " to "
                                             << poses[i + 2].pose.position.x << ","
                                             << poses[i + 2].pose.position.y);
        r4c_msgs::msg::NavTask man_nav_task = createNavTask(
          poses[i],
          poses[i + 1],
          poses[i + 2],
          poses[i + 3],
          r4c_msgs::msg::NavTask::TASK_MANEUVER_NAVIGATION);
        mission.tasks.push_back(man_nav_task);
      }
    }
    return mission;
  }

}  // namespace r4c_commander
