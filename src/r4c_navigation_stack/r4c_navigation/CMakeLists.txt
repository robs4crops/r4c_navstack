cmake_minimum_required(VERSION 3.5)
project(r4c_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(eut_crop_row_estimator REQUIRED)
find_package(eut_fixed_gplanner REQUIRED)
find_package(eut_maneuver_generation REQUIRED)
find_package(eut_pure_pursuit REQUIRED)
find_package(vector_pursuit_controller REQUIRED)
find_package(eut_nav_utils REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclpy
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  eut_crop_row_estimator
  eut_fixed_gplanner
  eut_maneuver_generation
  eut_pure_pursuit
  eut_nav_utils
  vector_pursuit_controller
  std_msgs
  std_srvs
  nav2_util
  nav_2d_utils
  nav2_core
  action_msgs
  rosidl_default_runtime
)

install(DIRECTORY
  action
  behavior_trees
  config
  include
  farms
  launch
  maps
  plugins
  waypoints
  DESTINATION share/${PROJECT_NAME}/
)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PathNavigation.action"
  DEPENDENCIES std_msgs action_msgs nav_msgs
)

# Libraries
add_library(do_while_in_parcel_bt_node SHARED plugins/decorators/do_while_in_parcel.cpp)
list(APPEND plugin_libs do_while_in_parcel_bt_node)

add_library(load_control_point_bt_node SHARED plugins/action/load_control_point.cpp)
list(APPEND plugin_libs load_control_point_bt_node)

# add_library(change_waypoints_bt_node SHARED plugins/action/change_waypoints.cpp)
# list(APPEND plugin_libs change_waypoints_bt_node)

add_library(compute_maneuver_path_bt_node SHARED plugins/action/compute_maneuver_path.cpp)
list(APPEND plugin_libs compute_maneuver_path_bt_node)

add_library(path_blackboard_bt_node SHARED plugins/action/path_blackboard.cpp)
list(APPEND plugin_libs path_blackboard_bt_node)

add_library(super_wait_bt_node SHARED plugins/action/super_wait.cpp)
list(APPEND plugin_libs super_wait_bt_node)

add_library(set_navigation_poses_bt_node SHARED plugins/action/set_navigation_poses.cpp)
list(APPEND plugin_libs set_navigation_poses_bt_node)

add_library(get_path_service_bt_node SHARED plugins/action/get_path_service.cpp)
list(APPEND plugin_libs get_path_service_bt_node)

add_library(set_maneuver_params_bt_node SHARED plugins/action/set_maneuver_params.cpp)
list(APPEND plugin_libs set_maneuver_params_bt_node)

add_library(extend_aligned_path_bt_node SHARED plugins/action/extend_aligned_path.cpp)
list(APPEND plugin_libs extend_aligned_path_bt_node)

add_library(call_empty_srv_bt_node SHARED plugins/action/call_empty_srv.cpp)
list(APPEND plugin_libs call_empty_srv_bt_node)

add_library(define_parcel_bt_node SHARED plugins/action/define_parcel.cpp)
list(APPEND plugin_libs define_parcel_bt_node)

add_library(get_alignment_pose_bt_node SHARED plugins/action/get_alignment_pose.cpp)
list(APPEND plugin_libs get_alignment_pose_bt_node)

add_library(get_maneuver_origin_bt_node SHARED plugins/action/get_maneuver_origin.cpp)
list(APPEND plugin_libs get_maneuver_origin_bt_node)

add_library(append_paths_bt_node SHARED plugins/action/append_paths.cpp)
list(APPEND plugin_libs append_paths_bt_node)

add_library(is_path_outdated_bt_node SHARED plugins/conditions/is_path_outdated.cpp)
list(APPEND plugin_libs is_path_outdated_bt_node)

add_library(rate_pipeline_sequence_bt_node SHARED plugins/control/rate_pipeline_sequence.cpp)
list(APPEND plugin_libs rate_pipeline_sequence_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PRIVATE include)
  target_compile_options( ${bt_plugin} PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# Waypoint collector

add_library(waypoint_collector SHARED utils/r4c_navigation/waypoint_collector.cpp)
rosidl_target_interfaces(waypoint_collector ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(waypoint_collector PUBLIC include)
target_compile_options(waypoint_collector PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(waypoint_collector ${dependencies})

add_executable(waypoint_collector_node utils/waypoint_collector_node.cpp)
target_include_directories(waypoint_collector_node PRIVATE)
target_link_libraries(waypoint_collector_node waypoint_collector)
target_compile_options(waypoint_collector_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(waypoint_collector_node ${dependencies})


#### TODO: Check if these utils are used and reprogram them not to use custom r4c_navigation::path_navigation
# add_executable(cancel_path_navigation utils/cancel_path_navigation.cpp)
# rosidl_target_interfaces(cancel_path_navigation ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_include_directories(cancel_path_navigation PRIVATE)
# target_compile_options(cancel_path_navigation PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
# ament_target_dependencies(cancel_path_navigation ${dependencies})
# 
# add_executable(publish_reactive_path utils/publish_reactive_path.cpp)
# target_include_directories(publish_reactive_path PRIVATE)
# target_compile_options(publish_reactive_path PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
# ament_target_dependencies(publish_reactive_path ${dependencies})

# Install libraries
install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install cpp executables
install(TARGETS
  waypoint_collector_node
  # cancel_path_navigation
  # publish_reactive_path
  DESTINATION lib/${PROJECT_NAME}
)

# Install python executables
install(PROGRAMS
  utils/publish_fixed_path.py
  utils/follow_row_path_navigation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
