cmake_minimum_required(VERSION 3.5)
project(r4c_commander)

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

set(library_name ${PROJECT_NAME})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(r4c_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(r4c_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)

set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclpy
  can_msgs
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  r4c_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  r4c_interfaces
  std_msgs
  std_srvs
  action_msgs
  rosidl_default_runtime
  robot_localization
  geographic_msgs
  nav_2d_utils
)

install(DIRECTORY
  config
  include
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(r4c_commander_node src/r4c_commander_node.cpp src/r4c_commander/r4c_commander.cpp)
target_include_directories(r4c_commander_node PRIVATE include)
target_link_libraries(r4c_commander_node)
target_compile_options(r4c_commander_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(r4c_commander_node ${dependencies})

# Install cpp executables
install(TARGETS
  r4c_commander_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install python executables
install(PROGRAMS
  scripts/fc_diagnostic_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  r4c_commander
)

ament_export_dependencies(${dependencies})

ament_package()
