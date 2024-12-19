cmake_minimum_required(VERSION 3.5)
project(r4c_reporter)

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
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(r4c_interfaces REQUIRED)
find_package(r4c_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclpy
  can_msgs
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  r4c_interfaces
  r4c_msgs
  rtcm_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  action_msgs
  rosidl_default_runtime
)

install(DIRECTORY
  config
  include
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(sensors_reporter_node src/sensors_reporter_node.cpp src/r4c_reporter/sensors_reporter.cpp)
target_include_directories(sensors_reporter_node PRIVATE include)
target_link_libraries(sensors_reporter_node)
target_compile_options(sensors_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(sensors_reporter_node ${dependencies})

add_executable(camera_reporter_node src/camera_reporter_node.cpp src/r4c_reporter/camera_reporter.cpp)
target_include_directories(camera_reporter_node PRIVATE include)
target_link_libraries(camera_reporter_node)
target_compile_options(camera_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(camera_reporter_node ${dependencies})

add_executable(imu_reporter_node src/imu_reporter_node.cpp src/r4c_reporter/imu_reporter.cpp)
target_include_directories(imu_reporter_node PRIVATE include)
target_link_libraries(imu_reporter_node)
target_compile_options(imu_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(imu_reporter_node ${dependencies})

add_executable(gnss_reporter_node src/gnss_reporter_node.cpp src/r4c_reporter/gnss_reporter.cpp)
target_include_directories(gnss_reporter_node PRIVATE include)
target_link_libraries(gnss_reporter_node)
target_compile_options(gnss_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(gnss_reporter_node ${dependencies})

add_executable(lidar_reporter_node src/lidar_reporter_node.cpp src/r4c_reporter/lidar_reporter.cpp)
target_include_directories(lidar_reporter_node PRIVATE include)
target_link_libraries(lidar_reporter_node)
target_compile_options(lidar_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(lidar_reporter_node ${dependencies})

add_executable(internet_reporter_node src/internet_reporter_node.cpp src/r4c_reporter/internet_reporter.cpp)
target_include_directories(internet_reporter_node PRIVATE include)
target_link_libraries(internet_reporter_node)
target_compile_options(internet_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(internet_reporter_node ${dependencies})

add_executable(can_reporter_node src/can_reporter_node.cpp src/r4c_reporter/can_reporter.cpp)
target_include_directories(can_reporter_node PRIVATE include)
target_link_libraries(can_reporter_node)
target_compile_options(can_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(can_reporter_node ${dependencies})

add_executable(communication_reporter_node src/communication_reporter_node.cpp src/r4c_reporter/communication_reporter.cpp)
target_include_directories(communication_reporter_node PRIVATE include)
target_link_libraries(communication_reporter_node)
target_compile_options(communication_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(communication_reporter_node ${dependencies})

add_executable(localization_reporter_node src/localization_reporter_node.cpp src/r4c_reporter/localization_reporter.cpp)
target_include_directories(localization_reporter_node PRIVATE include)
target_link_libraries(localization_reporter_node)
target_compile_options(localization_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(localization_reporter_node ${dependencies})

add_executable(tractor_reporter_node src/tractor_reporter_node.cpp src/r4c_reporter/tractor_reporter.cpp)
target_include_directories(tractor_reporter_node PRIVATE include)
target_link_libraries(tractor_reporter_node)
target_compile_options(tractor_reporter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(tractor_reporter_node ${dependencies})

# Install cpp executables
install(TARGETS
  sensors_reporter_node
  camera_reporter_node
  imu_reporter_node
  gnss_reporter_node
  lidar_reporter_node
  internet_reporter_node
  can_reporter_node
  communication_reporter_node
  localization_reporter_node
  tractor_reporter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  sensors_reporter
  camera_reporter
  imu_reporter
  gnss_reporter
  lidar_reporter
  internet_reporter
  can_reporter
  communication_reporter
  localization_reporter
  tractor_reporter
)

ament_export_dependencies(${dependencies})

ament_package()