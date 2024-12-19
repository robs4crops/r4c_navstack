cmake_minimum_required(VERSION 3.5)
project(r4c_localization)

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
find_package(eut_pcl_utils REQUIRED)
find_package(nav2_common REQUIRED)
find_package(r4c_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eut_fast_lio REQUIRED)

set(DEPENDENCIES
  rclcpp
  r4c_msgs
  sensor_msgs
)

add_executable(agcbox_imu_filter_node src/agcbox_imu_filter.cpp)
ament_target_dependencies(agcbox_imu_filter_node ${DEPENDENCIES})

add_executable(enu_pos_to_navsatfix_node src/enu_pos_to_navsatfix.cpp)
target_link_libraries(enu_pos_to_navsatfix_node ${GeographicLib_LIBRARIES})
ament_target_dependencies(enu_pos_to_navsatfix_node ${DEPENDENCIES})

add_executable(pub_datum_fix_node src/pub_datum_fix.cpp)
ament_target_dependencies(pub_datum_fix_node ${DEPENDENCIES})

install(TARGETS
  agcbox_imu_filter_node
  enu_pos_to_navsatfix_node
  pub_datum_fix_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies()

ament_package()
