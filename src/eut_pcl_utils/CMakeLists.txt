cmake_minimum_required(VERSION 3.5)
project(eut_pcl_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
#tf2

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  angles
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  nav_msgs
  livox_interfaces
  livox_ros_driver2
  )

add_library(eut_pcl_utils SHARED
  src/ground_detector.cpp
  src/pcl_utils.cpp
)
target_include_directories(eut_pcl_utils PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(eut_pcl_utils ${PCL_LIBRARIES})
target_compile_options(eut_pcl_utils PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(eut_pcl_utils ${dependencies})

add_executable(registered_cloud_assembler_node src/registered_cloud_assembler.cpp)
# target_compile_definitions(registered_cloud_assembler_node PRIVATE <ANY_DEFINITION>)
target_include_directories(registered_cloud_assembler_node PRIVATE include)
target_compile_options(registered_cloud_assembler_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(registered_cloud_assembler_node ${dependencies})

add_executable(cloud_circular_assembler_node src/cloud_circular_assembler.cpp)
# target_compile_definitions(cloud_circular_assembler_node PRIVATE <ANY_DEFINITION>)
target_include_directories(cloud_circular_assembler_node PRIVATE include)
target_compile_options(cloud_circular_assembler_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(cloud_circular_assembler_node ${dependencies})

add_executable(livox_repub_node src/livox_repub.cpp)
# target_compile_definitions(livox_repub_node PRIVATE <ANY_DEFINITION>)
target_include_directories(livox_repub_node PRIVATE)
target_compile_options(livox_repub_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(livox_repub_node ${dependencies})

add_executable(rslidar_horizontal_angle_range_filter_node src/rslidar_horizontal_angle_range_filter.cpp)
target_include_directories(rslidar_horizontal_angle_range_filter_node PRIVATE)
target_link_libraries(rslidar_horizontal_angle_range_filter_node eut_pcl_utils)
target_compile_options(rslidar_horizontal_angle_range_filter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(rslidar_horizontal_angle_range_filter_node ${dependencies})

add_executable(rshelios_converter_node src/rshelios_converter.cpp)
# target_compile_definitions(rshelios_converter_node PRIVATE <ANY_DEFINITION>)
target_include_directories(rshelios_converter_node PRIVATE)
target_compile_options(rshelios_converter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(rshelios_converter_node ${dependencies})

install(TARGETS eut_pcl_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cloud_circular_assembler_node
  rslidar_horizontal_angle_range_filter_node
  livox_repub_node
  registered_cloud_assembler_node
  rshelios_converter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(eut_pcl_utils)
ament_export_dependencies(${dependencies})
ament_package()