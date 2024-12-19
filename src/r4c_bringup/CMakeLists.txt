cmake_minimum_required(VERSION 3.5)
project(r4c_bringup)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(r4c_msgs REQUIRED)
find_package(r4c_description REQUIRED)
find_package(r4c_simulation REQUIRED)
find_package(twist_mux REQUIRED)
find_package(eut_ground_vehicle_twist_odometry REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(um7 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  scripts
  test
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(r4c_gnss_init_node src/r4c_gnss_init_main.cpp)
# target_compile_definitions(r4c_gnss_init_node PRIVATE <ANY_DEFINITION>)
target_include_directories(r4c_gnss_init_node PRIVATE include)
target_compile_options(r4c_gnss_init_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(r4c_gnss_init_node rclcpp r4c_msgs sensor_msgs geographic_msgs robot_localization tf2 tf2_geometry_msgs)

install(TARGETS
  r4c_gnss_init_node
  DESTINATION lib/${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/compress_image.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
