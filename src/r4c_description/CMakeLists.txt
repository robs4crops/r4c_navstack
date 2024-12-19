cmake_minimum_required(VERSION 3.5)
project(r4c_description)

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

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

find_package(r4c_msgs REQUIRED)

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

set(GAZEBO_MODEL_PATH $ENV{HOME}/.gazebo/models)

add_custom_target(COMMAND cmake -E make_directory ${GAZEBO_MODEL_PATH})

add_executable(diff_drive_joint_states_to_twist_node src/diff_drive_joint_states_to_twist_main.cpp src/diff_drive_joint_states_to_twist.cpp)
# target_compile_definitions(diff_drive_joint_states_to_twist_node PRIVATE <ANY_DEFINITION>)
target_include_directories(diff_drive_joint_states_to_twist_node PRIVATE include)
target_compile_options(diff_drive_joint_states_to_twist_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(diff_drive_joint_states_to_twist_node rclcpp geometry_msgs sensor_msgs)

add_executable(diff_drive_twist_to_joint_states_node src/diff_drive_twist_to_joint_states_main.cpp src/diff_drive_twist_to_joint_states.cpp)
# target_compile_definitions(diff_drive_twist_to_joint_states_node PRIVATE <ANY_DEFINITION>)
target_include_directories(diff_drive_twist_to_joint_states_node PRIVATE include)
target_compile_options(diff_drive_twist_to_joint_states_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic) # -Werror
ament_target_dependencies(diff_drive_twist_to_joint_states_node rclcpp angles geometry_msgs sensor_msgs)

install(
  TARGETS
  diff_drive_joint_states_to_twist_node
  diff_drive_twist_to_joint_states_node
  DESTINATION
  lib/${PROJECT_NAME}
)

install(DIRECTORY config meshes urdf DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY ../${PROJECT_NAME} DESTINATION ${GAZEBO_MODEL_PATH})

ament_package()