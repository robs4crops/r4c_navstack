cmake_minimum_required(VERSION 3.5)
project(r4c_can_adapter)

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

add_subdirectory(3rdparty/dbcppp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(angles REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(r4c_msgs REQUIRED)

add_executable(agcbox_ens_emulator_node src/agcbox_ens_emulator_main.cpp src/agcbox_ens_emulator.cpp)
target_include_directories(agcbox_ens_emulator_node PRIVATE include)
target_compile_options(agcbox_ens_emulator_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
ament_target_dependencies(agcbox_ens_emulator_node rclcpp rclcpp_components std_msgs r4c_msgs)

add_executable(carob_can2ros_adapter_node src/carob_can2ros_adapter_main.cpp src/carob_can2ros_adapter.cpp src/can2ros_adapter.cpp src/dbc_model_checker.cpp)
#target_compile_definitions(carob_can2ros_adapter_node PRIVATE R4C_CAN_ADAPTER_EXTRA_LOG)
target_include_directories(carob_can2ros_adapter_node PRIVATE include)
target_include_directories(carob_can2ros_adapter_node SYSTEM PRIVATE 3rdparty/dbcppp/include)
target_compile_options(carob_can2ros_adapter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
ament_target_dependencies(carob_can2ros_adapter_node angles std_srvs rclcpp rclcpp_components can_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs r4c_msgs)
target_link_libraries(carob_can2ros_adapter_node libdbcppp)

add_executable(carob_ros2can_adapter_node src/carob_ros2can_adapter_main.cpp src/carob_ros2can_adapter.cpp src/ros2can_adapter.cpp src/dbc_model_checker.cpp)
#target_compile_definitions(carob_ros2can_adapter_node PRIVATE R4C_CAN_ADAPTER_EXTRA_LOG)
target_include_directories(carob_ros2can_adapter_node PRIVATE include)
target_include_directories(carob_ros2can_adapter_node SYSTEM PRIVATE 3rdparty/dbcppp/include)
target_compile_options(carob_ros2can_adapter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
ament_target_dependencies(carob_ros2can_adapter_node angles std_srvs rclcpp rclcpp_components can_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs r4c_msgs)
target_link_libraries(carob_ros2can_adapter_node libdbcppp)

add_executable(tractor_can2ros_adapter_node src/tractor_can2ros_adapter_main.cpp src/tractor_can2ros_adapter.cpp src/can2ros_adapter.cpp src/dbc_model_checker.cpp)
#target_compile_definitions(tractor_can2ros_adapter_node PRIVATE R4C_CAN_ADAPTER_EXTRA_LOG)
target_include_directories(tractor_can2ros_adapter_node PRIVATE include)
target_include_directories(tractor_can2ros_adapter_node SYSTEM PRIVATE 3rdparty/dbcppp/include)
target_compile_options(tractor_can2ros_adapter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
ament_target_dependencies(tractor_can2ros_adapter_node angles std_srvs rclcpp rclcpp_components can_msgs geometry_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs r4c_msgs)
target_link_libraries(tractor_can2ros_adapter_node libdbcppp)

add_executable(tractor_ros2can_adapter_node src/tractor_ros2can_adapter_main.cpp src/tractor_ros2can_adapter.cpp src/ros2can_adapter.cpp src/dbc_model_checker.cpp)
#target_compile_definitions(tractor_ros2can_adapter_node PRIVATE R4C_CAN_ADAPTER_EXTRA_LOG)
target_include_directories(tractor_ros2can_adapter_node PRIVATE include)
target_include_directories(tractor_ros2can_adapter_node SYSTEM PRIVATE 3rdparty/dbcppp/include)
target_compile_options(tractor_ros2can_adapter_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
ament_target_dependencies(tractor_ros2can_adapter_node angles std_srvs rclcpp rclcpp_components can_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs r4c_msgs)
target_link_libraries(tractor_ros2can_adapter_node libdbcppp)

install(TARGETS agcbox_ens_emulator_node carob_can2ros_adapter_node carob_ros2can_adapter_node tractor_can2ros_adapter_node tractor_ros2can_adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config dbc launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
