cmake_minimum_required(VERSION 3.16)
project(dt_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dt_msgs REQUIRED)
find_package(dt_core REQUIRED)

add_library(dt_ros_node SHARED src/dt_node.cpp)

target_include_directories(dt_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dt_ros_node
  rclcpp
  sensor_msgs
  dt_msgs
  dt_core
)

ament_export_targets(export_dt_ros_node HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs dt_msgs dt_core)

ament_package()