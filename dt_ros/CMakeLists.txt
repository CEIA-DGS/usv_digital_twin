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

target_link_libraries(dt_ros_node dt_core::dt_core)

ament_target_dependencies(dt_ros_node
  rclcpp
  sensor_msgs
  dt_msgs
)

ament_export_dependencies(rclcpp sensor_msgs dt_msgs)

ament_package()