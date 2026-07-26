cmake_minimum_required(VERSION 3.16)
project(dt_viz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dt_msgs REQUIRED)
find_package(dt_core REQUIRED) 
find_package(dt_ros REQUIRED)  

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(dt_visualizer_node
  src/main.cpp
  src/main_window.cpp
)

target_include_directories(dt_visualizer_node PRIVATE include)

target_link_libraries(dt_visualizer_node dt_core::dt_core Qt5::Widgets)

ament_target_dependencies(dt_visualizer_node
  rclcpp
  nav_msgs
  dt_msgs
  dt_ros
)

install(TARGETS dt_visualizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

ament_package()