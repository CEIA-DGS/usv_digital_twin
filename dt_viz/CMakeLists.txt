cmake_minimum_required(VERSION 3.16)
project(dt_viz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# ROS 2 & Digital Twin Dependencies
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dt_msgs REQUIRED)
find_package(dt_core REQUIRED) 
find_package(dt_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)  

# ==========================================
# Qt5 Dependencies
# ==========================================
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ==========================================
# External Libraries
# ==========================================
# Explicitly finding GDAL since map_canvas.cpp includes ogrsf_frmts.h
find_package(GDAL REQUIRED)

# ==========================================
# Executable Definition
# ==========================================
add_executable(dt_visualizer_node
  include/dt_viz/controllers/simulation_controller.hpp
  include/dt_viz/ui/main_window.hpp
  include/dt_viz/ui/map_canvas.hpp
  include/dt_viz/ui/navigation_scene.hpp
  include/dt_viz/ui/telemetry_panel.hpp
  include/dt_viz/ui/camera_grid_widget.hpp
  include/dt_viz/ui/camera_control_panel.hpp
  src/main.cpp
  src/controllers/simulation_controller.cpp
  src/ui/main_window.cpp
  src/ui/map_canvas.cpp
  src/ui/navigation_scene.cpp
  src/ui/telemetry_panel.cpp
  src/ui/camera_grid_widget.cpp
  src/ui/camera_control_panel.cpp
)

target_include_directories(dt_visualizer_node PRIVATE include)

if(GDAL_INCLUDE_DIRS)
  target_include_directories(dt_visualizer_node PRIVATE ${GDAL_INCLUDE_DIRS})
endif()

target_link_libraries(dt_visualizer_node 
  dt_core::dt_core 
  Qt5::Widgets
  ${GDAL_LIBRARIES}
)

ament_target_dependencies(dt_visualizer_node
  rclcpp
  nav_msgs
  dt_msgs
  dt_ros
  ament_index_cpp
)

# ==========================================
# Install Rules
# ==========================================
install(TARGETS dt_visualizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

ament_package()