cmake_minimum_required(VERSION 3.16)
project(dt_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ==============================================================================
# 1. SYSTEM & ROS 2 DEPENDENCIES
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dt_msgs REQUIRED)
find_package(dt_core REQUIRED)
find_package(tf2 REQUIRED)

# ==============================================================================
# 2. LIBRARY BUILD
# ==============================================================================
add_library(dt_ros_node SHARED 
  src/dt_node.cpp
  src/conversions.cpp
  src/utils/geo_utils.cpp
)

target_include_directories(dt_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dt_ros_node dt_core::dt_core)

ament_target_dependencies(dt_ros_node
  rclcpp
  sensor_msgs
  dt_msgs
  tf2
)

# ==============================================================================
# 3. INSTALLATION & EXPORT RULES
# ==============================================================================
install(TARGETS dt_ros_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(dt_ros_node)
ament_export_dependencies(rclcpp sensor_msgs dt_msgs tf2 dt_core)

# ==============================================================================
# 4. UNIT TESTS (GTEST)
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_include_directories(test_conversions PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_conversions
    rclcpp
    sensor_msgs
    dt_msgs
    tf2
    dt_core
  )
  target_link_libraries(test_conversions dt_ros_node)

  ament_add_gtest(test_geo_utils test/utils/test_geo_utils.cpp)
  target_include_directories(test_geo_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_geo_utils dt_ros_node)
endif()

ament_package()