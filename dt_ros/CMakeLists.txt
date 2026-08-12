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
    file(GLOB_RECURSE TEST_SOURCES "test/*.cpp")
    if(TEST_SOURCES)
        ament_add_gtest(test_dt_ros ${TEST_SOURCES})
        target_include_directories(test_dt_ros PRIVATE include)
        target_link_libraries(test_dt_ros dt_ros_node)
        ament_target_dependencies(test_dt_ros
            rclcpp
            sensor_msgs
            dt_msgs
            tf2
            dt_core
        )
    else()
        message(WARNING "No test files found in the 'test/' directory.")
    endif()
endif()

ament_package()