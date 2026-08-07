/**
 * @file conversions.hpp
 * @brief Conversion utilities between ROS 2 messages and Digital Twin Core types.
 */

#pragma once

#include <vector>
#include <cmath>

#include "dt_msgs/msg/ais_report.hpp"
#include "dt_msgs/msg/waypoint_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "dt_core/types.hpp" 

namespace dt_ros {
namespace conversions {

    /// Conversion factor from knots to meters per second.
    constexpr double KNOTS_TO_MS = 0.514444;
    
    /// Conversion factor from degrees to radians.
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    
    /// Yaw offset to align the ROS coordinate frame with the Unity left-handed frame.
    constexpr double UNITY_YAW_OFFSET = M_PI / 2.0;

    /**
     * @brief Converts an array of ROS AIS targets into Core domain targets.
     * 
     * @param msg The incoming ROS AIS report message.
     * @return std::vector<types::Target> A list of targets with computed UTM positions and velocities.
     */
    std::vector<types::Target> aisToCoreTargets(const dt_msgs::msg::AisReport& msg);

    /**
     * @brief Updates a domain Pose object using geographical GPS data.
     * 
     * Converts WGS84 latitude and longitude to UTM planar coordinates and updates the target pose.
     * 
     * @param msg The incoming ROS NavSatFix message.
     * @param pose Reference to the Pose object to be updated.
     */
    void applyGpsToPose(const sensor_msgs::msg::NavSatFix& msg, types::Pose& pose);

    /**
     * @brief Updates a domain Pose object using IMU orientation data.
     * 
     * Converts ROS Quaternions to Euler angles, applies visualization coordinate frame offsets,
     * and normalizes the resulting yaw.
     * 
     * @param msg The incoming ROS Imu message.
     * @param pose Reference to the Pose object to be updated.
     */
    void applyImuToPose(const sensor_msgs::msg::Imu& msg, types::Pose& pose);

    /**
     * @brief Converts a ROS waypoint mission array into a Core Trajectory.
     * 
     * @param msg The incoming ROS WaypointArray message.
     * @return types::Trajectory The resulting planned trajectory with UTM coordinates.
     */
    types::Trajectory waypointsToTrajectory(const dt_msgs::msg::WaypointArray& msg);

} // namespace conversions
} // namespace dt_ros