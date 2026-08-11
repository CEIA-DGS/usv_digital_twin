/**
 * @file dt_node.hpp
 * @brief Header file for the DigitalTwinNode class.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <dt_msgs/msg/ais_report.hpp>
#include <dt_msgs/msg/waypoint_array.hpp>
#include <dt_core/types.hpp>
#include <dt_core/twin_interface.hpp>
#include <dt_ros/conversions.hpp>
#include <memory>

namespace dt_ros {

/**
 * @class DigitalTwinNode
 * @brief ROS 2 Node responsible for bridging ROS data to the Digital Twin Core.
 * 
 * This node subscribes to telemetry, navigation, and environmental data (GPS, IMU, AIS, Waypoints),
 * converts the ROS messages into domain-specific types, and updates the core digital twin state.
 */
class DigitalTwinNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Digital Twin Node object.
     * 
     * @param dt_core Shared pointer to the Digital Twin Core interface.
     * @param options ROS 2 Node options (defaults to empty options).
     */
    explicit DigitalTwinNode(std::shared_ptr<dt::DigitalTwinCore> dt_core, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::shared_ptr<dt::DigitalTwinCore> dt_core_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<dt_msgs::msg::AisReport>::SharedPtr ais_sub_;
    rclcpp::Subscription<dt_msgs::msg::WaypointArray>::SharedPtr waypoint_sub_;

    /**
     * @brief Callback for GPS data. 
     * 
     * Converts WGS84 coordinates to UTM and updates the vehicle's position.
     * 
     * @param msg NavSatFix message containing latitude, longitude, and altitude.
     */
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief Callback for IMU data. 
     * 
     * Converts ROS Quaternions to Euler angles and applies coordinate frame offsets.
     * 
     * @param msg Imu message containing orientation data.
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Callback for AIS reports. 
     * 
     * Processes external vessel telemetry and updates dynamic targets.
     * 
     * @param msg AisReport message containing surrounding targets.
     */
    void ais_callback(const dt_msgs::msg::AisReport::SharedPtr msg);

    /**
     * @brief Callback for waypoint arrays. 
     * 
     * Updates the vehicle's planned trajectory in the digital twin.
     * 
     * @param msg WaypointArray message containing the mission route.
     */
    void waypoint_callback(const dt_msgs::msg::WaypointArray::SharedPtr msg);
    
    /// Holds the current state (position and orientation) of the ego vehicle.
    types::Pose current_pose_;
};

} // namespace dt_ros