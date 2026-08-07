/**
 * @file dt_node.cpp
 * @brief Implementation of the DigitalTwinNode class.
 */

#include "dt_ros/dt_node.hpp"
#include "dt_ros/conversions.hpp" 

namespace dt_ros {

DigitalTwinNode::DigitalTwinNode(std::shared_ptr<dt::DigitalTwinCore> dt_core, const rclcpp::NodeOptions& options)
    : Node("digital_twin_node", options), dt_core_(std::move(dt_core)) 
{
    RCLCPP_INFO(this->get_logger(), "Initializing Digital Twin ROS Adapter...");

    // QoS Profile for sensor data (Best Effort for high-frequency topics)
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", sensor_qos,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { gps_callback(msg); }
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); }
    );

    ais_sub_ = this->create_subscription<dt_msgs::msg::AisReport>(
        "/ais/report", 10,
        [this](const dt_msgs::msg::AisReport::SharedPtr msg) { ais_callback(msg); }
    );

    waypoint_sub_ = this->create_subscription<dt_msgs::msg::WaypointArray>(
        "/mission/waypoints", 10,
        [this](const dt_msgs::msg::WaypointArray::SharedPtr msg) { waypoint_callback(msg); }
    );
}

void DigitalTwinNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
        return;
    }

    conversions::applyGpsToPose(*msg, current_pose_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "GPS Received and Pose updated! Lat: %.4f | Lon: %.4f", 
        msg->latitude, msg->longitude);

    dt_core_->update_vehicle_pose(current_pose_);
}

void DigitalTwinNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    conversions::applyImuToPose(*msg, current_pose_);
}

void DigitalTwinNode::ais_callback(const dt_msgs::msg::AisReport::SharedPtr msg) {
    std::vector<types::Target> core_targets = conversions::aisToCoreTargets(*msg);
    dt_core_->update_dynamic_targets(core_targets);
}

void DigitalTwinNode::waypoint_callback(const dt_msgs::msg::WaypointArray::SharedPtr msg) {
    types::Trajectory planned_trajectory = conversions::waypointsToTrajectory(*msg);
    dt_core_->update_planned_trajectory(planned_trajectory);
    
    RCLCPP_INFO(this->get_logger(), "New route updated with %zu waypoints.", msg->waypoints.size());
}

} // namespace dt_ros