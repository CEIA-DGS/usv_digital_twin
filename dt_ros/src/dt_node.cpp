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

types::Velocity DigitalTwinNode::estimate_velocity(const types::Pose& current_pose, const rclcpp::Time& current_time) {
    if (is_first_gps_) {
        last_gps_time_ = current_time;
        last_gps_pose_ = current_pose;
        is_first_gps_ = false;
        return types::Velocity(0.0, 0.0, 0.0);
    }

    double dt = (current_time - last_gps_time_).seconds();
    
    // Prevention of division by zero if two messages arrive with the same stamp.
    if (dt <= 0.001) {
        return current_velocity_; 
    }

    double dx = current_pose.get_x() - last_gps_pose_.get_x();
    double dy = current_pose.get_y() - last_gps_pose_.get_y();

    double raw_vx = dx / dt;
    double raw_vy = dy / dt;

    // avoid startup irrealistic velocity
    double current_speed = std::hypot(raw_vx, raw_vy);
    if (current_speed > 25.0) {
        last_gps_time_ = current_time;
        last_gps_pose_ = current_pose;
        return current_velocity_;
    }

    // Low-Pass Filter to smooth the speed (alpha = 0.4)
    // 0.4 means we assign 40% weight to the new measurement and 60% to the movement's inertia
    double alpha = 0.4; 
    double filtered_vx = alpha * raw_vx + (1.0 - alpha) * current_velocity_.get_vx();
    double filtered_vy = alpha * raw_vy + (1.0 - alpha) * current_velocity_.get_vy();

    // Updates previous states
    last_gps_time_ = current_time;
    last_gps_pose_ = current_pose;
    
    return types::Velocity(filtered_vx, filtered_vy, 0.0);
}

void DigitalTwinNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
        return;
    }

    // Update pose
    conversions::apply_gps_to_pose(*msg, current_pose_);

    // Estimate Velocity
    rclcpp::Time current_time(msg->header.stamp);
    current_velocity_ = estimate_velocity(current_pose_, current_time);
    types::Kinematics current_kinematics(current_velocity_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "GPS Received, Pose and Vel updated! Lat: %.4f | Lon: %.4f | Vel: %.4f m/s", 
        msg->latitude, msg->longitude, std::hypot(current_velocity_.get_vx(), current_velocity_.get_vy()));

    dt_core_->update_vehicle_pose(current_pose_);
    dt_core_->update_vehicle_kinematics(current_kinematics);
}

void DigitalTwinNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    conversions::apply_imu_to_pose(*msg, current_pose_);
}

void DigitalTwinNode::ais_callback(const dt_msgs::msg::AisReport::SharedPtr msg) {
    std::vector<types::Target> core_targets = conversions::ais_to_core_targets(*msg);
    dt_core_->update_dynamic_targets(core_targets);
}

void DigitalTwinNode::waypoint_callback(const dt_msgs::msg::WaypointArray::SharedPtr msg) {
    types::Trajectory planned_trajectory = conversions::waypoints_to_trajectory(*msg);
    dt_core_->update_planned_trajectory(planned_trajectory);
    
    RCLCPP_INFO(this->get_logger(), "New route updated with %zu waypoints.", msg->waypoints.size());
}

} // namespace dt_ros