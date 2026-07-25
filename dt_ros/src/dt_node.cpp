#include "dt_ros/dt_node.hpp"
#include <cmath>

namespace dt_ros {

DigitalTwinNode::DigitalTwinNode(std::shared_ptr<dt::DigitalTwinCore> dt_core, const rclcpp::NodeOptions& options)
    : Node("digital_twin_node", options), dt_core_(std::move(dt_core)) 
{
    RCLCPP_INFO(this->get_logger(), "Inicializando Digital Twin ROS Adapter...");

    // QoS Profile para sensores
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();

    // Inicialização dos Subscribers
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "sensors/gps/fix", sensor_qos,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { gps_callback(msg); }
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "sensors/imu/data", sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); }
    );

    ais_sub_ = this->create_subscription<dt_msgs::msg::AisReport>(
        "sensors/ais/report", 10,
        [this](const dt_msgs::msg::AisReport::SharedPtr msg) { ais_callback(msg); }
    );
}

void DigitalTwinNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "GPS sem sinal de fixação.");
        return;
    }

    current_pose_.position.x = msg->latitude;
    current_pose_.position.y = msg->longitude;
    current_pose_.position.z = msg->altitude;

    // dt_core_->update_vehicle_pose(current_pose_);
}

void DigitalTwinNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    
    current_pose_.orientation = msg->orientation;
    
    // dt_core_->update_vehicle_pose(current_pose_);
}

void DigitalTwinNode::ais_callback(const dt_msgs::msg::AisReport::SharedPtr msg) {
    std::vector<dt::types::Target> core_targets;
    core_targets.reserve(msg->targets.size());

    const double KNOTS_TO_MS = 0.514444;
    const double DEG_TO_RAD = M_PI / 180.0;

    for (const auto& ais_target : msg->targets) {
        dt::types::Target t;
        t.id = ais_target.mmsi;
        
        // Posição
        t.pose.position.x = ais_target.latitude;
        t.pose.position.y = ais_target.longitude;
        t.pose.position.z = 0.0; 
        
        // Conversões náuticas -> SI
        t.velocity = ais_target.sog * KNOTS_TO_MS;
        
        // O AIS fornece COG (Course) e Heading (Proa). 
        t.pose.yaw = ais_target.heading * DEG_TO_RAD; 
        
        core_targets.push_back(t);
    }

    //dt_core_->update_dynamic_targets(core_targets);
}

} // namespace dt_ros