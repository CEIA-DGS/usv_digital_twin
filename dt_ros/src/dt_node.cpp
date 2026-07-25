#include "dt_ros/dt_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

    // Atualiza a posição usando os setters encapsulados da classe Pose/Point
    current_pose_.set_position(msg->latitude, msg->longitude, msg->altitude);

    dt_core_->update_vehicle_pose(current_pose_);
}

void DigitalTwinNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Converte Quaternion do ROS para Ângulos de Euler (Roll, Pitch, Yaw) que o core espera
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Atualiza a orientação usando os setters encapsulados
    current_pose_.set_orientation(roll, pitch, yaw);

    dt_core_->update_vehicle_pose(current_pose_);
}

void DigitalTwinNode::ais_callback(const dt_msgs::msg::AisReport::SharedPtr msg) {
    std::vector<types::Target> core_targets;
    core_targets.reserve(msg->targets.size());

    const double KNOTS_TO_MS = 0.514444;
    const double DEG_TO_RAD = M_PI / 180.0;

    for (const auto& ais_target : msg->targets) {
        // Constrói a pose e cinemática do alvo utilizando o construtor do Target
        types::Pose target_pose(ais_target.latitude, ais_target.longitude, 0.0, 0.0, 0.0, ais_target.heading * DEG_TO_RAD);
        types::Velocity target_vel(ais_target.sog * KNOTS_TO_MS, 0.0, 0.0);
        types::Kinematics target_kin(target_vel);

        // Instancia o Target usando o construtor obrigatório exigido pelo types.hpp
        types::Target t(
            static_cast<int32_t>(ais_target.mmsi),
            "AIS_Target",
            target_pose,
            target_kin
        );
        
        core_targets.push_back(t);
    }

    dt_core_->update_dynamic_targets(core_targets);
}

} // namespace dt_ros