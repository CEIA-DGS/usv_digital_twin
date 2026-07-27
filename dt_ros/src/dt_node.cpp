#include "dt_ros/dt_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace dt_ros {

void latLonToUTM(double lat, double lon, double& utm_x, double& utm_y) {
    // Constantes do Elipsóide WGS84
    const double a = 6378137.0; // Raio equatorial
    const double eccSquared = 0.00669438; // Excentricidade ao quadrado
    const double k0 = 0.9996; // Fator de escala no meridiano central

    double latRad = lat * M_PI / 180.0;
    double lonRad = lon * M_PI / 180.0;

    // Determina o fuso (Zone) do UTM
    int zoneNumber = static_cast<int>((lon + 180.0) / 6.0) + 1;
    double lonOrigin = (zoneNumber - 1) * 6.0 - 180.0 + 3.0; // Centro do fuso
    double lonOriginRad = lonOrigin * M_PI / 180.0;

    double N = a / std::sqrt(1 - eccSquared * std::sin(latRad) * std::sin(latRad));
    double T = std::tan(latRad) * std::tan(latRad);
    double C = eccSquared / (1 - eccSquared) * std::cos(latRad) * std::cos(latRad);
    double A = std::cos(latRad) * (lonRad - lonOriginRad);

    // Cálculo do Meridiano Arco (M)
    double M = a * ((1 - eccSquared/4 - 3*eccSquared*eccSquared/64 - 5*eccSquared*eccSquared*eccSquared/256) * latRad 
                - (3*eccSquared/8 + 3*eccSquared*eccSquared/32 + 45*eccSquared*eccSquared*eccSquared/1024) * std::sin(2*latRad) 
                + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024) * std::sin(4*latRad) 
                - (35*eccSquared*eccSquared*eccSquared/3072) * std::sin(6*latRad));

    utm_x = k0 * N * (A + (1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C-58*eccSquared)*A*A*A*A*A/120) + 500000.0;
    utm_y = k0 * (M + N*std::tan(latRad)*(A*A/2 + (5-T+9*C+4*C*C)*A*A*A*A/24 + (61-58*T+T*T+600*C-330*eccSquared)*A*A*A*A*A*A/720));
    
    if (lat < 0) {
        utm_y += 10000000.0; // Deslocamento para o hemisfério sul
    }
}

DigitalTwinNode::DigitalTwinNode(std::shared_ptr<dt::DigitalTwinCore> dt_core, const rclcpp::NodeOptions& options)
    : Node("digital_twin_node", options), dt_core_(std::move(dt_core)) 
{
    RCLCPP_INFO(this->get_logger(), "Inicializando Digital Twin ROS Adapter...");

    // QoS Profile para sensores
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();

    // Inicialização dos Subscribers
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

    double utm_x, utm_y;
    latLonToUTM(msg->latitude, msg->longitude, utm_x, utm_y);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "GPS Recebido! Lat: %.4f | Lon: %.4f --> UTM_X: %.1f | UTM_Y: %.1f", 
        msg->latitude, msg->longitude, utm_x, utm_y);

    // Precisamos instanciar um Point intermediário para usar o set_lat_lon
    types::Point p;
    p.set_lat_lon(msg->latitude, msg->longitude); // Salva o dado bruto
    p.set_x(utm_x); // Salva a posição espacial real em metros (UTM)
    p.set_y(utm_y); // Salva a posição espacial real em metros (UTM)
    p.set_z(msg->altitude);

    // Substitui a posição dentro da Pose
    current_pose_.set_position(p);

    dt_core_->update_vehicle_pose(current_pose_);
}

void DigitalTwinNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Converte Quaternion do ROS para Ângulos de Euler (Roll, Pitch, Yaw)
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Compensação do referencial do Unity (90 graus)
    yaw += (M_PI / 2.0);

    // Normaliza o ângulo para mantê-lo sempre no intervalo circular [-PI, PI]
    if (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    } else if (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }

    // Atualiza a orientação usando os setters encapsulados
    current_pose_.set_orientation(roll, pitch, yaw);
}

void DigitalTwinNode::ais_callback(const dt_msgs::msg::AisReport::SharedPtr msg) {
    std::vector<types::Target> core_targets;
    core_targets.reserve(msg->targets.size());

    const double KNOTS_TO_MS = 0.514444;
    const double DEG_TO_RAD = M_PI / 180.0;

    for (const auto& ais_target : msg->targets) {
        // 1. Converte a coordenada geográfica do alvo para UTM
        double target_utm_x, target_utm_y;
        latLonToUTM(ais_target.latitude, ais_target.longitude, target_utm_x, target_utm_y);

        // 2. Constrói a pose usando as coordenadas UTM planas
        types::Pose target_pose(target_utm_x, target_utm_y, 0.0, 0.0, 0.0, ais_target.heading * DEG_TO_RAD);
        
        // Além disso, é vital salvar o Lat/Lon original para inspeção
        types::Point p = target_pose.get_position();
        p.set_lat_lon(ais_target.latitude, ais_target.longitude);
        target_pose.set_position(p);

        types::Velocity target_vel(ais_target.sog * KNOTS_TO_MS, 0.0, 0.0);
        types::Kinematics target_kin(target_vel);

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

void DigitalTwinNode::waypoint_callback(const dt_msgs::msg::WaypointArray::SharedPtr msg) {
    types::Trajectory planned_trajectory;
    
    for (const auto& wp : msg->waypoints) {
        double utm_x, utm_y;
        latLonToUTM(wp.latitude, wp.longitude, utm_x, utm_y);

        types::Point p;
        p.set_lat_lon(wp.latitude, wp.longitude);
        p.set_x(utm_x);
        p.set_y(utm_y);
        p.set_z(wp.altitude);

        types::Pose wp_pose;
        wp_pose.set_position(p);
        
        planned_trajectory.add_pose(wp_pose);
    }

    dt_core_->update_planned_trajectory(planned_trajectory);
    
    RCLCPP_INFO(this->get_logger(), "Nova rota recebida com %zu waypoints e enviada ao Core.", msg->waypoints.size());
}

} // namespace dt_ros