#include "dt_ros/conversions.hpp"
#include "dt_ros/utils/geo_utils.hpp" // Onde agora reside latLonToUTM
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace dt_ros {
namespace conversions {

void applyGpsToPose(const sensor_msgs::msg::NavSatFix& msg, types::Pose& pose) {
    // 1. Extrai as coordenadas geográficas para UTM
    utils::UTMCoord utm = utils::latLonToUTM(msg.latitude, msg.longitude);

    // 2. Instancia um Point intermediário conforme a interface original
    types::Point p;
    p.set_lat_lon(msg.latitude, msg.longitude); 
    p.set_x(utm.x); 
    p.set_y(utm.y); 
    p.set_z(msg.altitude);

    // 3. Substitui a posição dentro da Pose
    pose.set_position(p);
}

void applyImuToPose(const sensor_msgs::msg::Imu& msg, types::Pose& pose) {
    // 1. Converte Quaternion do ROS para Ângulos de Euler (Roll, Pitch, Yaw)
    tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 2. Compensação do referencial do Unity (usando a constante do hpp)
    yaw += UNITY_YAW_OFFSET;

    // 3. Normaliza o ângulo para mantê-lo sempre no intervalo circular [-PI, PI]
    if (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    } else if (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }

    // 4. Atualiza a orientação da pose passada por referência
    pose.set_orientation(roll, pitch, yaw);
}

std::vector<types::Target> aisToCoreTargets(const dt_msgs::msg::AisReport& msg) {
    std::vector<types::Target> core_targets;
    core_targets.reserve(msg.targets.size());

    for (const auto& ais_target : msg.targets) {
        // 1. Converte a coordenada geográfica do alvo para UTM
        utils::UTMCoord utm = utils::latLonToUTM(ais_target.latitude, ais_target.longitude);

        // 2. Constrói a pose usando as coordenadas UTM planas
        types::Pose target_pose(utm.x, utm.y, 0.0, 0.0, 0.0, ais_target.heading * DEG_TO_RAD);
        
        // 3. Salva o Lat/Lon original no Point
        types::Point p = target_pose.get_position();
        p.set_lat_lon(ais_target.latitude, ais_target.longitude);
        target_pose.set_position(p);

        // 4. Converte velocidade (Knots para m/s) e monta a cinemática
        types::Velocity target_vel(ais_target.sog * KNOTS_TO_MS, 0.0, 0.0);
        types::Kinematics target_kin(target_vel);

        // 5. Monta o Target final
        types::Target t(
            static_cast<int32_t>(ais_target.mmsi),
            "AIS_Target",
            target_pose,
            target_kin
        );
        
        core_targets.push_back(t);
    }

    return core_targets;
}

types::Trajectory waypointsToTrajectory(const dt_msgs::msg::WaypointArray& msg) {
    types::Trajectory planned_trajectory;
    
    for (const auto& wp : msg.waypoints) {
        // 1. Converte lat/lon para UTM
        utils::UTMCoord utm = utils::latLonToUTM(wp.latitude, wp.longitude);

        // 2. Monta o Point com todos os dados
        types::Point p;
        p.set_lat_lon(wp.latitude, wp.longitude);
        p.set_x(utm.x);
        p.set_y(utm.y);
        p.set_z(wp.altitude);

        // 3. Insere na Pose
        types::Pose wp_pose;
        wp_pose.set_position(p);
        
        // 4. Adiciona à trajetória
        planned_trajectory.add_pose(wp_pose);
    }

    return planned_trajectory;
}

} // namespace conversions
} // namespace dt_ros