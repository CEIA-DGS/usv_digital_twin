#pragma once // Evita inclusões duplas na compilação

#include <vector>
#include <cmath>

// Mensagens do ROS
#include "dt_msgs/msg/ais_report.hpp"
#include "dt_msgs/msg/waypoint_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "dt_core/types.hpp" 

namespace dt_ros {
namespace conversions {

    constexpr double KNOTS_TO_MS = 0.514444;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double UNITY_YAW_OFFSET = M_PI / 2.0;

    // Converte uma lista de alvos AIS do ROS para o formato do Core
    std::vector<types::Target> aisToCoreTargets(const dt_msgs::msg::AisReport& msg);

    // Converte NavSatFix para Pose atualizando X, Y (UTM) e Z
    void applyGpsToPose(const sensor_msgs::msg::NavSatFix& msg, types::Pose& pose);

    // Converte Quaternion do ROS para os ângulos de Euler com a compensação do Unity
    void applyImuToPose(const sensor_msgs::msg::Imu& msg, types::Pose& pose);

    // Converte array de waypoints do ROS para Trajectory do Core
    types::Trajectory waypointsToTrajectory(const dt_msgs::msg::WaypointArray& msg);

} // namespace conversions
} // namespace dt_ros