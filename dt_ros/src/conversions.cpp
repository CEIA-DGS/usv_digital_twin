/**
 * @file conversions.cpp
 * @brief Implementation of ROS to Core conversion utilities.
 */

#include "dt_ros/conversions.hpp"
#include "dt_ros/utils/geo_utils.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace dt_ros {
namespace conversions {

void apply_gps_to_pose(const sensor_msgs::msg::NavSatFix& msg, types::Pose& pose) {
    utils::UTMCoord utm = utils::lat_lon_to_utm(msg.latitude, msg.longitude);

    types::Point p;
    p.set_lat_lon(msg.latitude, msg.longitude); 
    p.set_x(utm.x); 
    p.set_y(utm.y); 
    p.set_z(msg.altitude);

    pose.set_position(p);
}

void apply_imu_to_pose(const sensor_msgs::msg::Imu& msg, types::Pose& pose) {
    tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Apply visualization engine reference frame compensation
    yaw += UNITY_YAW_OFFSET;

    // Normalize yaw to strictly bound within [-PI, PI]
    if (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    } else if (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }

    pose.set_orientation(roll, pitch, yaw);
}

std::vector<types::Target> ais_to_core_targets(const dt_msgs::msg::AisReport& msg) {
    std::vector<types::Target> core_targets;
    core_targets.reserve(msg.targets.size());

    for (const auto& ais_target : msg.targets) {
        utils::UTMCoord utm = utils::lat_lon_to_utm(ais_target.latitude, ais_target.longitude);

        types::Pose target_pose(utm.x, utm.y, 0.0, 0.0, 0.0, ais_target.heading * DEG_TO_RAD);
        
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

    return core_targets;
}

types::Trajectory waypoints_to_trajectory(const dt_msgs::msg::WaypointArray& msg) {
    types::Trajectory planned_trajectory;
    
    for (const auto& wp : msg.waypoints) {
        utils::UTMCoord utm = utils::lat_lon_to_utm(wp.latitude, wp.longitude);

        types::Point p;
        p.set_lat_lon(wp.latitude, wp.longitude);
        p.set_x(utm.x);
        p.set_y(utm.y);
        p.set_z(wp.altitude);

        types::Pose wp_pose;
        wp_pose.set_position(p);
        
        planned_trajectory.add_pose(wp_pose);
    }

    return planned_trajectory;
}

} // namespace conversions
} // namespace dt_ros