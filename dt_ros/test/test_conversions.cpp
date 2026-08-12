/**
 * @file test_conversions.cpp
 * @brief Unit tests for ROS to Core conversion utilities.
 */

#include <gtest/gtest.h>
#include <cmath>
#include "dt_ros/conversions.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "dt_msgs/msg/ais_report.hpp"
#include "dt_msgs/msg/waypoint_array.hpp"

namespace dt_ros {
namespace conversions {

/**
 * @brief Test suite for all conversion functions.
 */
class ConversionsTest : public ::testing::Test {
protected:
    // Tolerance for floating-point comparisons in spatial calculations
    const double EPSILON = 1e-5;
};

/**
 * @brief Verifies that GPS coordinates are correctly applied to a Pose object.
 */
TEST_F(ConversionsTest, ApplyGpsToPose) {
    sensor_msgs::msg::NavSatFix msg;
    msg.latitude = -15.7634;
    msg.longitude = -47.8703;
    msg.altitude = 1050.0;

    types::Pose pose;
    apply_gps_to_pose(msg, pose);

    types::Point result_point = pose.get_position();

    EXPECT_NEAR(result_point.get_lat(), msg.latitude, EPSILON);
    EXPECT_NEAR(result_point.get_lon(), msg.longitude, EPSILON);
    EXPECT_NEAR(result_point.get_z(), msg.altitude, EPSILON);
}

/**
 * @brief Verifies IMU quaternion conversion to Euler angles with offset and normalization.
 */
TEST_F(ConversionsTest, ApplyImuToPose) {
    sensor_msgs::msg::Imu msg;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.7071068;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.7071068;

    types::Pose pose;
    apply_imu_to_pose(msg, pose);

    EXPECT_NEAR(pose.get_pitch(), M_PI / 2.0, EPSILON);
}

/**
 * @brief Verifies the conversion of an AIS report array into core Target objects.
 */
TEST_F(ConversionsTest, AisToCoreTargets) {
    dt_msgs::msg::AisReport msg;
    
    dt_msgs::msg::AisTarget target;
    target.mmsi = 123456789;
    target.latitude = -15.7634;
    target.longitude = -47.8703;
    target.heading = 90.0; 
    target.sog = 10.0;
    msg.targets.push_back(target);

    std::vector<types::Target> core_targets = ais_to_core_targets(msg);

    ASSERT_EQ(core_targets.size(), 1);
    EXPECT_EQ(core_targets[0].get_id(), 123456789);
    
    EXPECT_EQ(core_targets[0].get_description(), "AIS_Target");
    EXPECT_NEAR(core_targets[0].get_pose().get_yaw(), 90.0 * (M_PI / 180.0), EPSILON);
}

/**
 * @brief Verifies the conversion of a ROS WaypointArray into a core Trajectory.
 */
TEST_F(ConversionsTest, WaypointsToTrajectory) {
    dt_msgs::msg::WaypointArray msg;
    
    msg.waypoints.resize(1);
    msg.waypoints[0].latitude = -15.7634;
    msg.waypoints[0].longitude = -47.8703;
    msg.waypoints[0].altitude = 0.0;

    types::Trajectory trajectory = waypoints_to_trajectory(msg);

    ASSERT_EQ(trajectory.get_poses().size(), 1);
    
    types::Point p = trajectory.get_poses()[0].get_position();
    EXPECT_NEAR(p.get_lat(), msg.waypoints[0].latitude, EPSILON);
    EXPECT_NEAR(p.get_lon(), msg.waypoints[0].longitude, EPSILON);
}

} // namespace conversions
} // namespace dt_ros