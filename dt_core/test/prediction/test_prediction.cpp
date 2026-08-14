/**
 * @file test_prediction.cpp
 * @brief Unit tests for the prediction namespace and methods.
 * 
 * Validates trajectory generation, collision reporting, and dynamic risk 
 * calculations, ensuring proper exception handling for invalid inputs.
 */

#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "dt_core/types.hpp"
#include "prediction/prediction.hpp"

/**
 * @class PredictionTest
 * @brief Test suite fixture for the prediction methods.
 * 
 * Prepares standard entities, targets, and trajectories to be used across 
 * multiple tests, ensuring a clean and isolated state for each execution.
 */
class PredictionTest : public ::testing::Test {
protected:
    types::Entity usv_entity = types::Entity(
        types::Pose(0.0, 0.0, 0.0), 
        types::Kinematics(types::Velocity(0.0, 0.0, 0.0)),
        types::Covariance(1.0, 0.0, 1.0)
    );

    std::vector<types::Target> context_targets;
    types::Trajectory base_trajectory;

    /**
     * @brief Setup phase executed before each test runs.
     * 
     * Populates the standard USV state and generates a predictable environment
     * with standard targets to be evaluated by the prediction algorithms.
     */
    void SetUp() override {
        // Initialize a USV moving at 2.0 m/s on the X-axis
        usv_entity.set_pose(types::Pose(0.0, 0.0, 0.0));
        usv_entity.set_velocity(types::Velocity(2.0, 0.0, 0.0));

        // Initialize a static target directly in front of the USV (Risk of collision)
        types::Target target1(
            1, "Static Obstacle", 
            types::Pose(10.0, 0.0, 0.0), 
            types::Kinematics(types::Velocity(0.0, 0.0, 0.0)),
            types::Covariance(1.0, 0.0, 1.0)
        );

        // Initialize a dynamic target moving away safely
        types::Target target2(
            2, "Moving Vessel", 
            types::Pose(0.0, 50.0, 0.0), 
            types::Kinematics(types::Velocity(0.0, 5.0, 0.0)),
            types::Covariance(1.0, 0.0, 1.0)
        );

        context_targets.push_back(target1);
        context_targets.push_back(target2);

        // Create a simple straight trajectory from (0,0) to (20,0)
        base_trajectory.add_pose(types::Pose(0.0, 0.0, 0.0));
        base_trajectory.add_pose(types::Pose(10.0, 0.0, 0.0));
        base_trajectory.add_pose(types::Pose(20.0, 0.0, 0.0));
    }
};

/**
 * @brief Validates the linear mathematical progression of predict_trajectory.
 */
TEST_F(PredictionTest, PredictTrajectory_ValidProgression) {
    double horizon = 2.0;
    double step = 1.0;
    
    types::Trajectory result = prediction::predict_trajectory(usv_entity, horizon, step);

    // Expecting 3 points for t = 0.0, 1.0, 2.0
    ASSERT_EQ(result.size(), 3);
    
    // t=0.0 -> x = 0.0
    EXPECT_DOUBLE_EQ(result.get_pose_by_index(0).get_x(), 0.0);
    // t=1.0 -> x = 2.0 (v_x = 2.0)
    EXPECT_DOUBLE_EQ(result.get_pose_by_index(1).get_x(), 2.0);
    // t=2.0 -> x = 4.0
    EXPECT_DOUBLE_EQ(result.get_pose_by_index(2).get_x(), 4.0);
}

/**
 * @brief Validates input protection for predict_trajectory.
 */
TEST_F(PredictionTest, PredictTrajectory_InvalidInputsThrows) {
    EXPECT_THROW(prediction::predict_trajectory(usv_entity, -1.0, 1.0), std::invalid_argument);
    EXPECT_THROW(prediction::predict_trajectory(usv_entity, 5.0, -1.0), std::invalid_argument);
    EXPECT_THROW(prediction::predict_trajectory(usv_entity, 2.0, 5.0), std::invalid_argument); // Step > Horizon
}

/**
 * @brief Validates target retrieval and trajectory calculation by ID.
 */
TEST_F(PredictionTest, PredictTrajectoryById_ValidTarget) {
    types::Trajectory result = prediction::predict_trajectory_by_id(2, context_targets, 2.0, 1.0);
    
    ASSERT_EQ(result.size(), 3);
    // target2 starts at y=50.0 and moves with vy=5.0
    EXPECT_DOUBLE_EQ(result.get_pose_by_index(2).get_y(), 60.0); // t=2.0 -> y=60.0
}

/**
 * @brief Validates input protection and search logic for predict_trajectory_by_id.
 */
TEST_F(PredictionTest, PredictTrajectoryById_InvalidInputsThrows) {
    EXPECT_THROW(prediction::predict_trajectory_by_id(-1, context_targets, 5.0, 1.0), std::invalid_argument);
    
    std::vector<types::Target> empty_targets;
    EXPECT_THROW(prediction::predict_trajectory_by_id(1, empty_targets, 5.0, 1.0), std::invalid_argument);
    
    // Non-existent ID
    EXPECT_THROW(prediction::predict_trajectory_by_id(99, context_targets, 5.0, 1.0), std::invalid_argument);
}

/**
 * @brief Validates spatial discretization between two distinct points.
 */
TEST_F(PredictionTest, MakeTrajectoryBetween_ValidDiscretization) {
    types::Point origin(0.0, 0.0, 0.0);
    types::Point dest(10.0, 0.0, 0.0);
    double step = 5.0;

    types::Trajectory result = prediction::make_trajectory_between(origin, dest, step);
    
    // Total distance = 10. Step = 5. Points: 0, 5, 10
    ASSERT_EQ(result.size(), 3);
    EXPECT_DOUBLE_EQ(result.get_pose_by_index(0).get_x(), 0.0);
    EXPECT_DOUBLE_EQ(result.get_pose_by_index(1).get_x(), 5.0);
    EXPECT_DOUBLE_EQ(result.get_pose_by_index(2).get_x(), 10.0);
}

/**
 * @brief Validates input protection for trajectory generation between points.
 */
TEST_F(PredictionTest, MakeTrajectoryBetween_InvalidInputsThrows) {
    types::Point origin(0.0, 0.0, 0.0);
    types::Point dest(0.0, 0.0, 0.0);
    
    // Same points
    EXPECT_THROW(prediction::make_trajectory_between(origin, dest, 1.0), std::invalid_argument);
    
    dest.set_x(5.0);
    // Negative step
    EXPECT_THROW(prediction::make_trajectory_between(origin, dest, -1.0), std::invalid_argument);
    // Step larger than distance
    EXPECT_THROW(prediction::make_trajectory_between(origin, dest, 10.0), std::invalid_argument);
}

/**
 * @brief Verifies a safe trajectory correctly returns a safe collision report.
 */
TEST_F(PredictionTest, CheckCollisionsOnTrajectory_SafeRoute) {
    // Override context targets to only have the safe target
    std::vector<types::Target> safe_targets = { context_targets[1] }; 
    
    types::TargetCollisionReport report = prediction::check_collisions_on_trajectory(
        base_trajectory, usv_entity, 2.0, safe_targets
    );

    EXPECT_TRUE(report.is_safe());
    EXPECT_EQ(report.get_msg(), "Report: Trajectory is safe!");
}

/**
 * @brief Verifies an unsafe trajectory detects risk and reports collision.
 */
TEST_F(PredictionTest, CheckCollisionsOnTrajectory_UnsafeRoute) {
    types::TargetCollisionReport report = prediction::check_collisions_on_trajectory(
        base_trajectory, usv_entity, 2.0, context_targets
    );

    // Context contains Target 1 at (10.0, 0.0), exactly on the path
    EXPECT_FALSE(report.is_safe());
    EXPECT_EQ(report.get_id(), 1); // Must identify the correct obstacle ID
    EXPECT_EQ(report.get_msg(), "Warning: Collision along the trajectory!");
}

/**
 * @brief Validates input protection for trajectory collision checking.
 */
TEST_F(PredictionTest, CheckCollisionsOnTrajectory_InvalidInputsThrows) {
    types::Trajectory empty_trajectory;
    EXPECT_THROW(prediction::check_collisions_on_trajectory(empty_trajectory, usv_entity, 2.0, context_targets), std::invalid_argument);

    types::Trajectory small_trajectory;
    small_trajectory.add_pose(types::Pose(0.0, 0.0, 0.0));
    EXPECT_THROW(prediction::check_collisions_on_trajectory(small_trajectory, usv_entity, 2.0, context_targets), std::invalid_argument);

    EXPECT_THROW(prediction::check_collisions_on_trajectory(base_trajectory, usv_entity, -2.0, context_targets), std::invalid_argument);
}

/**
 * @brief Validates Mahalanobis distance logic returning higher risk closer to the target.
 */
TEST_F(PredictionTest, DynamicRiskField_CalculatesProperRisk) {
    types::Point safe_point(0.0, 0.0, 0.0);
    types::Point risky_point(10.0, 0.0, 0.0); // Exactly on target 1 position

    double safe_risk = prediction::get_dynamic_risk_field(safe_point, 0.0, usv_entity, context_targets);
    double high_risk = prediction::get_dynamic_risk_field(risky_point, 0.0, usv_entity, context_targets);

    // Risk at the exact target location should be extremely high compared to far away
    EXPECT_GT(high_risk, safe_risk);
    
    // Risk at the center of the covariance should be close to 1.0 (exponential of 0)
    EXPECT_NEAR(high_risk, 1.0, 0.001);
}