/**
 * @file prediction.hpp
 * @brief Prediction methods for planning support
 */

#pragma once
#include "dt_core/types.hpp"

namespace prediction{
  /**
   * @brief Predicts the trajectory of a specific entity over a given time horizon.
   * @param entity Current entity to predict trajectory.
   * @param time_horizon Total prediction time in seconds.
   * @param time_step Time increment between prediction steps.
   * @return Predicted trajectory for the specified entity.
   */
  types::Trajectory predict_trajectory(const types::Entity& entity, const double time_horizon, const double time_step);
  
  /**
   * @brief Predicts the trajectory of a specific target over a given time horizon.
   * @param id The ID of the target to predict.
   * @param targets Current list of context targets.
   * @param time_horizon Total prediction time in seconds.
   * @param time_step Time increment between prediction steps.
   * @return Predicted trajectory for the specified target.
   */
  types::Trajectory predict_trajectory_by_id(const int32_t id, const std::vector<types::Target>& targets, const double time_horizon, const double time_step);

  /**
   * @brief Discretizes a trajectory between two points in space.
   * @param origin Starting point.
   * @param destination Ending point.
   * @param step Distance between discrete inner trajectory points.
   * @return Trajectory between origin and destination.
   */
  types::Trajectory make_trajectory_between(const types::Point& origin, const types::Point& destination, const double step);

  /**
   * @brief Evaluates a potential collision along a candidate trajectory against a known target.
   * @param candidate_trajectory The planned route to evaluate.
   * @param usv_state Current kinematics and pose of the Ego Vehicle.
   * @param speed_profile Assumed speed profile along the route.
   * @param target Surrounding dynamic target.
   * @param start_time Evaluation start offset.
   * @return Detailed report containing CPA (Closest Point of Approach) metrics.
   */
  types::TargetCollisionReport check_collision_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, const double speed_profile, const types::Target& target, const double start_time);

  /**
   * @brief Evaluates potential collisions along a candidate trajectory against known targets.
   * @param candidate_trajectory The planned route to evaluate.
   * @param usv_state Current kinematics and pose of the Ego Vehicle.
   * @param speed_profile Assumed speed profile along the route.
   * @param targets Surrounding dynamic targets.
   * @param start_time Evaluation start offset.
   * @return List of detailed report containing CPA (Closest Point of Approach) metrics.
   */
  std::vector<types::TargetCollisionReport> check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, const double speed_profile, const std::vector<types::Target>& targets, const double start_time = 0.0);
  
  /**
   * @brief Calculates a dynamic risk value for a given position based on target kinematics.
   * @param position Point in space to evaluate.
   * @param usv_covariance Ego vehicle covariance.
   * @param target Dynamic environmental target.
   * @param timestamp Time offset for the risk evaluation.
   * @return Risk factor (higher indicates greater collision probability).
   */
  double get_dynamic_risk(const types::Point& position, const types::Covariance& usv_covariance, const types::Target& target, const double timestamp);

  /**
   * @brief Calculates the max dynamic risk value for a given position based on targets kinematics.
   * @param position Point in space to evaluate.
   * @param usv_covariance Ego vehicle covariance.
   * @param targets Dynamic environmental targets.
   * @param timestamp Time offset for the risk evaluation.
   * @return Max Risk factor (higher indicates greater collision probability).
   */
  double get_max_dynamic_risk(const types::Point& position, const types::Covariance& usv_covariance, const std::vector<types::Target>& targets, const double timestamp);

  /**
   * @brief Checks if a given target is approaching to Ego vihecle.
   * @param usv Current kinematics and pose of the Ego Vehicle.
   * @param target Current kinematics and pose of the Target.
   * @param alert_radius Analysis radius.
   * @return Whether the target is approaching or not.
   */
  bool is_target_approaching(const types::Entity& usv, const types::Target& target, double alert_radius);
  
} // namespace prediction
