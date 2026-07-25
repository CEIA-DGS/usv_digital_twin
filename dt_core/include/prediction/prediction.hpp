#pragma once
#include "dt_core/types.hpp"

namespace prediction{
  types::Trajectory predict_trajectory(const types::Entity& entity, const double time_horizon, const double time_step);
  
  types::Trajectory predict_trajectory_by_id(const int32_t id, const std::vector<types::Target>& targets, const double time_horizon, const double time_step);

  types::Trajectory make_trajectory_between(const types::Point& origin, const types::Point& destination, const double step);

  types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, const double start_time = 0.0);
  
  double get_dynamic_risk_field(const types::Point& position, const double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets);
} // namespace prediction
