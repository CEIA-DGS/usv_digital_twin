#include "../../include/prediction/prediction.hpp"
#include <stdexcept>
#include <cmath>

namespace prediction{
  types::Trajectory predict_trajectory(const types::Entity& entity, const double time_horizon, const double time_step){
    /* Prevê a trajetória linear de um objeto 'time_horizon' segundos no futuro a um passo de tempo de 'time_step' segundos */

    if(time_horizon <= 0.0){
      throw std::invalid_argument("Error: time_horizon must be positive!");
    }
    if(time_step <= 0.0){
      throw std::invalid_argument("Error: time_step must be positive!");
    }
    if(time_step > time_horizon){
      throw std::invalid_argument("Error: time_horizon must be greater than time_step!");
    }


    types::Trajectory predicted_trajectory = types::Trajectory();

    types::Pose initial_pose = entity.get_pose();
    types::Velocity velocity = entity.get_velocity();
    types::Pose future_pose = initial_pose;
    for(double t=0.0; t <= time_horizon; t += time_step){
      double future_x = initial_pose.get_x() + velocity.get_vx() * t;
      double future_y = initial_pose.get_y() + velocity.get_vy() * t;
      double future_z = initial_pose.get_z();

      future_pose.set_position(future_x, future_y, future_z);

      predicted_trajectory.add_pose(future_pose);
    }
    return predicted_trajectory;
  }

  types::Trajectory predict_trajectory_by_id(const int32_t id, const std::vector<types::Target>& targets, const double time_horizon, const double time_step){
    if (id < 0) {
      throw std::invalid_argument("Error: Target ID must be positive!");
    }
    if (targets.empty()){
      throw std::invalid_argument("Error: Targets list must be not empty!");
    }

    // search for target
    const types::Target* target_of_interest = nullptr;
    for(const auto& target : targets){
      if(target.get_id() == static_cast<uint32_t>(id)){
        target_of_interest = &target;
        break;
      }
    }

    if(target_of_interest == nullptr){
      throw std::invalid_argument("Error: Target not found");
    }

    return predict_trajectory(*target_of_interest, time_horizon, time_step);
  }

  types::Trajectory make_trajectory_between(const types::Point& origin, const types::Point& destination, const double step){
    if(step <= 0.0){
      throw std::invalid_argument("Error: step must be positive!");
    }

    types::Trajectory trajectory = types::Trajectory();

    double dx = destination.get_x() - origin.get_x();
    double dy = destination.get_y() - origin.get_y();
    double distance = std::hypot(dx, dy);

    if(distance <= 0){
      throw std::invalid_argument("Error: origin and destination must be different!");
    }

    if(step > distance){
      throw std::invalid_argument("Error: distance between waypoints must be greater than step!");
    }

    double num_segments = distance / step;
    
    types::Pose pose = types::Pose();

    // garante inclusão da origem e destino evitando erros por arredondamento
    pose.set_position(origin.get_x(), origin.get_y(), 0.0);
    trajectory.add_pose(pose);

    for(double s = 1.0; s < num_segments; s += 1.0){
      double x = origin.get_x() + ((s * dx) / num_segments);
      double y = origin.get_y() + ((s * dy) / num_segments);
      pose.set_position(x, y, 0.0);
      trajectory.add_pose(pose);
    }

    pose.set_position(origin.get_x(), origin.get_y(), 0.0);
    trajectory.add_pose(pose);

    return trajectory;
  }

  types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, const double start_time){
    types::TargetCollisionReport report;

    if(candidate_trajectory.empty()){
      throw std::invalid_argument("Error: canditade trajectory cant be empy!");
    }
    if(candidate_trajectory.size() < 2){
      throw std::invalid_argument("Error: candidate trajectory must have at least 2 points!");
    }
    if(speed_profile <= 0){
      throw std::invalid_argument("Error: speed profile must be positive!");
    }

    double dx = candidate_trajectory.get_pose_by_index(1).get_x() - candidate_trajectory.get_pose_by_index(0).get_x();
    double dy = candidate_trajectory.get_pose_by_index(1).get_y() - candidate_trajectory.get_pose_by_index(0).get_y();
    double dh = std::hypot(dx, dy);
    double dt = dh / speed_profile;
    double current_time = start_time;

    for(const auto& pose : candidate_trajectory.get_poses()){
      for(const auto& target :  targets){
        double target_start_x = target.get_pose().get_x();
        double target_start_y = target.get_pose().get_y();
        double target_current_x = target_start_x + (target.get_velocity().get_vx() * current_time);
        double target_current_y = target_start_y + (target.get_velocity().get_vy() * current_time);

        double relative_x = pose.get_x() - target_current_x;
        double relative_y = pose.get_y() - target_current_y;
        double relative_distance = std::hypot(relative_x, relative_y);

        if(relative_distance < report.get_dcpa() || report.get_dcpa() == -1.0){
          report.set_dcpa(relative_distance);
          report.set_tcpa(current_time);
          report.set_usv_cpa(pose);

          types::Pose current_target_pose = target.get_pose();
          current_target_pose.set_position(target_current_x, target_current_y, current_target_pose.get_z());
          report.set_obstacle_cpa(current_target_pose);
          report.set_id(target.get_id());
        }

        // Fusão de Covariância
        double cov_xx = target.get_covariance().get_xx() + usv_state.get_covariance().get_xx() + 1.0;
        double cov_yy = target.get_covariance().get_yy() + usv_state.get_covariance().get_yy() + 1.0;
        double cov_xy = target.get_covariance().get_xy() + usv_state.get_covariance().get_xy();

        // Inversão da Matriz 2x2 para a Distância de Mahalanobis
        double det = (cov_xx * cov_yy) - (cov_xy * cov_xy);
        if (det < 1e-6) det = 1e-6; // Evita matriz singular ou divisão por zero

        double inv_xx = cov_yy / det;
        double inv_yy = cov_xx / det;
        double inv_xy = -cov_xy / det;

        // D_M^2 = p_rel^T * Sigma_rel^-1 * p_rel
        double d_m_sq = (relative_x * inv_xx + relative_y * inv_xy) * relative_x + 
                        (relative_x * inv_xy + relative_y * inv_yy) * relative_y;

        // Conversão geométrica de Mahalanobis para campo escalar de Risco
        // Função decaimento exponencial Gaussiana: Risk = exp(-0.5 * D_M^2)
        double risk = std::exp(-0.5 * d_m_sq);
        
        if(risk > report.get_risk()){
          report.set_risk(risk);
        }

        double threshold = 0.55; // % de certeza de colisão 0 a 1
        if(risk > threshold){
          report.set_safety(false);
          report.set_id(target.get_id());
          report.set_msg("Warning: Collision along the trajectory!");
          return report;
        }
      }
      current_time += dt;
    }
    report.set_msg("Report: Trajectory is safe!");
    return report;
  }

  double get_dynamic_risk_field(const types::Point& position, const double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets){
    double max_risk = -1.0;
    for(const auto& target :  targets){
      double target_start_x = target.get_pose().get_x();
      double target_start_y = target.get_pose().get_y();

      double target_future_x = target_start_x + (target.get_velocity().get_vx() * timestamp);
      double target_future_y = target_start_y + (target.get_velocity().get_vy() * timestamp);

      double relative_x = position.get_x() - target_future_x;
      double relative_y = position.get_y() - target_future_y;

      // Fusão de Covariância
      double cov_xx = target.get_covariance().get_xx() + usv_state.get_covariance().get_xx() + 1.0;
      double cov_yy = target.get_covariance().get_yy() + usv_state.get_covariance().get_yy() + 1.0;
      double cov_xy = target.get_covariance().get_xy() + usv_state.get_covariance().get_xy();

      // Inversão da Matriz 2x2 para a Distância de Mahalanobis
      double det = (cov_xx * cov_yy) - (cov_xy * cov_xy);
      if (det < 1e-6) det = 1e-6; // Evita matriz singular ou divisão por zero

      double inv_xx = cov_yy / det;
      double inv_yy = cov_xx / det;
      double inv_xy = -cov_xy / det;

      // D_M^2 = p_rel^T * Sigma_rel^-1 * p_rel
      double d_m_sq = (relative_x * inv_xx + relative_y * inv_xy) * relative_x + 
                      (relative_x * inv_xy + relative_y * inv_yy) * relative_y;

      // Conversão geométrica de Mahalanobis para campo escalar de Risco
      // Função decaimento exponencial Gaussiana: Risk = exp(-0.5 * D_M^2)
      double risk = std::exp(-0.5 * d_m_sq);

      if(risk > max_risk){
        max_risk = risk;
      }
    }
    return max_risk;
  }
} // namespace prediction
