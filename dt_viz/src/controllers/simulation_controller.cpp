#include "dt_viz/controllers/simulation_controller.hpp"
#include <cmath>

namespace dt_viz {

SimulationController::SimulationController(std::shared_ptr<dt::DigitalTwinCore> dt_core, QObject * parent)
: QObject(parent),
  dt_core_(std::move(dt_core)),
  timer_(new QTimer(this))
{
  connect(timer_, &QTimer::timeout, this, &SimulationController::processTick);
  timer_->start(33); 
}

void SimulationController::processTick() {
  if (!dt_core_) return;

  auto snapshot = dt_core_->get_latest_state();
  if (!snapshot) return;

  const types::Entity usv_state = snapshot->get_vehicle_state();

  // update position
  double usv_x = usv_state.get_pose().get_x();
  double usv_y = usv_state.get_pose().get_y();
  double heading_deg = -usv_state.get_pose().get_yaw() * (180.0 / M_PI);

  double usv_velocity = std::hypot(usv_state.get_velocity().get_vx(), usv_state.get_velocity().get_vy());

  emit usvPoseUpdated(usv_x, usv_y, heading_deg);

  // update usv predicted trajectory
  double time_horizon = 15.0;
  double time_step = 1.0;
  auto usv_predicted_traj = snapshot->predict_trajectory(usv_state, time_horizon, time_step);
  emit usvPredictedTrajectoryUpdated(usv_predicted_traj);

  // update targets
  const auto targets = snapshot->get_all_targets();
  emit targetsUpdated(targets);

  // update target predicted trajectory
  std::vector<types::Trajectory> targets_predicted_trajs;
  targets_predicted_trajs.reserve(targets.size());
  
  for (const auto& target : targets) {
    targets_predicted_trajs.push_back(snapshot->predict_trajectory(target, time_horizon, time_step));

    // update approaching
    double alert_radius = 1000.0;
    bool is_approaching = snapshot->is_target_approaching(usv_state, target, alert_radius);
    emit targetApproachingUpdated(target.get_id(), is_approaching);
  }
  
  emit targetsPredictedTrajectoriesUpdated(targets_predicted_trajs);

  // update collision points
  double speed_profile = std::max(usv_velocity, 0.1); 

  float alert_radius = 3000.0f; 
  auto local_targets = snapshot->get_active_local_targets(usv_state.get_pose().get_position(), alert_radius);

  auto collision_reports = snapshot->check_collisions_on_trajectory(usv_predicted_traj, usv_state, speed_profile, local_targets, 0.0);

  std::vector<RoutePoint> collision_points;

  for (const auto& report : collision_reports) {
      if (!report.is_safe()) {
          emit collisionAlertUpdated(report.get_id(), true);
          RoutePoint p;
          p.x = report.get_usv_cpa().get_x();
          p.y = -report.get_usv_cpa().get_y();
          collision_points.push_back(p);
      } else {
          emit collisionAlertUpdated(report.get_id(), false);
      }
  }
  
  emit collisionPointsUpdated(collision_points);

  // update planned route
  const types::Trajectory planned_traj = snapshot->get_planned_trajectory();
  const auto & core_waypoints = planned_traj.get_poses();

  std::vector<RoutePoint> display_route;
  display_route.reserve(core_waypoints.size());

  for (const auto & wp : core_waypoints) {
    RoutePoint rp;
    rp.x = wp.get_x();
    rp.y = -wp.get_y();
    display_route.push_back(rp);
  }

  emit plannedRouteUpdated(display_route);

  QString status_msg = QString("USV: x=%1 m | y=%2 m | heading=%3° | vessels=%4 | Vel=%5 m/s")
                         .arg(usv_x, 0, 'f', 1)
                         .arg(usv_y, 0, 'f', 1)
                         .arg(heading_deg, 0, 'f', 1)
                         .arg(targets.size())
                         .arg(usv_velocity, 0, 'f', 1);
                         
  emit simulationStatusUpdated(status_msg);
}

} // namespace dt_viz