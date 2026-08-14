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

  const types::Pose usv_pose = snapshot->get_vehicle_pose();
  double usv_x = usv_pose.get_x();
  double usv_y = usv_pose.get_y();
  double heading_deg = -usv_pose.get_yaw() * (180.0 / M_PI);

  emit usvPoseUpdated(usv_x, usv_y, heading_deg);

  const auto targets = snapshot->get_all_targets();
  emit targetsUpdated(targets);

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

  QString status_msg = QString("USV: x=%1 m | y=%2 m | heading=%3° | vessels=%4")
                         .arg(usv_x, 0, 'f', 1)
                         .arg(usv_y, 0, 'f', 1)
                         .arg(heading_deg, 0, 'f', 1)
                         .arg(targets.size());
                         
  emit simulationStatusUpdated(status_msg);
}

} // namespace dt_viz