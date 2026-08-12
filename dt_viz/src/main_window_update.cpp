#include "dt_viz/main_window.hpp"

#include <QPen>
#include <QBrush>
#include <QTransform>
#include <QString>
#include <QStatusBar>
#include <cmath>

namespace dt_viz {

void MainWindow::updateSimulation(){
  if (!dt_core_) return;

  auto snapshot = dt_core_->get_latest_state();
  if (!snapshot) return;

  const types::Pose usv_pose = snapshot->get_vehicle_pose();
  double usv_x = usv_pose.get_x();
  double usv_y = usv_pose.get_y();
  
  const double heading = -usv_pose.get_yaw() * (180.0 / M_PI); 

  if (std::abs(usv_x) < 0.1 && std::abs(usv_y) < 0.1) {
      usv_x = map_center_x_;
      usv_y = -map_center_y_; 
  }

  const double render_x = usv_x;
  const double render_y = -usv_y;

  usv_->setPos(render_x, render_y);
  usv_->setRotation(heading);
  usv_label_->setPos(render_x, render_y); 
  heading_line_->setPos(render_x, render_y);
  heading_line_->setRotation(heading);
  
  updateUsvTrajectory(render_x, render_y);

  const auto targets = snapshot->get_all_targets();
  
  for (const auto& target : targets) {
    std::uint32_t mmsi = target.get_id();
    double t_x = target.get_pose().get_x();
    double t_y = target.get_pose().get_y();

    double render_t_x = t_x;
    double render_t_y = -t_y;

    if (vessel_items_by_mmsi_.find(mmsi) == vessel_items_by_mmsi_.end()) {
      auto * vessel = scene_->addEllipse(-11.0, -11.0, 22.0, 22.0,
        QPen(QColor(160, 55, 35), 2.0), normalVesselBrush());
      vessel->setZValue(4.0);
      vessel->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      scene_->addItem(vessel);
      vessel_items_by_mmsi_[mmsi] = vessel;

      auto * label = scene_->addSimpleText(QString("MMSI %1").arg(mmsi));
      label->setBrush(QBrush(QColor(125, 45, 30)));
      label->setZValue(5.0);
      label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      label->setTransform(QTransform().translate(16.0, -18.0));
      scene_->addItem(label);
      vessel_labels_by_mmsi_[mmsi] = label;
    }

    vessel_items_by_mmsi_[mmsi]->setPos(render_t_x, render_t_y);
    vessel_labels_by_mmsi_[mmsi]->setPos(render_t_x, render_t_y);
  }

  const types::Trajectory planned_traj = snapshot->get_planned_trajectory();
  const auto& core_waypoints = planned_traj.get_poses();

  std::vector<RoutePoint> display_route;
  display_route.reserve(core_waypoints.size());

  for (const auto& wp : core_waypoints) {
      RoutePoint rp;
      rp.x = wp.get_x();
      rp.y = -wp.get_y(); 
      display_route.push_back(rp);
  }

  updatePlannedRoute(display_route);
  updateInformationPanel(usv_x, usv_y, heading);
  
  statusBar()->showMessage(
    QString("USV: x=%1 m | y=%2 m | heading=%3° | embarcações=%4")
      .arg(usv_x, 0, 'f', 1)
      .arg(usv_y, 0, 'f', 1)
      .arg(heading, 0, 'f', 1)
      .arg(targets.size())
  );
}

void MainWindow::updateUsvTrajectory(double x, double y){
  trajectory_points_.emplace_back(x, y);

  constexpr std::size_t maximum_points = 250;
  if (trajectory_points_.size() > maximum_points) {
    trajectory_points_.erase(trajectory_points_.begin());
  }

  trajectory_path_ = QPainterPath();

  if (!trajectory_points_.empty()) {
    trajectory_path_.moveTo(trajectory_points_.front());
    for (std::size_t i = 1; i < trajectory_points_.size(); ++i) {
      trajectory_path_.lineTo(trajectory_points_[i]);
    }
  }

  trajectory_item_->setPath(trajectory_path_);
}

void MainWindow::updateInformationPanel(double usv_x, double usv_y, double heading){
  usv_position_label_->setText(
    QString("Posição:\nx = %1 m\ny = %2 m")
      .arg(usv_x, 0, 'f', 1)
      .arg(usv_y, 0, 'f', 1)
  );

  heading_label_->setText(
    QString("Heading: %1°").arg(heading, 0, 'f', 1)
  );

  vessel_count_label_->setText(
    QString("Monitored vessels: %1").arg(vessel_items_by_mmsi_.size())
  );
}

void MainWindow::updateCollisionAlert(std::uint32_t mmsi, bool collision_imminent){
  const auto vessel_iterator = vessel_items_by_mmsi_.find(mmsi);
  if (vessel_iterator == vessel_items_by_mmsi_.end()) {
    return;
  }

  QGraphicsEllipseItem * vessel = vessel_iterator->second;
  const auto label_iterator = vessel_labels_by_mmsi_.find(mmsi);

  if (collision_imminent) {
    vessel->setBrush(collisionVesselBrush());
    vessel->setPen(QPen(QColor(120, 0, 0), 4.0));
    vessel->setScale(1.5); 

    if (label_iterator != vessel_labels_by_mmsi_.end()) {
      label_iterator->second->setText(QString("ALERT - MMSI %1\nCOLLISION RISK").arg(mmsi));
      label_iterator->second->setBrush(QBrush(QColor(185, 0, 0)));
    }
    return;
  }

  vessel->setBrush(normalVesselBrush());
  vessel->setPen(QPen(QColor(160, 55, 35), 2.0));
  vessel->setScale(1.0);

  if (label_iterator != vessel_labels_by_mmsi_.end()) {
    label_iterator->second->setText(QString("MMSI %1").arg(mmsi));
    label_iterator->second->setBrush(QBrush(QColor(125, 45, 30)));
  }
}

} // namespace dt_viz