#include "dt_viz/main_window.hpp"

#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QString>
#include <QTransform>

namespace dt_viz {

void MainWindow::updatePlannedRoute(const std::vector<RoutePoint> & route){
  planned_route_ = route;
  drawPlannedRoute();
}

void MainWindow::clearPlannedRoute(){
  if (planned_route_item_ != nullptr) {
    scene_->removeItem(planned_route_item_);
    delete planned_route_item_;
    planned_route_item_ = nullptr;
  }

  for (auto * waypoint : waypoint_items_) {
    scene_->removeItem(waypoint);
    delete waypoint;
  }
  waypoint_items_.clear();

  for (auto * label : waypoint_labels_) {
    scene_->removeItem(label);
    delete label;
  }
  waypoint_labels_.clear();
}

void MainWindow::drawPlannedRoute(){
  clearPlannedRoute();

  if (planned_route_.empty()) {
    return;
  }

  QPainterPath route_path;
  route_path.moveTo(planned_route_.front().x, planned_route_.front().y);

  for (std::size_t index = 1; index < planned_route_.size(); ++index) {
    route_path.lineTo(planned_route_[index].x, planned_route_[index].y);
  }

  QPen route_pen(QColor(130, 65, 200));
  route_pen.setWidthF(3.0);
  route_pen.setStyle(Qt::DashLine);

  planned_route_item_ = scene_->addPath(route_path, route_pen);
  planned_route_item_->setZValue(3.0);

  for (std::size_t index = 0; index < planned_route_.size(); ++index) {
    const RoutePoint & point = planned_route_[index];

    auto * waypoint = scene_->addEllipse(
      -6.0, -6.0, 12.0, 12.0,
      QPen(QColor(85, 35, 145), 2.0),
      QBrush(QColor(180, 120, 235))
    );

    waypoint->setPos(point.x, point.y);
    waypoint->setZValue(4.0);
    waypoint->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    waypoint_items_.push_back(waypoint);

    auto * label = scene_->addSimpleText(QString("WP%1").arg(index + 1));
    label->setPos(point.x, point.y);
    label->setBrush(QBrush(QColor(85, 35, 145)));
    label->setZValue(5.0);
    label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    label->setTransform(QTransform().translate(8.0, -20.0));
    waypoint_labels_.push_back(label);
  }
}

} // namespace dt_viz