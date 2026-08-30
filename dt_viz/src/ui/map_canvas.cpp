#include "dt_viz/ui/map_canvas.hpp"

#include <QPen>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QTransform>
#include <QDateTime>
#include <cmath>

#include <ogrsf_frmts.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace dt_viz {

MapCanvas::MapCanvas(QWidget * parent)
: QGraphicsView(parent),
  scene_(new NavigationScene(this)),
  free_zone_(nullptr),
  usv_(nullptr),
  usv_label_(nullptr),
  predicted_usv_route_item_(nullptr),
  trajectory_item_(nullptr),
  planned_route_item_(nullptr),
  targets_predicted_routes_item_(nullptr),
  collision_stars_item_(nullptr)
{
  setScene(scene_);
  setRenderHint(QPainter::Antialiasing);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  scene_->setBackgroundBrush(QColor(255, 235, 215));

  setupScene();
  scale(3.0, 3.0);
}

void MapCanvas::setupScene() {
  drawFreeZone();
  drawUsv();
  drawUsvPrediction();
  drawTargetsPrediction();
  drawCollisionPoint();
  drawScaleBar();
}

void MapCanvas::wheelEvent(QWheelEvent * event) {
  if (event->angleDelta().y() > 0) {
    // ZOOM IN
    constexpr double max_scale = 25.0; 
    double current_scale = transform().m11();
    
    if (current_scale * 1.15 <= max_scale) {
      scale(1.15, 1.15); 
    } else {
      double adjust_factor = max_scale / current_scale;
      scale(adjust_factor, adjust_factor);
    }
  } else {
    // ZOOM OUT
    QRectF scene_rect = scene_->sceneRect();
    QRectF view_rect = viewport()->rect();
    
    double min_scale_x = view_rect.width() / scene_rect.width();
    double min_scale_y = view_rect.height() / scene_rect.height();
    double min_scale = std::min(min_scale_x, min_scale_y);
    
    double current_scale = transform().m11();
    double next_scale = current_scale / 1.15;
    
    if (next_scale <= min_scale) {
      fitInView(scene_rect, Qt::KeepAspectRatio);
    } else {
      scale(1.0 / 1.15, 1.0 / 1.15); 
    }
  }
  
  updateGridIndicator();
}

void MapCanvas::mousePressEvent(QMouseEvent * event) {
  if (event->button() == Qt::LeftButton) {
    is_tracking_usv_ = false;
    emit trackingInterrupted();
  }
  QGraphicsView::mousePressEvent(event);
}

void MapCanvas::updateGridIndicator() {
  const double current_scale = transform().m11();
  const double current_grid_step = scene_->calculateGridStep(current_scale);
  emit gridResolutionChanged(current_grid_step);
}

void MapCanvas::updateUsvPose(double x, double y, double heading_deg) {
  double render_x = x;
  double render_y = -y;

  // Fallback to center if origin is uninitialized
  if (std::abs(render_x) < 0.1 && std::abs(render_y) < 0.1) {
    render_x = map_center_x_;
    render_y = map_center_y_; 
  }

  usv_->setPos(render_x, render_y);
  usv_->setRotation(heading_deg);

  if (is_tracking_usv_) {
    centerOn(render_x, render_y);
  }
  
  usv_label_->setPos(render_x, render_y); 
  
  trajectory_points_.emplace_back(render_x, render_y);

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

void MapCanvas::centerOnUsv() {
  is_tracking_usv_ = true;

  if (usv_) {
    centerOn(usv_->pos());
  }
}

void MapCanvas::updateTargets(const std::vector<types::Target> & targets) {
  std::unordered_set<std::uint32_t> active_mmsis;
  
  // blink
  long long current_time = QDateTime::currentMSecsSinceEpoch();
  long blink_period = 400;
  bool blink_on = (current_time / blink_period) % 2 == 0; 
  
  for (const auto & target : targets) {
    std::uint32_t mmsi = target.get_id();
    active_mmsis.insert(mmsi);

    double render_t_x = target.get_pose().get_x();
    double render_t_y = -target.get_pose().get_y();

    // create new target
    if (vessel_items_by_mmsi_.find(mmsi) == vessel_items_by_mmsi_.end()) {
      auto * vessel = scene_->addEllipse(-11.0, -11.0, 22.0, 22.0, QPen(), QBrush());
      vessel->setZValue(4.0);
      vessel->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      vessel_items_by_mmsi_[mmsi] = vessel;

      auto * label = scene_->addSimpleText("");
      label->setZValue(5.0);
      label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      label->setTransform(QTransform().translate(16.0, -18.0));
      vessel_labels_by_mmsi_[mmsi] = label;
    }

    // update position
    auto* vessel = vessel_items_by_mmsi_[mmsi];
    auto* label = vessel_labels_by_mmsi_[mmsi];
    vessel->setPos(render_t_x, render_t_y);
    label->setPos(render_t_x, render_t_y);
    
    // update style
    if (collision_mmsis_.count(mmsi)) {
      // Imminent Collision
      vessel->setBrush(collisionVesselBrush());
      vessel->setPen(QPen(QColor(120, 0, 0), 4.0));
      vessel->setRect(-16.5, -16.5, 33.0, 33.0); 
      label->setText(QString("ALERT - MMSI %1\nCOLLISION RISK").arg(mmsi));
      label->setBrush(QBrush(QColor(185, 0, 0)));
      
    } else if (approaching_mmsis_.count(mmsi)) {
      // Approaching
      if (blink_on) {
        vessel->setBrush(QBrush(QColor(255, 180, 0))); // Laranja
        vessel->setPen(QPen(QColor(200, 100, 0), 3.0));
      } else {
        vessel->setBrush(normalVesselBrush());
        vessel->setPen(QPen(QColor(160, 55, 35), 2.0));
      }
      vessel->setRect(-11.0, -11.0, 22.0, 22.0);
      label->setText(QString("MMSI %1").arg(mmsi));
      label->setBrush(QBrush(QColor(125, 45, 30)));
      
    } else {
      // Normal
      vessel->setBrush(normalVesselBrush());
      vessel->setPen(QPen(QColor(160, 55, 35), 2.0));
      vessel->setRect(-11.0, -11.0, 22.0, 22.0);
      label->setText(QString("MMSI %1").arg(mmsi));
      label->setBrush(QBrush(QColor(125, 45, 30)));
    }
  }

  // Purge inactive targets
  for (auto it = vessel_items_by_mmsi_.begin(); it != vessel_items_by_mmsi_.end(); ) {
    std::uint32_t mmsi = it->first;
    if (active_mmsis.find(mmsi) == active_mmsis.end()) {
      scene_->removeItem(it->second);
      delete it->second;
      
      auto label_it = vessel_labels_by_mmsi_.find(mmsi);
      if (label_it != vessel_labels_by_mmsi_.end()) {
        scene_->removeItem(label_it->second);
        delete label_it->second;
        vessel_labels_by_mmsi_.erase(label_it);
      }
      
      collision_mmsis_.erase(mmsi);
      approaching_mmsis_.erase(mmsi);
      
      it = vessel_items_by_mmsi_.erase(it);
    } else {
      ++it;
    }
  }
}

void MapCanvas::updatePlannedRoute(const std::vector<RoutePoint> & route) {
  clearPlannedRoute();
  if (route.empty()) {
    return;
  }

  QPainterPath route_path;
  route_path.moveTo(route.front().x, route.front().y);

  for (std::size_t index = 1; index < route.size(); ++index) {
    route_path.lineTo(route[index].x, route[index].y);
  }

  QPen route_pen(QColor(130, 65, 200));
  route_pen.setWidthF(3.0);
  route_pen.setStyle(Qt::DashLine);

  planned_route_item_ = scene_->addPath(route_path, route_pen);
  planned_route_item_->setZValue(3.0);

  for (std::size_t index = 0; index < route.size(); ++index) {
    const RoutePoint & point = route[index];

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

void MapCanvas::clearPlannedRoute() {
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

void MapCanvas::setCollisionAlert(std::uint32_t mmsi, bool collision_imminent) {
  if (collision_imminent) collision_mmsis_.insert(mmsi);
  else collision_mmsis_.erase(mmsi);
}

void MapCanvas::setApproachingAlert(std::uint32_t mmsi, bool approaching) {
  if (approaching) approaching_mmsis_.insert(mmsi);
  else approaching_mmsis_.erase(mmsi);
}

void MapCanvas::drawFreeZone() {
  GDALAllRegister();
  
  std::string dt_core_path = ament_index_cpp::get_package_share_directory("dt_core");
  std::string shapefile_path = dt_core_path + "/data/output/NavMesh_Shapefiles_BR501511/4_Malha_NavMesh.shp";
  
  GDALDataset* ds = (GDALDataset*)GDALOpenEx(shapefile_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);

  if (!ds) {
    qWarning("Unable to load the Shapefile for quick preview. Check the path.");
    return;
  }

  QPen border_pen(QColor(100, 160, 220, 100)); 
  border_pen.setWidthF(0.5);
  QBrush fill_brush(QColor(175, 215, 250, 120)); 

  OGRLayer* layer = ds->GetLayer(0);
  OGREnvelope envelope;

  if (layer->GetExtent(&envelope) == OGRERR_NONE) {
    map_center_x_ = (envelope.MinX + envelope.MaxX) / 2.0;
    map_center_y_ = -((envelope.MinY + envelope.MaxY) / 2.0); 
    
    centerOn(map_center_x_, map_center_y_);

    double width = envelope.MaxX - envelope.MinX;
    double height = envelope.MaxY - envelope.MinY;
    
    scene_->setSceneRect(envelope.MinX, -envelope.MaxY, width, height);
  }

  OGRFeature* feat;
  layer->ResetReading();

  while ((feat = layer->GetNextFeature()) != nullptr) {
    OGRGeometry* geom = feat->GetGeometryRef();
    if (geom && wkbFlatten(geom->getGeometryType()) == wkbPolygon) {
      OGRLinearRing* ring = ((OGRPolygon*)geom)->getExteriorRing();
      if (ring) {
        QPolygonF qpoly;
        for (int i = 0; i < ring->getNumPoints(); i++) {
          qpoly << QPointF(ring->getX(i), -ring->getY(i));
        }
        auto* item = scene_->addPolygon(qpoly, border_pen, fill_brush);
        item->setZValue(-1.0);
      }
    }
    OGRFeature::DestroyFeature(feat);
  }
  GDALClose(ds);
}

void MapCanvas::drawUsv() {
  QPolygonF shape;
  shape << QPointF(24.0, 0.0) << QPointF(-18.0, -14.0) << QPointF(-10.0, 0.0) << QPointF(-18.0, 14.0);

  QPen usv_pen(QColor(20, 70, 150)); 
  usv_pen.setWidthF(2.0);
  QBrush usv_brush(QColor(60, 135, 235));

  usv_ = scene_->addPolygon(shape, usv_pen, usv_brush);
  usv_->setPos(0.0, 0.0);
  usv_->setZValue(5.0);
  usv_->setFlag(QGraphicsItem::ItemIgnoresTransformations);

  usv_label_ = scene_->addSimpleText("USV");
  QFont label_font; 
  label_font.setBold(true);
  usv_label_->setFont(label_font);
  usv_label_->setBrush(QBrush(QColor(20, 55, 125)));
  usv_label_->setZValue(6.0);
  usv_label_->setFlag(QGraphicsItem::ItemIgnoresTransformations);
  usv_label_->setTransform(QTransform().translate(-12.0, 20.0));


  QPen trajectory_pen(QColor(40, 100, 190));
  trajectory_pen.setWidthF(2.0); 
  trajectory_pen.setStyle(Qt::DotLine);
  trajectory_item_ = scene_->addPath(trajectory_path_, trajectory_pen);
  trajectory_item_->setZValue(2.0);
}

void MapCanvas::drawUsvPrediction(){
  QPen predicted_pen(QColor(20, 70, 150));
  predicted_pen.setWidthF(3.0);
  predicted_pen.setStyle(Qt::DashLine);
  predicted_pen.setCosmetic(true);

  predicted_usv_route_item_ = scene_->addPath(QPainterPath(), predicted_pen);
  predicted_usv_route_item_->setZValue(4.0);
}

void MapCanvas::drawTargetsPrediction() {
  QPen targets_pred_pen(QColor(235, 105, 75)); 
  targets_pred_pen.setWidthF(3.0);
  targets_pred_pen.setStyle(Qt::DashLine);
  targets_pred_pen.setCosmetic(true);

  targets_predicted_routes_item_ = scene_->addPath(QPainterPath(), targets_pred_pen);
  targets_predicted_routes_item_->setZValue(3.5);
}

void MapCanvas::drawCollisionPoint() {
  QPen star_pen(QColor(255, 0, 0), 2.0);
  star_pen.setCosmetic(true);
  QBrush star_brush(QColor(255, 50, 50, 200));

  collision_stars_item_ = scene_->addPath(QPainterPath(), star_pen, star_brush);
  collision_stars_item_->setZValue(6.0);
}

void MapCanvas::drawScaleBar() {
  constexpr double scale_length = 50.0;
  QPen scale_pen(QColor(40, 50, 60));
  scale_pen.setWidthF(3.0);

  const double start_x = 360.0;
  const double start_y = 310.0;

  scene_->addLine(start_x, start_y, start_x + scale_length, start_y, scale_pen);
  scene_->addLine(start_x, start_y - 5.0, start_x, start_y + 5.0, scale_pen);
  scene_->addLine(start_x + scale_length, start_y - 5.0, start_x + scale_length, start_y + 5.0, scale_pen);

  auto * scale_label = scene_->addSimpleText("50 m");
  scale_label->setBrush(QBrush(QColor(40, 50, 60)));
  scale_label->setPos(start_x + 10.0, start_y - 25.0);
}

void MapCanvas::updateUsvPredictedTrajectory(const types::Trajectory& traj) {
  QPainterPath path;
  bool first = true;
  for (const auto& pose : traj.get_poses()) {
    if (first) {
      path.moveTo(pose.get_x(), -pose.get_y());
      first = false;
    } else {
      path.lineTo(pose.get_x(), -pose.get_y());
    }
  }
  predicted_usv_route_item_->setPath(path);
}

void MapCanvas::updateTargetsPredictedTrajectories(const std::vector<types::Trajectory>& trajs) {
  QPainterPath combined_path;
  
  for (const auto& traj : trajs) {
    bool first = true;
    for (const auto& pose : traj.get_poses()) {
      if (first) {
        combined_path.moveTo(pose.get_x(), -pose.get_y());
        first = false;
      } else {
        combined_path.lineTo(pose.get_x(), -pose.get_y());
      }
    }
  }
  
  targets_predicted_routes_item_->setPath(combined_path);
}

void MapCanvas::updateCollisionPoints(const std::vector<RoutePoint>& points) {
  QPainterPath path;
  
  for (const auto& pt : points) {
    QPolygonF star;
    for (int i = 0; i < 10; ++i) {
      double r = (i % 2 == 0) ? 25.0 : 10.0;
      double angle = i * (M_PI / 5.0) - (M_PI / 2.0);
      star << QPointF(pt.x + r * std::cos(angle), pt.y + r * std::sin(angle));
    }
    path.addPolygon(star);
  }
  
  collision_stars_item_->setPath(path);
}

QBrush MapCanvas::normalVesselBrush() const {
  return QBrush(QColor(235, 105, 75));
}

QBrush MapCanvas::collisionVesselBrush() const {
  return QBrush(QColor(230, 25, 25));
}

} // namespace dt_viz