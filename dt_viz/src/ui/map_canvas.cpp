#include "dt_viz/ui/map_canvas.hpp"

#include <QPen>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QTransform>
#include <cmath>
#include <unordered_set>

#include <ogrsf_frmts.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace dt_viz {

MapCanvas::MapCanvas(QWidget * parent)
: QGraphicsView(parent),
  scene_(new NavigationScene(this)),
  free_zone_(nullptr),
  usv_(nullptr),
  usv_label_(nullptr),
  heading_line_(nullptr),
  trajectory_item_(nullptr),
  planned_route_item_(nullptr)
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
  drawScaleBar();
}

void MapCanvas::wheelEvent(QWheelEvent * event) {
  if (event->angleDelta().y() > 0) {
    scale(1.15, 1.15); 
  } else {
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
  
  heading_line_->setPos(render_x, render_y);
  heading_line_->setRotation(heading_deg);
  
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
  
  for (const auto & target : targets) {
    std::uint32_t mmsi = target.get_id();
    active_mmsis.insert(mmsi);

    double render_t_x = target.get_pose().get_x();
    double render_t_y = -target.get_pose().get_y();

    if (vessel_items_by_mmsi_.find(mmsi) == vessel_items_by_mmsi_.end()) {
      auto * vessel = scene_->addEllipse(-11.0, -11.0, 22.0, 22.0,
        QPen(QColor(160, 55, 35), 2.0), normalVesselBrush());
      
      vessel->setZValue(4.0);
      vessel->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      vessel_items_by_mmsi_[mmsi] = vessel;

      auto * label = scene_->addSimpleText(QString("MMSI %1").arg(mmsi));
      label->setBrush(QBrush(QColor(125, 45, 30)));
      label->setZValue(5.0);
      label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      label->setTransform(QTransform().translate(16.0, -18.0));
      vessel_labels_by_mmsi_[mmsi] = label;
    }

    vessel_items_by_mmsi_[mmsi]->setPos(render_t_x, render_t_y);
    vessel_labels_by_mmsi_[mmsi]->setPos(render_t_x, render_t_y);
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
  auto vessel_iterator = vessel_items_by_mmsi_.find(mmsi);
  if (vessel_iterator == vessel_items_by_mmsi_.end()) {
    return;
  }

  QGraphicsEllipseItem * vessel = vessel_iterator->second;
  auto label_iterator = vessel_labels_by_mmsi_.find(mmsi);

  if (collision_imminent) {
    vessel->setBrush(collisionVesselBrush());
    vessel->setPen(QPen(QColor(120, 0, 0), 4.0));
    vessel->setRect(-16.5, -16.5, 33.0, 33.0); 

    if (label_iterator != vessel_labels_by_mmsi_.end()) {
      label_iterator->second->setText(QString("ALERT - MMSI %1\nCOLLISION RISK").arg(mmsi));
      label_iterator->second->setBrush(QBrush(QColor(185, 0, 0)));
    }
    return;
  }

  vessel->setBrush(normalVesselBrush());
  vessel->setPen(QPen(QColor(160, 55, 35), 2.0));
  vessel->setRect(-11.0, -11.0, 22.0, 22.0);

  if (label_iterator != vessel_labels_by_mmsi_.end()) {
    label_iterator->second->setText(QString("MMSI %1").arg(mmsi));
    label_iterator->second->setBrush(QBrush(QColor(125, 45, 30)));
  }
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

  QPen heading_pen(QColor(20, 70, 150));
  heading_pen.setWidthF(2.0);
  heading_pen.setStyle(Qt::DashLine);

  heading_line_ = scene_->addLine(0.0, 0.0, 65.0, 0.0, heading_pen);
  heading_line_->setZValue(4.0);
  heading_line_->setFlag(QGraphicsItem::ItemIgnoresTransformations);

  QPen trajectory_pen(QColor(40, 100, 190));
  trajectory_pen.setWidthF(2.0); 
  trajectory_pen.setStyle(Qt::DotLine);
  trajectory_item_ = scene_->addPath(trajectory_path_, trajectory_pen);
  trajectory_item_->setZValue(2.0);
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

QBrush MapCanvas::normalVesselBrush() const {
  return QBrush(QColor(235, 105, 75));
}

QBrush MapCanvas::collisionVesselBrush() const {
  return QBrush(QColor(230, 25, 25));
}

} // namespace dt_viz