#include "dt_viz/main_window.hpp"

#include <QBrush>
#include <QColor>
#include <QPen>
#include <QPolygonF>
#include <QTransform>
#include <QFont>

#include <ogrsf_frmts.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace dt_viz {

void MainWindow::drawGrid() {} 
void MainWindow::drawAxes() {} 

void MainWindow::drawFreeZone()
{
  GDALAllRegister();
  
  std::string dt_core_path = ament_index_cpp::get_package_share_directory("dt_core");
  std::string caminho = dt_core_path + "/data/output/NavMesh_Shapefiles_BR501511/4_Malha_NavMesh.shp";
  
  GDALDataset* ds = (GDALDataset*)GDALOpenEx(caminho.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);

  if (!ds) {
    qWarning("Não foi possível carregar o Shapefile para visualização rápida. Verifique o caminho.");
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
    
    view_->centerOn(map_center_x_, map_center_y_);
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

void MainWindow::drawUsv()
{
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

void MainWindow::drawScaleBar()
{
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

QBrush MainWindow::normalVesselBrush() const
{
  return QBrush(QColor(235, 105, 75));
}

QBrush MainWindow::collisionVesselBrush() const
{
  return QBrush(QColor(230, 25, 25));
}

} // namespace dt_viz