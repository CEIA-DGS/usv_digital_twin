#include "dt_viz/main_window.hpp"
#include "dt_viz/shapefile_loader.hpp"

#include <QBrush>
#include <QColor>
#include <QFont>
#include <QHBoxLayout>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QTransform>
#include <filesystem>

#include <algorithm>
#include <cmath>

#include <ogrsf_frmts.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

MainWindow::MainWindow(std::shared_ptr<dt::DigitalTwinCore> dt_core, QWidget * parent)
: QMainWindow(parent),
  dt_core_(std::move(dt_core)), 
  scene_(new QGraphicsScene(this)),
  view_(new QGraphicsView(scene_, this)),
  timer_(new QTimer(this)),
  central_widget_(new QWidget(this)),
  information_panel_(new QWidget(this)),
  usv_position_label_(nullptr),
  heading_label_(nullptr),
  vessel_count_label_(nullptr),
  simulation_status_label_(nullptr),
  free_zone_(nullptr),
  usv_(nullptr),
  usv_label_(nullptr),
  heading_line_(nullptr),
  trajectory_item_(nullptr),
  planned_route_item_(nullptr),
  simulation_time_(0.0)
{
  configureWindow();
  createInformationPanel();
  createScene();

  connect(
    timer_,
    &QTimer::timeout,
    this,
    [this]() {
      updateSimulation(); 
    });

  timer_->start(33); 
}

void MainWindow::configureWindow()
{
  setWindowTitle("CEIA DGS - Ferramenta de Diagnóstico Visual");
  resize(1200, 750);

  // Cor de fundo para as áreas não navegáveis (Laranja claro / Areia)
  scene_->setBackgroundBrush(QColor(255, 235, 215));

  view_->setRenderHint(QPainter::Antialiasing);
  view_->setDragMode(QGraphicsView::ScrollHandDrag);
  view_->setTransformationAnchor(
    QGraphicsView::AnchorUnderMouse
  );

  auto * main_layout = new QHBoxLayout(central_widget_);

  main_layout->setContentsMargins(5, 5, 5, 5);
  main_layout->setSpacing(8);

  main_layout->addWidget(view_, 1);
  main_layout->addWidget(information_panel_);

  setCentralWidget(central_widget_);

  statusBar()->showMessage(
    "Interface provisória | Vista superior | Dados simulados"
  );

  // Instala o filtro de eventos no viewport do QGraphicsView para habilitar o Zoom do mouse
  view_->viewport()->installEventFilter(this);
}

void MainWindow::createInformationPanel()
{
  information_panel_->setFixedWidth(250);

  information_panel_->setStyleSheet(
    "QWidget {"
    "  background-color: #f4f6f8;"
    "  color: #1f2933;"
    "}"
    "QLabel {"
    "  padding: 4px;"
    "}"
  );

  auto * layout = new QVBoxLayout(information_panel_);

  auto * title = new QLabel("Diagnóstico do USV");
  QFont title_font;
  title_font.setBold(true);
  title_font.setPointSize(14);
  title->setFont(title_font);

  auto * usv_section = new QLabel("USV");
  QFont section_font;
  section_font.setBold(true);
  section_font.setPointSize(11);
  usv_section->setFont(section_font);

  usv_position_label_ = new QLabel("Posição:\nx = 0.0 m\ny = 0.0 m");
  heading_label_ = new QLabel("Heading: 0.0°");
  vessel_count_label_ = new QLabel("Embarcações monitoradas: 3");

  auto * legend_title = new QLabel("Legenda");
  legend_title->setFont(section_font);

  auto * legend = new QLabel(
    "▲  USV\n"
    "●  Embarcação monitorada\n"
    "━  Limite da zona livre\n"
    "··· Trajetória do USV"
  );

  simulation_status_label_ = new QLabel(
    "Estado: simulação ativa"
  );

  simulation_status_label_->setStyleSheet(
    "color: #16784b;"
    "font-weight: bold;"
  );

  layout->addWidget(title);
  layout->addSpacing(10);

  layout->addWidget(usv_section);
  layout->addWidget(usv_position_label_);
  layout->addWidget(heading_label_);
  layout->addWidget(vessel_count_label_);

  layout->addSpacing(20);
  layout->addWidget(legend_title);
  layout->addWidget(legend);

  layout->addStretch();
  layout->addWidget(simulation_status_label_);
}

void MainWindow::createScene()
{
  scene_->setSceneRect(
    -500.0,
    -350.0,
    1000.0,
    700.0);

  drawGrid();
  drawAxes();

  // A zona livre fixa foi substituída pela carta náutica.
  // drawFreeZone();

  drawUsv();
  drawTrackedVessels();
  drawScaleBar();
}

void MainWindow::drawGrid() {} // Inutilizado para o modo UTM real
void MainWindow::drawAxes() {} // Inutilizado para o modo UTM real

void MainWindow::drawFreeZone()
{
  GDALAllRegister();
  
  GDALAllRegister();
  
  // Busca automaticamente a raiz do pacote dt_core no computador atual
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
    // INVERSÃO 1: O Centro geométrico ganha o Y invertido
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
          // INVERSÃO 2: As coordenadas da malha ganham o Y invertido
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
  // Mantém o tamanho fixo independentemente do zoom
  usv_->setFlag(QGraphicsItem::ItemIgnoresTransformations);

  usv_label_ = scene_->addSimpleText("USV");
  QFont label_font; 
  label_font.setBold(true);
  usv_label_->setFont(label_font);
  usv_label_->setBrush(QBrush(QColor(20, 55, 125)));
  usv_label_->setZValue(6.0);
  // Mantém o tamanho fixo e desloca o texto da origem em pixels via QTransform
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

  scene_->addLine(
    start_x,
    start_y,
    start_x + scale_length,
    start_y,
    scale_pen
  );

  scene_->addLine(
    start_x,
    start_y - 5.0,
    start_x,
    start_y + 5.0,
    scale_pen
  );

  scene_->addLine(
    start_x + scale_length,
    start_y - 5.0,
    start_x + scale_length,
    start_y + 5.0,
    scale_pen
  );

  auto * scale_label = scene_->addSimpleText("50 m");
  scale_label->setBrush(QBrush(QColor(40, 50, 60)));
  scale_label->setPos(start_x + 10.0, start_y - 25.0);
}

void MainWindow::updateSimulation()
{
  if (!dt_core_) return;

  auto snapshot = dt_core_->get_latest_state();
  if (!snapshot) return;

  const types::Pose usv_pose = snapshot->get_vehicle_pose();
  double usv_x = usv_pose.get_x();
  double usv_y = usv_pose.get_y();
  
  // INVERSÃO DO ÂNGULO: O Yaw do ROS (Anti-horário) precisa ser invertido para o Qt (Horário)
  const double heading = -usv_pose.get_yaw() * (180.0 / M_PI); 

  // HACK: Se o barco ainda estiver na origem (0,0) por falta de sinal GPS,
  // teleportamos ele visualmente para o centro da carta náutica.
  if (std::abs(usv_x) < 0.1 && std::abs(usv_y) < 0.1) {
      usv_x = map_center_x_;
      // map_center_y_ já está negativo; revertemos para a lógica UTM manter a coerência
      usv_y = -map_center_y_; 
  }

  // VARIÁVEIS DE TELA (O Qt desenha invertido)
  const double render_x = usv_x;
  const double render_y = -usv_y;

  // Atualiza as posições do USV na cena usando os valores invertidos
  usv_->setPos(render_x, render_y);
  usv_->setRotation(heading);
  usv_label_->setPos(render_x, render_y); 
  heading_line_->setPos(render_x, render_y);
  heading_line_->setRotation(heading);
  
  updateUsvTrajectory(render_x, render_y);

  // Lê e atualiza os Alvos AIS
  const auto targets = snapshot->get_all_targets();
  
  for (const auto& target : targets) {
    std::uint32_t mmsi = target.get_id();
    double t_x = target.get_pose().get_x();
    double t_y = target.get_pose().get_y();

    // VARIÁVEIS DE TELA PARA OS ALVOS
    double render_t_x = t_x;
    double render_t_y = -t_y;

    // Cria visualização do alvo se ele ainda não existe na cena
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

    // Atualiza a posição no Qt
    vessel_items_by_mmsi_[mmsi]->setPos(render_t_x, render_t_y);
    vessel_labels_by_mmsi_[mmsi]->setPos(render_t_x, render_t_y);
  }

  const types::Trajectory planned_traj = snapshot->get_planned_trajectory();
  const auto& core_waypoints = planned_traj.get_poses();

  // 2. Converte para o RoutePoint que o Qt espera, invertendo o Y!
  std::vector<RoutePoint> display_route;
  display_route.reserve(core_waypoints.size());

  for (const auto& wp : core_waypoints) {
      RoutePoint rp;
      rp.x = wp.get_x();
      rp.y = -wp.get_y(); // INVERSÃO CRÍTICA PARA O QT
      display_route.push_back(rp);
  }

  // 3. Manda desenhar na tela
  updatePlannedRoute(display_route);

  // Atualiza painel lateral e barra de status
  updateInformationPanel(usv_x, usv_y, heading);
  statusBar()->showMessage(
    QString("USV: x=%1 m | y=%2 m | heading=%3° | embarcações=%4")
      .arg(usv_x, 0, 'f', 1)
      .arg(usv_y, 0, 'f', 1)
      .arg(heading, 0, 'f', 1)
      .arg(targets.size())
  );
}

void MainWindow::updateUsvTrajectory(double x, double y)
{
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

void MainWindow::updateInformationPanel(double usv_x, double usv_y, double heading)
{
  usv_position_label_->setText(
    QString("Posição:\nx = %1 m\ny = %2 m")
      .arg(usv_x, 0, 'f', 1)
      .arg(usv_y, 0, 'f', 1)
  );

  heading_label_->setText(
    QString("Heading: %1°").arg(heading, 0, 'f', 1)
  );

  vessel_count_label_->setText(
    QString("Embarcações monitoradas: %1").arg(vessel_items_by_mmsi_.size())
  );
}

void MainWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);
  // view_->fitInView() propositalmente removido para não conflitar com UTM
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
  // Intercepta eventos do scroll apenas na área de desenho para dar zoom
  if (watched == view_->viewport() && event->type() == QEvent::Wheel) {
    QWheelEvent *wheel_event = static_cast<QWheelEvent *>(event);
    
    if (wheel_event->angleDelta().y() > 0) {
      view_->scale(1.15, 1.15); // Zoom In
    } else {
      view_->scale(1.0 / 1.15, 1.0 / 1.15); // Zoom Out
    }
    
    return true; 
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::updatePlannedRoute(const std::vector<RoutePoint> & route)
{
  planned_route_ = route;
  drawPlannedRoute();
}

void MainWindow::clearPlannedRoute()
{
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

void MainWindow::drawPlannedRoute()
{
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

QBrush MainWindow::normalVesselBrush() const
{
  return QBrush(QColor(235, 105, 75));
}

QBrush MainWindow::collisionVesselBrush() const
{
  return QBrush(QColor(230, 25, 25));
}

void MainWindow::updateCollisionAlert(std::uint32_t mmsi, bool collision_imminent)
{
  const auto vessel_iterator = vessel_items_by_mmsi_.find(mmsi);
  if (vessel_iterator == vessel_items_by_mmsi_.end()) {
    return;
  }

  QGraphicsEllipseItem * vessel = vessel_iterator->second;
  const auto label_iterator = vessel_labels_by_mmsi_.find(mmsi);

  if (collision_imminent) {
    vessel->setBrush(collisionVesselBrush());
    vessel->setPen(QPen(QColor(120, 0, 0), 4.0));
    
    // Como estamos usando ItemIgnoresTransformations, o setScale afeta apenas 
    // os limites do vetor do próprio objeto, e não interfere no zoom da cena.
    vessel->setScale(1.5); 

    if (label_iterator != vessel_labels_by_mmsi_.end()) {
      label_iterator->second->setText(QString("ALERTA - MMSI %1\nRISCO DE COLISÃO").arg(mmsi));
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

void MainWindow::clearNauticalChart()
{
  for (auto * item : nautical_chart_items_) {
    if (item == nullptr) {
      continue;
    }

    scene_->removeItem(item);
    delete item;
  }

  nautical_chart_items_.clear();
}

bool MainWindow::loadNauticalChartDirectory(
  const std::string & directory)
{
  namespace fs = std::filesystem;

  clearNauticalChart();

  /**
   * Cada camada recebe uma aparência distinta.
   *
   * A ordem visual é definida pelo z_value:
   * quanto maior o valor, mais acima a camada será desenhada.
   */
  struct LayerDefinition
  {
    std::string file_name;
    QPen pen;
    QBrush brush;
    double z_value;
  };

  const std::vector<LayerDefinition> layer_definitions = {
    {
      "3_Area_Navegavel_Limpa.shp",
      QPen(QColor(75, 145, 185), 1.0),
      QBrush(QColor(155, 215, 235, 130)),
      -40.0
    },
    {
      "1_Terra_Firme.shp",
      QPen(QColor(90, 75, 50), 1.5),
      QBrush(QColor(184, 160, 115, 230)),
      -30.0
    },
    {
      "2_Margem_Seguranca.shp",
      QPen(QColor(210, 145, 20), 1.5),
      QBrush(QColor(255, 205, 70, 105)),
      -20.0
    },
    {
      "4_Malha_NavMesh.shp",
      QPen(QColor(70, 90, 110, 150), 0.8),
      QBrush(Qt::NoBrush),
      -10.0
    }
  };

  struct LoadedLayer
  {
    LayerDefinition definition;
    std::vector<ShapefileGeometry> geometries;
  };

  std::vector<LoadedLayer> loaded_layers;

  QRectF source_bounds;
  bool has_source_bounds = false;

  for (const auto & definition : layer_definitions) {
    const fs::path shapefile_path =
      fs::path(directory) /
      definition.file_name;

    if (!fs::exists(shapefile_path)) {
      statusBar()->showMessage(
        QString(
          "Arquivo não encontrado: %1")
          .arg(
            QString::fromStdString(
              shapefile_path.string())));

      continue;
    }

    std::vector<ShapefileGeometry> geometries =
      ShapefileLoader::load(
        shapefile_path.string());

    if (geometries.empty()) {
      continue;
    }

    for (const auto & geometry : geometries) {
      if (!has_source_bounds) {
        source_bounds = geometry.bounds;
        has_source_bounds = true;
      } else {
        source_bounds =
          source_bounds.united(
            geometry.bounds);
      }
    }

    loaded_layers.push_back({
      definition,
      std::move(geometries)
    });
  }

  if (!has_source_bounds || loaded_layers.empty()) {
    statusBar()->showMessage(
      "Nenhuma geometria válida foi encontrada na carta.");

    return false;
  }

  if (source_bounds.width() <= 0.0 ||
      source_bounds.height() <= 0.0)
  {
    statusBar()->showMessage(
      "Os limites da carta náutica são inválidos.");

    return false;
  }

  // Área reservada da cena para desenhar a carta.
  const QRectF target_bounds =
    scene_->sceneRect().adjusted(
      35.0,
      35.0,
      -35.0,
      -35.0);

  const double scale_x =
    target_bounds.width() /
    source_bounds.width();

  const double scale_y =
    target_bounds.height() /
    source_bounds.height();

  // Mantém a proporção da carta.
  const double scale =
    std::min(scale_x, scale_y);

  const double mapped_width =
    source_bounds.width() * scale;

  const double mapped_height =
    source_bounds.height() * scale;

  const double offset_x =
    target_bounds.left() +
    (target_bounds.width() - mapped_width) / 2.0;

  const double offset_y =
    target_bounds.top() +
    (target_bounds.height() - mapped_height) / 2.0;

  /**
   * Transformação comum para todas as camadas.
   *
   * Isso garante que terra firme, margem, área navegável
   * e NavMesh permaneçam espacialmente alinhadas.
   */
  const QTransform chart_transform(
    scale,
    0.0,
    0.0,
    scale,
    offset_x - source_bounds.left() * scale,
    offset_y - source_bounds.top() * scale);

  std::size_t item_count = 0;

  for (const auto & layer : loaded_layers) {
    for (const auto & geometry : layer.geometries) {
      const QPainterPath transformed_path =
        chart_transform.map(
          geometry.path);

      auto * item = scene_->addPath(
        transformed_path,
        layer.definition.pen,
        layer.definition.brush);

      item->setZValue(
        layer.definition.z_value);

      nautical_chart_items_.push_back(item);
      ++item_count;
    }
  }

  if (item_count == 0) {
    statusBar()->showMessage(
      "A carta foi lida, mas nenhuma geometria foi desenhada.");

    return false;
  }

  view_->fitInView(
    scene_->sceneRect(),
    Qt::KeepAspectRatio);

  statusBar()->showMessage(
    QString(
      "Carta náutica carregada: %1 elementos gráficos.")
      .arg(item_count));

  return true;
}