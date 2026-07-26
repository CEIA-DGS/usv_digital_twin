#include "dt_viz/main_window.hpp"

#include <QBrush>
#include <QColor>
#include <QFont>
#include <QHBoxLayout>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QStatusBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

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

  scene_->setBackgroundBrush(QColor(225, 240, 250));

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
    700.0
  );

  drawGrid();
  drawAxes();
  drawFreeZone();
  drawUsv();
  drawScaleBar();
}

void MainWindow::drawGrid()
{
  QPen grid_pen(QColor(190, 205, 215, 130));
  grid_pen.setWidthF(0.4);

  constexpr int spacing = 50;

  for (int x = -500; x <= 500; x += spacing) {
    scene_->addLine(
      x,
      -350,
      x,
      350,
      grid_pen
    );
  }

  for (int y = -350; y <= 350; y += spacing) {
    scene_->addLine(
      -500,
      y,
      500,
      y,
      grid_pen
    );
  }
}

void MainWindow::drawAxes()
{
  QPen axis_pen(QColor(90, 105, 115));
  axis_pen.setWidthF(1.2);

  scene_->addLine(
    -500,
    0,
    500,
    0,
    axis_pen
  );

  scene_->addLine(
    0,
    -350,
    0,
    350,
    axis_pen
  );

  auto * origin_label = scene_->addSimpleText("(0,0)");

  origin_label->setBrush(
    QBrush(QColor(70, 80, 90))
  );

  origin_label->setPos(7.0, 7.0);
  origin_label->setZValue(4.0);
}

void MainWindow::drawFreeZone()
{
  QPolygonF polygon;

  polygon
    << QPointF(-420.0, -250.0)
    << QPointF(-230.0, -305.0)
    << QPointF(30.0, -290.0)
    << QPointF(350.0, -200.0)
    << QPointF(420.0, 30.0)
    << QPointF(350.0, 240.0)
    << QPointF(100.0, 300.0)
    << QPointF(-170.0, 270.0)
    << QPointF(-390.0, 150.0);

  QPen border_pen(QColor(25, 120, 80));
  border_pen.setWidthF(3.0);

  QBrush fill_brush(
    QColor(110, 205, 165, 75)
  );

  free_zone_ = scene_->addPolygon(
    polygon,
    border_pen,
    fill_brush
  );

  free_zone_->setZValue(-1.0);

  auto * label = scene_->addSimpleText(
    "Zona livre"
  );

  QFont label_font;
  label_font.setBold(true);

  label->setFont(label_font);

  label->setBrush(
    QBrush(QColor(25, 100, 65))
  );

  label->setPos(-390.0, -235.0);
  label->setZValue(1.0);
}

void MainWindow::drawUsv()
{
  QPolygonF shape;

  shape
    << QPointF(24.0, 0.0)
    << QPointF(-18.0, -14.0)
    << QPointF(-10.0, 0.0)
    << QPointF(-18.0, 14.0);

  QPen usv_pen(QColor(20, 70, 150));
  usv_pen.setWidthF(2.0);

  QBrush usv_brush(QColor(60, 135, 235));

  usv_ = scene_->addPolygon(
    shape,
    usv_pen,
    usv_brush
  );

  usv_->setPos(0.0, 0.0);
  usv_->setZValue(5.0);

  usv_label_ = scene_->addSimpleText("USV");

  QFont label_font;
  label_font.setBold(true);

  usv_label_->setFont(label_font);

  usv_label_->setBrush(
    QBrush(QColor(20, 55, 125))
  );

  usv_label_->setPos(-12.0, 20.0);
  usv_label_->setZValue(6.0);

  QPen heading_pen(QColor(20, 70, 150));
  heading_pen.setWidthF(2.0);
  heading_pen.setStyle(Qt::DashLine);

  heading_line_ = scene_->addLine(
    0.0,
    0.0,
    65.0,
    0.0,
    heading_pen
  );

  heading_line_->setZValue(4.0);

  QPen trajectory_pen(QColor(40, 100, 190));
  trajectory_pen.setWidthF(2.0);
  trajectory_pen.setStyle(Qt::DotLine);

  trajectory_item_ = scene_->addPath(
    trajectory_path_,
    trajectory_pen
  );

  trajectory_item_->setZValue(2.0);
}

// Desenho da barra de escala
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

  scale_label->setBrush(
    QBrush(QColor(40, 50, 60))
  );

  scale_label->setPos(
    start_x + 10.0,
    start_y - 25.0
  );
}

void MainWindow::updateSimulation()
{
  if (!dt_core_) return;

  // 1. Pega a snapshot mais recente do mundo
  auto snapshot = dt_core_->get_latest_state();
  if (!snapshot) return;

  // 2. Lê a Pose do USV
  const types::Pose usv_pose = snapshot->get_vehicle_pose();
  const double usv_x = usv_pose.get_x();
  const double usv_y = usv_pose.get_y();
  // Converte radianos para graus para o Qt (e inverte o sentido se necessário)
  const double heading = usv_pose.get_yaw() * (180.0 / M_PI); 

  // Atualiza os desenhos do USV
  usv_->setPos(usv_x, usv_y);
  usv_->setRotation(heading);
  usv_label_->setPos(usv_x - 12.0, usv_y + 20.0);
  heading_line_->setPos(usv_x, usv_y);
  heading_line_->setRotation(heading);
  updateUsvTrajectory(usv_x, usv_y);

  // 3. Lê e Atualiza os Alvos AIS dinamicamente
  const auto targets = snapshot->get_all_targets();
  
  for (const auto& target : targets) {
    std::uint32_t mmsi = target.get_id();
    double t_x = target.get_pose().get_x();
    double t_y = target.get_pose().get_y();

    // Se o alvo não existe na tela, nós o criamos dinamicamente
    if (vessel_items_by_mmsi_.find(mmsi) == vessel_items_by_mmsi_.end()) {
      auto * vessel = scene_->addEllipse(-11.0, -11.0, 22.0, 22.0,
        QPen(QColor(160, 55, 35), 2.0), normalVesselBrush());
      vessel->setZValue(4.0);
      scene_->addItem(vessel);
      vessel_items_by_mmsi_[mmsi] = vessel;

      auto * label = scene_->addSimpleText(QString("MMSI %1").arg(mmsi));
      label->setBrush(QBrush(QColor(125, 45, 30)));
      label->setZValue(5.0);
      scene_->addItem(label);
      vessel_labels_by_mmsi_[mmsi] = label;
    }

    // Atualiza a posição da bolinha e do texto
    vessel_items_by_mmsi_[mmsi]->setPos(t_x, t_y);
    vessel_labels_by_mmsi_[mmsi]->setPos(t_x + 16.0, t_y - 18.0);
    
    // Pede para o snapshot calcular se há risco de colisão real
    // auto report = snapshot->check_collisions_on_trajectory(..., target, ...);
    // updateCollisionAlert(mmsi, !report.is_safe());
  }

  // 4. Atualiza os painéis de texto
  updateInformationPanel(usv_x, usv_y, heading);
  statusBar()->showMessage(
    QString("USV: x=%1 m | y=%2 m | heading=%3° | embarcações=%4")
      .arg(usv_x, 0, 'f', 1)
      .arg(usv_y, 0, 'f', 1)
      .arg(heading, 0, 'f', 1)
      .arg(targets.size())
  );
}

void MainWindow::updateUsvTrajectory(
  double x,
  double y)
{
  trajectory_points_.emplace_back(x, y);

  constexpr std::size_t maximum_points = 250;

  if (trajectory_points_.size() > maximum_points) {
    trajectory_points_.erase(
      trajectory_points_.begin()
    );
  }

  trajectory_path_ = QPainterPath();

  if (!trajectory_points_.empty()) {
    trajectory_path_.moveTo(
      trajectory_points_.front()
    );

    for (std::size_t i = 1;
      i < trajectory_points_.size();
      ++i)
    {
      trajectory_path_.lineTo(
        trajectory_points_[i]
      );
    }
  }

  trajectory_item_->setPath(
    trajectory_path_
  );
}

void MainWindow::updateInformationPanel(
  double usv_x,
  double usv_y,
  double heading)
{
  usv_position_label_->setText(
    QString(
      "Posição:\n"
      "x = %1 m\n"
      "y = %2 m"
    )
      .arg(usv_x, 0, 'f', 1)
      .arg(usv_y, 0, 'f', 1)
  );

  heading_label_->setText(
    QString("Heading: %1°")
      .arg(heading, 0, 'f', 1)
  );

  vessel_count_label_->setText(
    QString("Embarcações monitoradas: %1")
      .arg(vessels_.size())
  );
}

void MainWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);

  if (view_ != nullptr && scene_ != nullptr) {
    view_->fitInView(
      scene_->sceneRect().adjusted(
        -20.0,
        -20.0,
        20.0,
        20.0
      ),
      Qt::KeepAspectRatio
    );
  }
}

// Atualização da derrota
void MainWindow::updatePlannedRoute(
  const std::vector<RoutePoint> & route)
{
  planned_route_ = route;
  drawPlannedRoute();
}

// Limpeza da rota anterior
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

// Desenho da derrota planejada
void MainWindow::drawPlannedRoute()
{
  clearPlannedRoute();

  if (planned_route_.empty()) {
    return;
  }

  QPainterPath route_path;

  route_path.moveTo(
    planned_route_.front().x,
    planned_route_.front().y
  );

  for (
    std::size_t index = 1;
    index < planned_route_.size();
    ++index)
  {
    route_path.lineTo(
      planned_route_[index].x,
      planned_route_[index].y
    );
  }

  QPen route_pen(QColor(130, 65, 200));
  route_pen.setWidthF(3.0);
  route_pen.setStyle(Qt::DashLine);

  planned_route_item_ = scene_->addPath(
    route_path,
    route_pen
  );

  planned_route_item_->setZValue(3.0);

  for (
    std::size_t index = 0;
    index < planned_route_.size();
    ++index)
  {
    const RoutePoint & point = planned_route_[index];

    auto * waypoint = scene_->addEllipse(
      -6.0,
      -6.0,
      12.0,
      12.0,
      QPen(QColor(85, 35, 145), 2.0),
      QBrush(QColor(180, 120, 235))
    );

    waypoint->setPos(point.x, point.y);
    waypoint->setZValue(4.0);

    waypoint_items_.push_back(waypoint);

    auto * label = scene_->addSimpleText(
      QString("WP%1").arg(index + 1)
    );

    label->setPos(
      point.x + 8.0,
      point.y - 20.0
    );

    label->setBrush(
      QBrush(QColor(85, 35, 145))
    );

    label->setZValue(5.0);

    waypoint_labels_.push_back(label);
  }
}

// Aparência dos alvos
QBrush MainWindow::normalVesselBrush() const
{
  return QBrush(QColor(235, 105, 75));
}

QBrush MainWindow::collisionVesselBrush() const
{
  return QBrush(QColor(230, 25, 25));
}

// Destaque de colisão
void MainWindow::updateCollisionAlert(
  std::uint32_t mmsi,
  bool collision_imminent)
{
  const auto vessel_iterator =
    vessel_items_by_mmsi_.find(mmsi);

  if (vessel_iterator == vessel_items_by_mmsi_.end()) {
    return;
  }

  QGraphicsEllipseItem * vessel =
    vessel_iterator->second;

  const auto label_iterator =
    vessel_labels_by_mmsi_.find(mmsi);

  if (collision_imminent) {
    vessel->setBrush(collisionVesselBrush());

    vessel->setPen(
      QPen(QColor(120, 0, 0), 4.0)
    );

    vessel->setScale(1.5);

    if (label_iterator != vessel_labels_by_mmsi_.end()) {
      label_iterator->second->setText(
        QString(
          "ALERTA - MMSI %1\nRISCO DE COLISÃO"
        ).arg(mmsi)
      );

      label_iterator->second->setBrush(
        QBrush(QColor(185, 0, 0))
      );
    }

    return;
  }

  vessel->setBrush(normalVesselBrush());

  vessel->setPen(
    QPen(QColor(160, 55, 35), 2.0)
  );

  vessel->setScale(1.0);

  if (label_iterator != vessel_labels_by_mmsi_.end()) {
    label_iterator->second->setText(
      QString("MMSI %1").arg(mmsi)
    );

    label_iterator->second->setBrush(
      QBrush(QColor(125, 45, 30))
    );
  }
}