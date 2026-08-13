#include "dt_viz/main_window.hpp"

#include <QColor>
#include <QFont>
#include <QHBoxLayout>
#include <QStatusBar>
#include <QVBoxLayout>

namespace dt_viz {

void MainWindow::configureWindow(){
  setWindowTitle("CEIA DGS - Visual Diagnostic Tool");
  resize(1200, 750);

  scene_->setBackgroundBrush(QColor(255, 235, 215));

  view_->setRenderHint(QPainter::Antialiasing);
  view_->setDragMode(QGraphicsView::ScrollHandDrag);
  view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

  auto * main_layout = new QHBoxLayout(central_widget_);

  main_layout->setContentsMargins(5, 5, 5, 5);
  main_layout->setSpacing(8);

  main_layout->addWidget(view_, 1);
  main_layout->addWidget(information_panel_);

  setCentralWidget(central_widget_);

  statusBar()->showMessage("Provisional interface | Top view | Simulated data");

  view_->viewport()->installEventFilter(this);
}

void MainWindow::createInformationPanel(){
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

  auto * title = new QLabel("USV Diagnosis");
  QFont title_font;
  title_font.setBold(true);
  title_font.setPointSize(14);
  title->setFont(title_font);

  auto * usv_section = new QLabel("USV");
  QFont section_font;
  section_font.setBold(true);
  section_font.setPointSize(11);
  usv_section->setFont(section_font);

  usv_position_label_ = new QLabel("Position:\nx = 0.0 m\ny = 0.0 m");
  heading_label_ = new QLabel("Heading: 0.0°");
  vessel_count_label_ = new QLabel("Monitored vessels: 3");

  auto * legend_title = new QLabel("Label");
  legend_title->setFont(section_font);

  auto * legend = new QLabel(
    "▲  USV\n"
    "●  Monitored vessels\n"
    "━  Free zone boundary\n"
    "··· USV Trajectory"
  );

  simulation_status_label_ = new QLabel("Status: active simulation");
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

  // Creation of the re-centering button
  recenter_button_ = new QPushButton("Recenter USV");
  recenter_button_->setCursor(Qt::PointingHandCursor);
  recenter_button_->setStyleSheet(
    "QPushButton {"
    "  background-color: #144696;"
    "  color: white;"
    "  border-radius: 4px;"
    "  padding: 6px;"
    "  font-weight: bold;"
    "}"
    "QPushButton:hover {"
    "  background-color: #2864be;"
    "}"
    "QPushButton:pressed {"
    "  background-color: #0a2d66;"
    "}"
  );

  layout->addWidget(recenter_button_);

  // Connects the button click to tracking flag
  connect(recenter_button_, &QPushButton::clicked, this, [this]() {
      is_tracking_usv_ = true;
      if (usv_) {
          view_->centerOn(usv_->pos());
      }
  });

  layout->addStretch();
  layout->addWidget(simulation_status_label_);
}

void MainWindow::createScene(){
  drawFreeZone();
  drawUsv();
  drawScaleBar();
}

} // namespace dt_viz