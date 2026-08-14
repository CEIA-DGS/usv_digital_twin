#include "dt_viz/ui/telemetry_panel.hpp"

#include <QVBoxLayout>
#include <QFont>

namespace dt_viz {

TelemetryPanel::TelemetryPanel(QWidget * parent) 
: QWidget(parent) 
{
  setupUi();
}

void TelemetryPanel::setupUi() {
  setFixedWidth(250);

  setStyleSheet(
    "QWidget {"
    "  background-color: #f4f6f8;"
    "  color: #1f2933;"
    "}"
    "QLabel {"
    "  padding: 4px;"
    "}"
  );

  auto * layout = new QVBoxLayout(this);

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

  // Recenter button
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

  connect(recenter_button_, &QPushButton::clicked, this, &TelemetryPanel::requestRecenter);

  grid_resolution_label_ = new QLabel("⊞ Grid: -- m");
  QFont grid_font;
  grid_font.setBold(true);
  grid_resolution_label_->setFont(grid_font);
  grid_resolution_label_->setStyleSheet("color: #4a5568; padding-top: 10px;");
  layout->addWidget(grid_resolution_label_);

  layout->addStretch();
  layout->addWidget(simulation_status_label_);
}

void TelemetryPanel::updateUsvTelemetry(double x, double y, double heading) {
  usv_position_label_->setText(
    QString("Posição:\nx = %1 m\ny = %2 m")
      .arg(x, 0, 'f', 1)
      .arg(y, 0, 'f', 1)
  );

  heading_label_->setText(
    QString("Heading: %1°").arg(heading, 0, 'f', 1)
  );
}

void TelemetryPanel::updateVesselCount(std::size_t count) {
  vessel_count_label_->setText(
    QString("Monitored vessels: %1").arg(count)
  );
}

void TelemetryPanel::updateGridResolution(double step) {
  grid_resolution_label_->setText(
    QString("⊞ Grid: %1 m").arg(step, 0, 'f', 0)
  );
}

void TelemetryPanel::setSimulationStatus(const QString & status) {
  simulation_status_label_->setText(status);
}

} // namespace dt_viz