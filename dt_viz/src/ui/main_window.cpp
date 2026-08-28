#include "dt_viz/ui/main_window.hpp"

#include <QHBoxLayout>
#include <QStatusBar>

namespace dt_viz {

MainWindow::MainWindow(std::shared_ptr<SimulationController> controller, QWidget * parent)
: QMainWindow(parent),
  controller_(std::move(controller)),
  map_canvas_(new MapCanvas(this)),
  telemetry_panel_(new TelemetryPanel(this))
{
  setWindowTitle("CEIA DGS - Visual Diagnostic Tool");
  resize(1200, 750);

  setupLayout();
  connectSignals();

  statusBar()->showMessage("Provisional interface | Top view | Simulated data");
}

void MainWindow::setupLayout() {
  QWidget * central_widget = new QWidget(this);
  auto * main_layout = new QHBoxLayout(central_widget);

  main_layout->setContentsMargins(5, 5, 5, 5);
  main_layout->setSpacing(8);

  main_layout->addWidget(map_canvas_, 1);
  main_layout->addWidget(telemetry_panel_);

  setCentralWidget(central_widget);
}

void MainWindow::connectSignals() {
  // Wire UI to UI
  connect(telemetry_panel_, &TelemetryPanel::requestRecenter,
          map_canvas_, &MapCanvas::centerOnUsv);

  connect(map_canvas_, &MapCanvas::gridResolutionChanged,
          telemetry_panel_, &TelemetryPanel::updateGridResolution);

  // Wire Controller to MapCanvas
  connect(controller_.get(), &SimulationController::usvPoseUpdated,
          map_canvas_, &MapCanvas::updateUsvPose);

  connect(controller_.get(), &SimulationController::targetsUpdated,
          map_canvas_, &MapCanvas::updateTargets);

  connect(controller_.get(), &SimulationController::plannedRouteUpdated,
          map_canvas_, &MapCanvas::updatePlannedRoute);

  connect(controller_.get(), &SimulationController::collisionAlertUpdated,
          map_canvas_, &MapCanvas::setCollisionAlert);

  connect(controller_.get(), &SimulationController::usvPredictedTrajectoryUpdated,
          map_canvas_, &MapCanvas::updateUsvPredictedTrajectory);

  // Wire Controller to TelemetryPanel
  connect(controller_.get(), &SimulationController::usvPoseUpdated,
          telemetry_panel_, &TelemetryPanel::updateUsvTelemetry);

  connect(controller_.get(), &SimulationController::targetsUpdated,
          this, [this](const std::vector<types::Target> & targets) {
            telemetry_panel_->updateVesselCount(targets.size());
          });

  // Wire Controller to StatusBar
  connect(controller_.get(), &SimulationController::simulationStatusUpdated,
          this, [this](const QString & status) {
            statusBar()->showMessage(status);
          });
}

} // namespace dt_viz