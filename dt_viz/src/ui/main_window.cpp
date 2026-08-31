#include "dt_viz/ui/main_window.hpp"

#include <QHBoxLayout>
#include <QStatusBar>

namespace dt_viz {

MainWindow::MainWindow(std::shared_ptr<SimulationController> controller, QWidget * parent)
: QMainWindow(parent),
  controller_(std::move(controller)),
  map_canvas_(new MapCanvas(this)),
  telemetry_panel_(new TelemetryPanel(this)),
  camera_grid_widget_(new CameraGridWidget(this)),
  camera_control_panel_(new CameraControlPanel(this))
{
  setWindowTitle("CEIA DGS - Visual Diagnostic Tool");
  resize(1200, 750);

  setupLayout();
  connectSignals();

  statusBar()->showMessage("Provisional interface | Top view | Simulated data");
}

void MainWindow::setupLayout() {
  QTabWidget * tab_widget = new QTabWidget(this);

  // --- ABA 1: Carta Náutica & Radar ---
  QWidget * map_tab = new QWidget(this);
  auto * map_layout = new QHBoxLayout(map_tab);
  map_layout->setContentsMargins(5, 5, 5, 5);
  map_layout->setSpacing(8);
  map_layout->addWidget(map_canvas_, 1);
  map_layout->addWidget(telemetry_panel_);
  tab_widget->addTab(map_tab, "🗺️ Carta Náutica & Radar");

  // --- ABA 2: Câmeras Onboard ---
  QWidget * camera_tab = new QWidget(this);
  auto * camera_layout = new QHBoxLayout(camera_tab);
  camera_layout->setContentsMargins(5, 5, 5, 5);
  camera_layout->setSpacing(8);
  camera_layout->addWidget(camera_grid_widget_, 1);
  camera_layout->addWidget(camera_control_panel_);
  tab_widget->addTab(camera_tab, "📷 Câmeras Onboard");

  setCentralWidget(tab_widget);
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

  connect(controller_.get(), &SimulationController::targetsPredictedTrajectoriesUpdated,
          map_canvas_, &MapCanvas::updateTargetsPredictedTrajectories);

  connect(controller_.get(), &SimulationController::collisionPointsUpdated,
          map_canvas_, &MapCanvas::updateCollisionPoints);

  connect(controller_.get(), &SimulationController::targetApproachingUpdated,
          map_canvas_, &MapCanvas::setApproachingAlert);

  // Wire Controller to TelemetryPanel
  connect(controller_.get(), &SimulationController::usvTelemetryUpdated,
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

// Wire Controller to Camera Control Panel (Active IDs)
  connect(controller_.get(), &SimulationController::allCameraFramesUpdated,
          this, [this](const std::unordered_map<std::string, QImage>& frames, const std::vector<std::string>& active_ids) {
            latest_camera_frames_ = frames;
            camera_control_panel_->updateAvailableCameras(active_ids);

            // Atualiza os slots ativos no grid com base no mapeamento atual
            for (int slot = 0; slot < 4; ++slot) {
              if (slot_camera_mapping_.find(slot) != slot_camera_mapping_.end()) {
                QString cam_id = slot_camera_mapping_[slot];
                auto it = latest_camera_frames_.find(cam_id.toStdString());
                if (it != latest_camera_frames_.end()) {
                  camera_grid_widget_->updateCameraFrame(slot, cam_id, it->second);
                } else {
                  camera_grid_widget_->updateCameraFrame(slot, cam_id, QImage()); // Sem sinal
                }
              }
            }
          });

// Wire Camera Control Panel to Grid Widget
  connect(camera_control_panel_, &CameraControlPanel::layoutModeChanged,
          camera_grid_widget_, &CameraGridWidget::setLayoutMode);

  connect(camera_control_panel_, &CameraControlPanel::cameraAssigned,
          this, [this](int slot_index, const QString& camera_id) {
            if (camera_id.isEmpty()) {
              slot_camera_mapping_.erase(slot_index);
              camera_grid_widget_->updateCameraFrame(slot_index, "Desligado", QImage());
            } else {
              slot_camera_mapping_[slot_index] = camera_id;
              auto it = latest_camera_frames_.find(camera_id.toStdString());
              if (it != latest_camera_frames_.end()) {
                camera_grid_widget_->updateCameraFrame(slot_index, camera_id, it->second);
              }
            }
          });
}

} // namespace dt_viz