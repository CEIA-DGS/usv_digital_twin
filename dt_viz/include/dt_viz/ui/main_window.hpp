#pragma once

#include "dt_viz/controllers/simulation_controller.hpp"
#include "dt_viz/ui/map_canvas.hpp"
#include "dt_viz/ui/telemetry_panel.hpp"
#include "dt_viz/ui/camera_grid_widget.hpp"
#include "dt_viz/ui/camera_control_panel.hpp"

#include <QMainWindow>
#include <memory>

namespace dt_viz {

/**
 * @brief Main application window.
 *
 * Acts solely as a layout container and signal/slot router between
 * the UI components (MapCanvas, TelemetryPanel) and the backend (SimulationController).
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  /**
   * @brief Constructs the main window and wiring.
   *
   * @param controller Shared pointer to the simulation controller.
   * @param parent Pointer to the parent widget.
   */
  explicit MainWindow(std::shared_ptr<SimulationController> controller, QWidget * parent = nullptr);

private:
  void setupLayout();
  void connectSignals();

  std::shared_ptr<SimulationController> controller_;
  MapCanvas * map_canvas_;
  TelemetryPanel * telemetry_panel_;
  CameraGridWidget * camera_grid_widget_;
  CameraControlPanel * camera_control_panel_;
  std::unordered_map<int, QString> slot_camera_mapping_;
  std::unordered_map<std::string, QImage> latest_camera_frames_;
};

} // namespace dt_viz