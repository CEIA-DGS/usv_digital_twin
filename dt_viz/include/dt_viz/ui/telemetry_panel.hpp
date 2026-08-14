#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <cstdint>

namespace dt_viz {

/**
 * @brief Visual component responsible for displaying USV telemetry data,
 *        target counts, and simulation legends.
 */
class TelemetryPanel : public QWidget {
  Q_OBJECT

public:
  explicit TelemetryPanel(QWidget * parent = nullptr);

  /**
   * @brief Updates the USV's navigation data on the screen.
   */
  void updateUsvTelemetry(double x, double y, double heading);

  /**
   * @brief Updates the count of monitored AIS targets.
   */
  void updateVesselCount(std::size_t count);

  /**
   * @brief Updates the text of the scale/grid indicator.
   */
  void updateGridResolution(double step);

  /**
   * @brief Updates the simulation's text status.
   */
  void setSimulationStatus(const QString & status);

signals:
  /**
   * @brief Emitted when the user clicks the "Recenter USV" button.
   * The View listening for this signal must handle the camera centering.
   */
  void requestRecenter();

private:
  void setupUi();

  QLabel * usv_position_label_;
  QLabel * heading_label_;
  QLabel * vessel_count_label_;
  QLabel * simulation_status_label_;
  QLabel * grid_resolution_label_;
  QPushButton * recenter_button_;
};

} // namespace dt_viz