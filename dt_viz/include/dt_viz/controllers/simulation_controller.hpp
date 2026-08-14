#pragma once

#include "dt_core/twin_interface.hpp"
#include "dt_viz/ui/map_canvas.hpp" 

#include <QObject>
#include <QTimer>
#include <QString>
#include <memory>
#include <vector>
#include <cstdint>

namespace dt_viz {

/**
 * @brief Controller responsible for bridging the core logic and the UI.
 *
 * Periodically polls the Digital Twin core for the latest state
 * and emits Qt signals with the processed data, ensuring UI components
 * remain decoupled from the business logic.
 */
class SimulationController : public QObject {
  Q_OBJECT

public:
  /**
   * @brief Constructs the SimulationController.
   *
   * @param dt_core Shared pointer to the Digital Twin Core instance.
   * @param parent Pointer to the parent QObject.
   */
  explicit SimulationController(std::shared_ptr<dt::DigitalTwinCore> dt_core, QObject * parent = nullptr);

signals:
  void usvPoseUpdated(double x, double y, double heading_deg);
  void targetsUpdated(const std::vector<types::Target> & targets);
  void plannedRouteUpdated(const std::vector<RoutePoint> & route);
  void collisionAlertUpdated(std::uint32_t mmsi, bool collision_imminent);
  void simulationStatusUpdated(const QString & status);

private slots:
  void processTick();

private:
  std::shared_ptr<dt::DigitalTwinCore> dt_core_;
  QTimer * timer_;
};

} // namespace dt_viz