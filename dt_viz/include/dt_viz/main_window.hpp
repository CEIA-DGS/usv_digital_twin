#pragma once
#include "dt_core/twin_interface.hpp"
#include <memory>

// ============================================================
// Qt Libraries
// ============================================================

#include <QBrush>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QLabel>
#include <QMainWindow>
#include <QPainterPath>
#include <QResizeEvent>
#include <QTimer>
#include <QWidget>
#include <QWheelEvent>
#include <QPushButton>

// ============================================================
// C++ Standard Libraries
// ============================================================

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace dt_viz {

/**
 * @brief Represents a point in the planned route.
 */
struct RoutePoint{
  double x;
  double y;
};

/**
 * @brief Main window of the visualization tool.
 *
 * This class draws the navigable free zone (NavMesh), the USV, the monitored
 * vessels, the traveled trajectory, and the planned route.
 *
 * It also visually alters targets that present an imminent collision risk based
 * on the digital twin predictive core.
 */
class MainWindow : public QMainWindow{
public:
  /**
   * @brief Constructs the MainWindow.
   * 
   * @param dt_core Shared pointer to the Digital Twin Core instance.
   * @param parent Pointer to the parent widget (default is nullptr).
   */
  explicit MainWindow(std::shared_ptr<dt::DigitalTwinCore> dt_core, QWidget * parent = nullptr);

  /**
   * @brief Updates the planned route displayed on the screen.
   *
   * @param route Ordered list of active waypoints.
   */
  void updatePlannedRoute(const std::vector<RoutePoint> & route);

  /**
   * @brief Alters the visual representation of a target.
   *
   * @param mmsi AIS identifier of the vessel.
   * @param collision_imminent Indicates the existence of a collision risk.
   */
  void updateCollisionAlert(std::uint32_t mmsi, bool collision_imminent);

protected:
  /**
   * @brief Adjusts the scale when the window is resized.
   * 
   * @param event Pointer to the resize event.
   */
  void resizeEvent(QResizeEvent * event) override;

  /**
   * @brief Intercepts events before they are processed (used for zooming).
   * 
   * @param watched The object being watched.
   * @param event The event being intercepted.
   * @return True if the event was filtered out, false otherwise.
   */
  bool eventFilter(QObject *watched, QEvent *event) override;

private:
    
  std::shared_ptr<dt::DigitalTwinCore> dt_core_;
  
  // Stores the geometric center of the NavMesh for initial positioning
  double map_center_x_ = 0.0;
  double map_center_y_ = 0.0;
  
  /**
   * @brief Configures the initial window settings, titles, and layouts.
   */
  void configureWindow();

  /**
   * @brief Creates and initializes the graphic scene elements.
   */
  void createScene();

  /**
   * @brief Creates the side information panel for USV telemetry and diagnostics.
   */
  void createInformationPanel();

  /**
   * @brief Draws the background grid (currently unused for real UTM mode).
   */
  void drawGrid();

  /**
   * @brief Draws the coordinate axes (currently unused for real UTM mode).
   */
  void drawAxes();

  /**
   * @brief Reads and draws the navigable free zone (NavMesh) using GDAL shapefiles.
   */
  void drawFreeZone();

  /**
   * @brief Draws the USV polygon, heading line, and labels.
   */
  void drawUsv();

  /**
   * @brief Draws a static scale bar reference on the scene.
   */
  void drawScaleBar();

  /**
   * @brief Renders the planned route path and waypoint markers on the scene.
   */
  void drawPlannedRoute();

  /**
   * @brief Clears the previously rendered planned route and waypoints from the scene.
   */
  void clearPlannedRoute();

  /**
   * @brief Main loop callback that fetches the latest state from the Core and updates all visual elements.
   */
  void updateSimulation();

  /**
   * @brief Updates the trail of the USV's past positions on the map.
   * 
   * @param x The current X coordinate of the USV.
   * @param y The current Y coordinate of the USV.
   */
  void updateUsvTrajectory(double x, double y);

  /**
   * @brief Updates the text data in the side information panel.
   * 
   * @param usv_x Current USV X position.
   * @param usv_y Current USV Y position.
   * @param heading Current USV heading in degrees.
   */
  void updateInformationPanel(
    double usv_x,
    double usv_y,
    double heading);

  /**
   * @brief Gets the default brush used for vessels without collision risk.
   * 
   * @return QBrush The brush for normal vessels.
   */
  QBrush normalVesselBrush() const;

  /**
   * @brief Gets the brush used for vessels with an imminent collision risk.
   * 
   * @return QBrush The brush for at-risk vessels.
   */
  QBrush collisionVesselBrush() const;

  // Main components.
  QGraphicsScene * scene_;
  QGraphicsView * view_;
  QTimer * timer_;

  QWidget * central_widget_;
  QWidget * information_panel_;

  QLabel * usv_position_label_;
  QLabel * heading_label_;
  QLabel * vessel_count_label_;
  QLabel * simulation_status_label_;
  QPushButton * recenter_button_;

  // Free zone and USV representation.
  QGraphicsPolygonItem * free_zone_;
  QGraphicsPolygonItem * usv_;
  QGraphicsSimpleTextItem * usv_label_;
  QGraphicsLineItem * heading_line_;

  // Traveled trajectory.
  QGraphicsPathItem * trajectory_item_;
  QPainterPath trajectory_path_;
  std::vector<QPointF> trajectory_points_;

  // Future planned route.
  std::vector<RoutePoint> planned_route_;
  QGraphicsPathItem * planned_route_item_;
  std::vector<QGraphicsEllipseItem *> waypoint_items_;
  std::vector<QGraphicsSimpleTextItem *> waypoint_labels_;

  // Monitored vessels.
  std::vector<QGraphicsEllipseItem *> vessels_;
  std::vector<QGraphicsSimpleTextItem *> vessel_labels_;

  // Maps to locate a specific target by its MMSI/ID.
  std::unordered_map<
    std::uint32_t,
    QGraphicsEllipseItem *
  > vessel_items_by_mmsi_;

  std::unordered_map<
    std::uint32_t,
    QGraphicsSimpleTextItem *
  > vessel_labels_by_mmsi_;

  double simulation_time_;
  bool is_tracking_usv_ = true;
};

} // namespace dt_viz