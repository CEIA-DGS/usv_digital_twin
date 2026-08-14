#pragma once

#include "dt_viz/ui/navigation_scene.hpp"
#include "dt_core/twin_interface.hpp"

#include <QGraphicsView>
#include <QGraphicsPolygonItem>
#include <QGraphicsPathItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsSimpleTextItem>
#include <QWheelEvent>
#include <QMouseEvent>

#include <vector>
#include <unordered_map>
#include <cstdint>

namespace dt_viz {

/**
 * @brief Represents a single waypoint in the planned visual route.
 */
struct RoutePoint {
  double x;
  double y;
};

/**
 * @brief Interactive graphical canvas for the digital twin simulation.
 *
 * This class inherits from QGraphicsView to encapsulate all rendering logic,
 * geometric transformations, and user interactions (panning and zooming).
 */
class MapCanvas : public QGraphicsView {
  Q_OBJECT

public:
  /**
   * @brief Constructs the MapCanvas and initializes visual elements.
   * 
   * @param parent Pointer to the parent widget.
   */
  explicit MapCanvas(QWidget * parent = nullptr);

  /**
   * @brief Updates the USV representation and its trajectory history.
   * 
   * @param x USV X coordinate in meters.
   * @param y USV Y coordinate in meters.
   * @param heading_deg USV heading in degrees.
   */
  void updateUsvPose(double x, double y, double heading_deg);

  /**
   * @brief Updates the rendered targets, dynamically creating or removing items.
   * 
   * @param targets Vector containing the latest state of all targets.
   */
  void updateTargets(const std::vector<types::Target> & targets);

  /**
   * @brief Renders the predictive path on the canvas.
   * 
   * @param route Ordered list of waypoints.
   */
  void updatePlannedRoute(const std::vector<RoutePoint> & route);

  /**
   * @brief Forces the view to center on the current USV representation.
   */
  void centerOnUsv();

  /**
   * @brief Changes the visual style of a vessel to indicate collision risk.
   * 
   * @param mmsi The identifier of the vessel.
   * @param collision_imminent True if a collision is predicted.
   */
  void setCollisionAlert(std::uint32_t mmsi, bool collision_imminent);

signals:
  /**
   * @brief Emitted when user interaction interrupts automatic USV tracking.
   */
  void trackingInterrupted();

  /**
   * @brief Emitted when the viewport scale changes, updating the grid step.
   * 
   * @param step The new calculated grid step in meters.
   */
  void gridResolutionChanged(double step);

protected:
  /**
   * @brief Overrides default wheel event to implement custom zoom logic.
   */
  void wheelEvent(QWheelEvent * event) override;

  /**
   * @brief Overrides mouse press to detect user-initiated panning.
   */
  void mousePressEvent(QMouseEvent * event) override;

private:
  void setupScene();
  void drawFreeZone();
  void drawUsv();
  void drawScaleBar();
  void clearPlannedRoute();
  void updateGridIndicator();

  QBrush normalVesselBrush() const;
  QBrush collisionVesselBrush() const;

  NavigationScene * scene_;

  double map_center_x_ = 0.0;
  double map_center_y_ = 0.0;

  QGraphicsPolygonItem * free_zone_;
  QGraphicsPolygonItem * usv_;
  QGraphicsSimpleTextItem * usv_label_;
  QGraphicsLineItem * heading_line_;

  QGraphicsPathItem * trajectory_item_;
  QPainterPath trajectory_path_;
  std::vector<QPointF> trajectory_points_;

  QGraphicsPathItem * planned_route_item_;
  std::vector<QGraphicsEllipseItem *> waypoint_items_;
  std::vector<QGraphicsSimpleTextItem *> waypoint_labels_;

  std::unordered_map<std::uint32_t, QGraphicsEllipseItem *> vessel_items_by_mmsi_;
  std::unordered_map<std::uint32_t, QGraphicsSimpleTextItem *> vessel_labels_by_mmsi_;

  bool is_tracking_usv_ = true;
};

} // namespace dt_viz