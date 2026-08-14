#pragma once

#include <QGraphicsScene>
#include <QPainter>
#include <QRectF>

namespace dt_viz {

/**
 * @brief Custom QGraphicsScene to handle adaptive background grid rendering.
 * 
 * This class overrides the background drawing to dynamically adjust
 * the grid resolution based on the current zoom level, maintaining
 * both rendering performance and visual clarity.
 */
class NavigationScene : public QGraphicsScene {
public:
  /**
   * @brief Constructs the NavigationScene.
   * 
   * @param parent Pointer to the parent object (default is nullptr).
   */
  explicit NavigationScene(QObject * parent = nullptr);

  /**
   * @brief Calculates the appropriate grid resolution based on the current scale.
   * 
   * @param scale Current view scale factor (e.g., m11 from the transform matrix).
   * @return The grid step size in meters.
   */
  double calculateGridStep(double scale) const;

protected:
  /**
   * @brief Draws the dynamic background grid on the exposed view area.
   * 
   * @param painter The painter object used for drawing.
   * @param rect The exposed rectangle area that needs to be updated.
   */
  void drawBackground(QPainter * painter, const QRectF & rect) override;
};

} // namespace dt_viz