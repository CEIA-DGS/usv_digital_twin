#include "dt_viz/navigation_scene.hpp"

#include <cmath>
#include <QPen>
#include <QColor>
#include <QVector>
#include <QLineF>

namespace dt_viz {

NavigationScene::NavigationScene(QObject * parent)
: QGraphicsScene(parent)
{
}

double NavigationScene::calculateGridStep(double scale) const {
  if (scale > 4.0) {
    return 10.0;
  }
  if (scale > 1.0) {
    return 50.0;
  }
  if (scale > 0.5) {
    return 100.0;
  }
  if (scale > 0.1){
    return 500.0;
  }
  return 1000.0;
}

void NavigationScene::drawBackground(QPainter * painter, const QRectF & rect) {
  QGraphicsScene::drawBackground(painter, rect);

  const double scale = std::abs(painter->transform().m11());
  const double grid_step = calculateGridStep(scale);

  QPen grid_pen(QColor(180, 180, 180, 150));
  grid_pen.setCosmetic(true);
  painter->setPen(grid_pen);

  // Align grid to global coordinates
  const double left = std::floor(rect.left() / grid_step) * grid_step;
  const double right = std::ceil(rect.right() / grid_step) * grid_step;
  const double top = std::floor(rect.top() / grid_step) * grid_step;
  const double bottom = std::ceil(rect.bottom() / grid_step) * grid_step;

  QVector<QLineF> lines;
  
  for (double x = left; x <= right; x += grid_step) {
    lines.append(QLineF(x, rect.top(), x, rect.bottom()));
  }
  for (double y = top; y <= bottom; y += grid_step) {
    lines.append(QLineF(rect.left(), y, rect.right(), y));
  }

  painter->drawLines(lines);
}

} // namespace dt_viz