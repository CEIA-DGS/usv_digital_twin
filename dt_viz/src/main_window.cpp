#include "dt_viz/main_window.hpp"

#include <QWheelEvent>

namespace dt_viz {

MainWindow::MainWindow(std::shared_ptr<dt::DigitalTwinCore> dt_core, QWidget * parent)
: QMainWindow(parent),
  dt_core_(std::move(dt_core)), 
  scene_(new QGraphicsScene(this)),
  view_(new QGraphicsView(scene_, this)),
  timer_(new QTimer(this)),
  central_widget_(new QWidget(this)),
  information_panel_(new QWidget(this)),
  usv_position_label_(nullptr),
  heading_label_(nullptr),
  vessel_count_label_(nullptr),
  simulation_status_label_(nullptr),
  free_zone_(nullptr),
  usv_(nullptr),
  usv_label_(nullptr),
  heading_line_(nullptr),
  trajectory_item_(nullptr),
  planned_route_item_(nullptr),
  simulation_time_(0.0)
{
  configureWindow();
  createInformationPanel();
  createScene();

  connect(
    timer_,
    &QTimer::timeout,
    this,
    [this]() {
      updateSimulation(); 
    });

  timer_->start(33); 
}

void MainWindow::resizeEvent(QResizeEvent * event){
  QMainWindow::resizeEvent(event);
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event){
  if (watched == view_->viewport() && event->type() == QEvent::Wheel) {
    QWheelEvent *wheel_event = static_cast<QWheelEvent *>(event);
    
    if (wheel_event->angleDelta().y() > 0) {
      view_->scale(1.15, 1.15); 
    } else {
      view_->scale(1.0 / 1.15, 1.0 / 1.15); 
    }
    
    return true; 
  }
  return QMainWindow::eventFilter(watched, event);
}

} // namespace dt_viz