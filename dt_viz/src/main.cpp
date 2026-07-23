#include "dt_viz/main_window.hpp"
#include "dt_viz/viz_node.hpp"

#include <QApplication>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include <memory>

/**
 * @brief Inicializa o ROS 2 e a interface Qt.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  QApplication application(argc, argv);

  MainWindow window;

  auto node = std::make_shared<VizNode>(&window);

  // Processa periodicamente os callbacks ROS 2 na thread do Qt.
  QTimer ros_timer;

  QObject::connect(
    &ros_timer,
    &QTimer::timeout,
    [node]() {
      rclcpp::spin_some(node);
    }
  );

  ros_timer.start(20);

  window.show();

  const int result = application.exec();

  rclcpp::shutdown();

  return result;
}