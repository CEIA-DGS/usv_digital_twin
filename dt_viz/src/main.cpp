#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>

#include "dt_core/twin_interface.hpp"
#include "dt_ros/dt_node.hpp"
#include "dt_viz/main_window.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);

  // Instancia o Core
  auto dt_core = std::make_shared<dt::DigitalTwinCore>();

  // Inicializa o Adaptador ROS injetando o Core
  rclcpp::NodeOptions options;
  auto dt_ros_node = std::make_shared<dt_ros::DigitalTwinNode>(dt_core, options);

  // Roda o ROS em background
  std::thread ros_thread([dt_ros_node]() {
    rclcpp::spin(dt_ros_node);
  });

  // Inicializa e mostra a Interface Gráfica
  MainWindow window(dt_core);
  window.show();

  // Trava a interface gráfica no loop principal
  const int result = application.exec();

  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  return result;
}