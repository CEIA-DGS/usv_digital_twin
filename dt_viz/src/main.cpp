#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>

#include "dt_core/twin_interface.hpp"
#include "dt_ros/dt_node.hpp"

#include "dt_viz/controllers/simulation_controller.hpp"
#include "dt_viz/ui/main_window.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);

  // Core instantiation
  auto dt_core = std::make_shared<dt::DigitalTwinCore>();

  // Initializes the ROS Adapter by injecting the Core
  rclcpp::NodeOptions options;
  auto dt_ros_node = std::make_shared<dt_ros::DigitalTwinNode>(dt_core, options);

  // Runs ROS in background
  std::thread ros_thread([dt_ros_node]() {
    rclcpp::spin(dt_ros_node);
  });

  // Initializes the controller that bridges the Core logic and the UI
  auto viz_controller = std::make_shared<dt_viz::SimulationController>(dt_core);

  // Initializes and displays the graphical interface, injecting the controller
  dt_viz::MainWindow window(viz_controller);
  window.show();

  // Freezes the graphical interface in the main loop
  const int result = application.exec();

  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  return result;
}