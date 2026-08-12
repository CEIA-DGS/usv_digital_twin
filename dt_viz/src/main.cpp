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

  // core instantiation
  auto dt_core = std::make_shared<dt::DigitalTwinCore>();

  // Initializes the ROS Adapter by injecting the Core.
  rclcpp::NodeOptions options;
  auto dt_ros_node = std::make_shared<dt_ros::DigitalTwinNode>(dt_core, options);

  // Runs ROS in background
  std::thread ros_thread([dt_ros_node]() {
    rclcpp::spin(dt_ros_node);
  });

  // Initializes and displays the graphical interface.
  dt_viz::MainWindow window(dt_core);
  window.show();

  // Freezes the graphical interface in the main loop.
  const int result = application.exec();

  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  return result;
}