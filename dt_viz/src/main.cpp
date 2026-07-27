#include "dt_viz/main_window.hpp"
#include "dt_viz/viz_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QApplication>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  QApplication application(argc, argv);

  MainWindow window;

  try {
    const std::string dt_core_share =
      ament_index_cpp::get_package_share_directory(
        "dt_core");

    const std::filesystem::path chart_directory =
      std::filesystem::path(dt_core_share) /
      "data" /
      "output" /
      "NavMesh_Shapefiles_BR401410";

    if (!window.loadNauticalChartDirectory(
        chart_directory.string()))
    {
      std::cerr
        << "[dt_viz] Não foi possível carregar a carta em: "
        << chart_directory
        << '\n';
    }
  } catch (const std::exception & error) {
    std::cerr
      << "[dt_viz] Erro ao localizar os dados do dt_core: "
      << error.what()
      << '\n';
  }

  auto node =
    std::make_shared<VizNode>(&window);

  QTimer ros_timer;

  QObject::connect(
    &ros_timer,
    &QTimer::timeout,
    [node]() {
      rclcpp::spin_some(node);
    });

  ros_timer.start(20);

  window.show();

  const int result =
    application.exec();

  rclcpp::shutdown();

  return result;
}