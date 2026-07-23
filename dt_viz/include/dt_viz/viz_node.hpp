#ifndef DT_VIZ_VIZ_NODE_HPP
#define DT_VIZ_VIZ_NODE_HPP

#include "dt_viz/main_window.hpp"

#include <dt_msgs/msg/collision_alert.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Nó ROS 2 responsável por receber dados para a interface.
 */
class VizNode : public rclcpp::Node
{
public:
  explicit VizNode(MainWindow * window);

private:
  /**
   * @brief Recebe a derrota planejada.
   */
  void plannedRouteCallback(
    const nav_msgs::msg::Path::SharedPtr message);

  /**
   * @brief Recebe um alerta de colisão.
   */
  void collisionAlertCallback(
    const dt_msgs::msg::CollisionAlert::SharedPtr message);

  MainWindow * window_;

  rclcpp::Subscription<
    nav_msgs::msg::Path
  >::SharedPtr planned_route_subscription_;

  rclcpp::Subscription<
    dt_msgs::msg::CollisionAlert
  >::SharedPtr collision_alert_subscription_;
};

#endif