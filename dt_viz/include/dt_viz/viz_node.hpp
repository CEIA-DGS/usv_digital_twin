#pragma once

#include "dt_viz/main_window.hpp"

#include <dt_msgs/msg/collision_alert.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dt_viz {

/**
 * @brief ROS 2 node responsible for receiving data for the interface.
 */
class VizNode : public rclcpp::Node
{
public:
  explicit VizNode(MainWindow * window);

private:
  /**
   * @brief Receives the planned trajectory.
   */
  void plannedRouteCallback(
    const nav_msgs::msg::Path::SharedPtr message);

  /**
   * @brief Receive a collision alert.
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

} // namespace dt_viz