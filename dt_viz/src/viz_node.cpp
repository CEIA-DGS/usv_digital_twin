#include "dt_viz/viz_node.hpp"

#include <functional>
#include <vector>

VizNode::VizNode(MainWindow * window)
: Node("dt_visualizer_node"),
  window_(window)
{
  planned_route_subscription_ =
    this->create_subscription<nav_msgs::msg::Path>(
      "/planned_route",
      10,
      std::bind(
        &VizNode::plannedRouteCallback,
        this,
        std::placeholders::_1
      )
    );

  collision_alert_subscription_ =
    this->create_subscription<
      dt_msgs::msg::CollisionAlert
    >(
      "/collision_alert",
      10,
      std::bind(
        &VizNode::collisionAlertCallback,
        this,
        std::placeholders::_1
      )
    );

  RCLCPP_INFO(
    this->get_logger(),
    "Interface inscrita nos tópicos /planned_route e /collision_alert."
  );
}

void VizNode::plannedRouteCallback(
  const nav_msgs::msg::Path::SharedPtr message)
{
  std::vector<RoutePoint> route;

  route.reserve(message->poses.size());

  for (const auto & pose_stamped : message->poses) {
    route.push_back({
      pose_stamped.pose.position.x,
      pose_stamped.pose.position.y
    });
  }

  window_->updatePlannedRoute(route);
}

void VizNode::collisionAlertCallback(
  const dt_msgs::msg::CollisionAlert::SharedPtr message)
{
  window_->updateCollisionAlert(
    message->mmsi,
    message->collision_imminent
  );
}