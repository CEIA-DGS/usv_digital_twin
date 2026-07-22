#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <usv_msgs/msg/ais_report.hpp>

#include <dt_core/twin_interface.hpp>
#include <memory>

namespace dt_ros {

class DigitalTwinNode : public rclcpp::Node {
public:
    explicit DigitalTwinNode(std::shared_ptr<dt::DigitalTwinCore> dt_core, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::shared_ptr<dt::DigitalTwinCore> dt_core_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<usv_msgs::msg::AisReport>::SharedPtr ais_sub_;

    // Callbacks
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void ais_callback(const usv_msgs::msg::AisReport::SharedPtr msg);
    
    // Variáveis de estado local 
    dt::types::Pose current_pose_;
};

} // namespace dt_ros