#ifndef XRM_ZMP_INTERFACE_HPP
#define XRM_ZMP_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>

// Subscribers: From Autoware
// Gate Mode
#include <tier4_control_msgs/msg/gate_mode.hpp>
// From Control - Vehicle Control Command
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

// From Planning - Vehicle Signal Commands
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

// Publishers: To Autoware
// Control Mode
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
// Steering Status
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
// Actuation Status
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
// Vehicle Signal Reports
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <zmp/vehicle_info.hpp>
#include <zmp/vehicle_util.hpp>

class XrmZmpNode : public rclcpp::Node
{
public:
    XrmZmpNode();

    VehicleUtil *vehicle_util_;
    string base_frame_id_;
    // Subscribers: From Autoware
    // Gate Mode
    rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::SharedPtr gate_mode_sub_;
    // From Control - Vehicle Control Command
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
    // From Planning - Vehicle Signal Commands
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_lights_cmd_sub_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_sub_;

    // Publishers: To Autoware
    // Control Mode
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
    rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr gate_mode_pub_;
    // Steering Status
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
    // Actuation Status
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;
    // Vehicle Signal Reports
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_report_pub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_report_pub_;

    // Callbacks
    void gate_mode_callback(const tier4_control_msgs::msg::GateMode::SharedPtr msg);
    void control_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
    void gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg);
    void hazard_lights_cmd_callback(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg);
    void turn_indicators_cmd_callback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg);

    // Functions
    void publishCommands();

    // Timer
    rclcpp::TimerBase::SharedPtr publish_to_autoware_timer_;
};

#endif // XRM_ZMP_INTERFACE_HPP