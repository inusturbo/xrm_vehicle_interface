#include <xrm_zmp_interface/xrm_zmp_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info.hpp>

XrmZmpNode::XrmZmpNode() : Node("xrm_zmp_interface")
{
    // Subscribers: From Autoware
    // Gate Mode
    gate_mode_sub_ = this->create_subscription<tier4_control_msgs::msg::GateMode>(
        "/control/gate_mode", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::gate_mode_callback, this, std::placeholders::_1));
    // From Control - Vehicle Control Command
    control_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::control_cmd_callback, this, std::placeholders::_1));
    gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::gear_cmd_callback, this, std::placeholders::_1));
    // From Planning - Vehicle Signal Commands
    hand_brake_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>(
        "/control/command/hand_brake_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::hand_brake_cmd_callback, this, std::placeholders::_1));
    hazard_lights_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
        "/control/command/hazard_lights_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::hazard_lights_cmd_callback, this, std::placeholders::_1));
    headlights_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HeadlightsCommand>(
        "/control/command/headlights_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::headlights_cmd_callback, this, std::placeholders::_1));
    horn_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HornCommand>(
        "/control/command//horn_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::horn_cmd_callback, this, std::placeholders::_1));
    stationary_locking_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::StationaryLockingCommand>(
        "/control/command/stationary_locking_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::stationary_locking_cmd_callback, this, std::placeholders::_1));
    turn_indicators_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::turn_indicators_cmd_callback, this, std::placeholders::_1));
    wipers_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::WipersCommand>(
        "/control/command/wipers_cmd", rclcpp::QoS(1),
        std::bind(&XrmZmpNode::wipers_cmd_callback, this, std::placeholders::_1));

    // Publishers: To Autoware
    // Control Mode
    control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));
    // Steering Status
    steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_report", rclcpp::QoS(1));
    steering_wheel_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>(
        "/vehicle/status/steering_wheel_status", rclcpp::QoS(1));
    // Actuation Status
    actuation_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
        "/vehicle/status/actuation_status", rclcpp::QoS(1));
    velocity_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_report", rclcpp::QoS(1));
    // Vehicle Signal Reports
    gear_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_report", rclcpp::QoS(1));
    hand_brake_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport>(
        "/vehicle/status/hand_brake_report", rclcpp::QoS(1));
    hazard_lights_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
        "/vehicle/status/hazard_lights_report", rclcpp::QoS(1));
    headlights_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HeadlightsReport>(
        "/vehicle/status/headlights_report", rclcpp::QoS(1));
    horn_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HornReport>(
        "/vehicle/status/horn_report", rclcpp::QoS(1));
    turn_indicators_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
        "/vehicle/status/turn_indicators_report", rclcpp::QoS(1));
    wipers_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::WipersReport>(
        "/vehicle/status/wipers_report", rclcpp::QoS(1));

    // Timer
    const auto period_ns = rclcpp::Rate(ZMP_LOOP_RATE).period();
    publish_to_autoware_timer_ = rclcpp::create_timer(this, this->get_clock(), period_ns,
                                                      std::bind(&XrmZmpNode::publishCommands, this));
}

void XrmZmpNode::control_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::control_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::gear_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::gate_mode_callback(const tier4_control_msgs::msg::GateMode::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::gate_mode_callback() LOG: " << std::endl;
}

void XrmZmpNode::hand_brake_cmd_callback(const autoware_auto_vehicle_msgs::msg::HandBrakeCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::hand_brake_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::hazard_lights_cmd_callback(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::hazard_lights_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::headlights_cmd_callback(const autoware_auto_vehicle_msgs::msg::HeadlightsCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::headlights_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::horn_cmd_callback(const autoware_auto_vehicle_msgs::msg::HornCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::horn_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::stationary_locking_cmd_callback(const autoware_auto_vehicle_msgs::msg::StationaryLockingCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::stationary_locking_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::turn_indicators_cmd_callback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::turn_indicators_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::wipers_cmd_callback(const autoware_auto_vehicle_msgs::msg::WipersCommand::SharedPtr msg)
{
    std::cout << "xrm_zmp_interface::wipers_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::publishCommands()
{
    std::cout << "xrm_zmp_interface::publishCommands() LOG: " << std::endl;
}