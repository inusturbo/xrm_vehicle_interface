#include <xrm_zmp_interface/xrm_zmp_interface.hpp>
#include <rclcpp/rclcpp.hpp>

XrmZmpNode::XrmZmpNode() : Node("xrm_zmp_interface")
{
  vehicle_util_ = new VehicleUtil();
  vehicle_util_->Init();
  vehicle_util_->Start();

  // Subscribers: From Autoware
  // Gate Mode
  gate_mode_sub_ = this->create_subscription<tier4_control_msgs::msg::GateMode>(
      "/control/gate_mode_cmd", rclcpp::QoS(1),
      std::bind(&XrmZmpNode::gate_mode_callback, this, std::placeholders::_1));
  // From Control - Vehicle Control Command
  control_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", rclcpp::QoS(1),
      std::bind(&XrmZmpNode::control_cmd_callback, this, std::placeholders::_1));
  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
      "/control/command/gear_cmd", rclcpp::QoS(1),
      std::bind(&XrmZmpNode::gear_cmd_callback, this, std::placeholders::_1));
  // From Planning - Vehicle Signal Commands
  hazard_lights_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", rclcpp::QoS(1),
      std::bind(&XrmZmpNode::hazard_lights_cmd_callback, this, std::placeholders::_1));
  turn_indicators_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", rclcpp::QoS(1),
      std::bind(&XrmZmpNode::turn_indicators_cmd_callback, this, std::placeholders::_1));

  // Publishers: To Autoware
  // Control Mode
  control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", rclcpp::QoS(1));
  gate_mode_pub_ = this->create_publisher<tier4_control_msgs::msg::GateMode>(
      "/control/current_gate_mode", rclcpp::QoS(1));
  // Steering Status
  steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", rclcpp::QoS(1));
  steering_wheel_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>(
      "/vehicle/status/steering_wheel_status", rclcpp::QoS(1));
  // Actuation Status
  actuation_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
      "/vehicle/status/actuation_status", rclcpp::QoS(1));
  velocity_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", rclcpp::QoS(1));
  // Vehicle Signal Reports
  gear_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
      "/vehicle/status/gear_status", rclcpp::QoS(1));
  hazard_lights_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
      "/vehicle/status/hazard_lights_status", rclcpp::QoS(1));
  turn_indicators_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS(1));

  // Timer
  const auto period_ns = rclcpp::Rate(ZMP_LOOP_RATE).period();
  publish_to_autoware_timer_ = rclcpp::create_timer(this, this->get_clock(), period_ns,
                                                    std::bind(&XrmZmpNode::publishCommands, this));
}

void XrmZmpNode::control_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  std::cout << "xrm_zmp_interface::control_cmd_callback() LOG: " << std::endl;
  float target_veloc = msg->longtitudinal.speed * 3.6f; // m/s to km/h
}

void XrmZmpNode::gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
  std::cout << "xrm_zmp_interface::gear_cmd_callback() LOG: " << std::endl;
}

void XrmZmpNode::gate_mode_callback(const tier4_control_msgs::msg::GateMode::SharedPtr msg)
{
  std::cout << "xrm_zmp_interface::gate_mode_callback() LOG: " << std::endl;
  uint8_t mode = msg->data;
  if (mode == tier4_control_msgs::msg::GateMode::AUTO)
  {
    vehicle_util_->SetProgram();
  }
  else if (mode == tier4_control_msgs::msg::GateMode::EXTERNAL)
  {
    vehicle_util_->SetManual();
  }
}

void XrmZmpNode::hazard_lights_cmd_callback(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
{
  std::cout << "xrm_zmp_interface::hazard_lights_cmd_callback() LOG: " << std::endl;
  if (msg->command == autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE)
  {
    std::cout << "xrm_zmp_interface::hazard_lights_cmd_callback() LOG: ENABLE" << std::endl;
    vehicle_util_->SetHazardLights(1);
  }
  else if (msg->command == autoware_auto_vehicle_msgs::msg::HazardLightsCommand::DISABLE)
  {
    std::cout << "xrm_zmp_interface::hazard_lights_cmd_callback() LOG: DISABLE" << std::endl;
    vehicle_util_->SetHazardLights(0);
  }
  else
  {
    std::cout << "xrm_zmp_interface::hazard_lights_cmd_callback() NO_COMMAND" << std::endl;
  }
}

void XrmZmpNode::turn_indicators_cmd_callback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
{
  std::cout << "xrm_zmp_interface::turn_indicators_cmd_callback() LOG: " << std::endl;
  if (msg->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT)
  {
    std::cout << "xrm_zmp_interface::turn_indicators_cmd_callback() LOG: LEFT" << std::endl;
    vehicle_util_->SetBlinkerLeft(1);
    vehicle_util_->SetBlinkerRight(0);
  }
  else if (msg->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT)
  {
    std::cout << "xrm_zmp_interface::turn_indicators_cmd_callback() LOG: RIGHT" << std::endl;
    vehicle_util_->SetBlinkerRight(1);
    vehicle_util_->SetBlinkerLeft(0);
  }
  else if (msg->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE)
  {
    std::cout << "xrm_zmp_interface::turn_indicators_cmd_callback() LOG: NONE" << std::endl;
    vehicle_util_->SetBlinkerRight(0);
    vehicle_util_->SetBlinkerLeft(0);
  }
  else
  {
    std::cout << "xrm_zmp_interface::turn_indicators_cmd_callback() NO_COMMAND" << std::endl;
  }
}

void XrmZmpNode::publishCommands()
{
  std::cout << "xrm_zmp_interface::publishCommands() LOG: " << std::endl;
}