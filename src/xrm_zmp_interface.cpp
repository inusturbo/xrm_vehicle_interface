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
  // Actuation Status
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
  // m/s to km/h
  float target_veloc = msg->longitudinal.speed * 3.6f;
  // m/s^2 to km/h^2
  float acc = msg->longitudinal.acceleration * 12960.0f;
  vehicle_util_->VelocityControl(target_veloc, acc);
  float cmd_steering_angle = msg->lateral.steering_tire_angle * 180.0f / M_PI;                  // rad to deg
  float steering_tire_rotation_rate = msg->lateral.steering_tire_rotation_rate * 180.0f / M_PI; // rad/s to deg/s
  vehicle_util_->SteeringControl(cmd_steering_angle, steering_tire_rotation_rate);
}

void XrmZmpNode::gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
  std::cout << "xrm_zmp_interface::gear_cmd_callback() LOG: " << std::endl;
  uint8_t gear = msg->command;

  if (vehicle_util_->GetDrvSpeedKmh() != 0.0f)
  {
    return;
  }
  vehicle_util_->StopVehicle();
  if (gear == autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL)
  {
    // shift N
    vehicle_util_->SetDrvShiftMode(SHIFT_POS_N);
  }
  else if (gear == autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE)
  {
    // shift D
    vehicle_util_->SetDrvShiftMode(SHIFT_POS_D);
  }
  else if (gear == autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE)
  {
    // shift R
    vehicle_util_->SetDrvShiftMode(SHIFT_POS_R);
  }
  else if (gear == autoware_auto_vehicle_msgs::msg::GearCommand::LOW)
  {
    // shift B
    vehicle_util_->SetDrvShiftMode(SHIFT_POS_B);
  }
  else if (gear == autoware_auto_vehicle_msgs::msg::GearCommand::PARK)
  {
    // shift B
    vehicle_util_->SetDrvShiftMode(SHIFT_POS_P);
  }
  else
  {
    // no command
    std::cout << "xrm_zmp_interface::gear_cmd_callback() NO_COMMAND" << std::endl;
  }
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
  vehicle_util_->UpdateState();
  const rclcpp::Time current_time = get_clock()->now();
  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = current_time;

  // publish control mode
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
  control_mode_msg.stamp = current_time;
  if (vehicle_util_->isDrvControled() == 1 && vehicle_util_->isStrControled() == 1)
  {
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  }
  else if (vehicle_util_->isDrvControled() == 1 && vehicle_util_->isStrControled() == 0)
  {
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;
  }
  else if (vehicle_util_->isDrvControled() == 0 && vehicle_util_->isStrControled() == 1)
  {
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY;
  }
  else if (vehicle_util_->isDrvControled() == 0 && vehicle_util_->isStrControled() == 0)
  {
    control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
  }
  control_mode_pub_->publish(control_mode_msg);

  // publish gate mode
  tier4_control_msgs::msg::GateMode gate_mode_msg;
  if (vehicle_util_->isDrvControled() == 1 && vehicle_util_->isStrControled() == 1)
  {
    gate_mode_msg.data = tier4_control_msgs::msg::GateMode::AUTO;
  }
  else if (vehicle_util_->isDrvControled() == 0 && vehicle_util_->isStrControled() == 0)
  {
    gate_mode_msg.data = tier4_control_msgs::msg::GateMode::EXTERNAL;
  }
  gate_mode_pub_->publish(gate_mode_msg);

  // publish steering status
  autoware_auto_vehicle_msgs::msg::SteeringReport steering_report_msg;
  steering_report_msg.stamp = current_time;
  float current_steer_wheel =
      vehicle_util_->GetStrRad(); // current vehicle steering wheel angle [rad]
  float current_steering_tire_angle = current_steer_wheel / WHEEL_TO_STEERING;
  steering_report_msg.steering_tire_angle = current_steering_tire_angle;
  steering_status_pub_.publish(steering_report_msg);

  // publish velocity status
  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg;
  velocity_report_msg.header = header;
  velocity_report_msg.longitudinal_velocity = vehicle_util_->GetDrvSpeedMps();
  velocity_report_msg.lateral_velocity = 0.0f;
  velocity_report_msg.heading_rate = vehicle_util_->GetDrvSpeedMps() * std::tan(current_steer_wheel) / WHEEL_BASE; // [rad/s]
  velocity_report_pub_->publish(velocity_report_msg);

  // publish gear status
  autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
  gear_report_msg.stamp = current_time;
  int current_gear = vehicle_util_->GetCurrentGear(); //(0x00=B, 0x10=D, 0x20=N, 0x40=R)
  if (current_gear == SHIFT_POS_N)
  {
    gear_report_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL;
  }
  else if (current_gear == SHIFT_POS_R)
  {
    gear_report_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::REVERSE;
  }
  else if (current_gear == SHIFT_POS_D)
  {
    gear_report_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
  }
  else if (current_gear == SHIFT_POS_B)
  {
    gear_report_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::LOW;
  }
  else if (current_gear == SHIFT_POS_P)
  {
    gear_report_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::PARK;
  }
  else
  {
    gear_report_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NONE;
  }
  gear_report_pub_->publish(gear_report_msg);

  // publish hazard lights status
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_lights_report_msg;
  hazard_lights_report_msg.stamp = current_time;
  if (vehicle_util_->GetHazardLights() == 1)
  {
    hazard_lights_report_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE;
  }
  else
  {
    hazard_lights_report_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE;
  }
  hazard_lights_report_pub_->publish(hazard_lights_report_msg);

  // publish turn indicators status
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_report_msg;
  turn_indicators_report_msg.stamp = current_time;
  if (vehicle_util_->GetBlinkerLeft() == 1 && vehicle_util_->GetBlinkerRight() == 0)
  {
    turn_indicators_report_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
  }
  else if (vehicle_util_->GetBlinkerLeft() == 0 && vehicle_util_->GetBlinkerRight() == 1)
  {
    turn_indicators_report_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
  }
  else if (vehicle_util_->GetBlinkerLeft() == 0 && vehicle_util_->GetBlinkerRight() == 0)
  {
    turn_indicators_report_msg.report = 0;
  }
  turn_indicators_report_pub_.publish(turn_indicators_report_msg);
}