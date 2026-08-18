// Copyright 2026 Phillip Pitschi

#include "stability_controller_node_cpp/stability_controller_node.hpp"

#include <algorithm>

StabilityControllerNode::StabilityControllerNode(
    const rclcpp::NodeOptions &options)
    : Node("StabilityController", "/core/control", options)
{
  // declare paramters
  declare_and_update_parameters();
  // overwrite vehicle parameters
  vehicle_->load_params(param_manager_composer_.get());

  topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);
  tam::core::TimeoutValueProvider timeout_values;

  // Parameters
  callback_handle_ = tam::pmg::connect_param_manager_to_ros_cb(this, param_manager_composer_);
  // Declare parameters from the controller managers.
  tam::pmg::declare_ros_params_from_param_manager(this, param_manager_composer_.get());

  // Control-loop timer
  if(!this->has_parameter("tS")){
    this->declare_parameter("tS", 0.01);
  }
  double time = this->get_parameter("tS").as_double();
  model_update_timer_ = tam::create_timer(
      this, std::chrono::microseconds(static_cast<uint64_t>(time * 1e6)),
      std::bind(&StabilityControllerNode::function_queue_callback, this));

  auto qos = tam::ros::get_qos();

  // Subscriptions
  sub_odometry_ = topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
      "/core/state/odometry", qos, std::bind(&StabilityControllerNode::odometry_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) odometry_received_ = false;
      },
      timeout_values.value_ms("StateEstimation"));
  sub_acceleration_ =
      topic_watchdog_->add_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
          "/core/state/acceleration", qos,
          std::bind(&StabilityControllerNode::acceleration_callback, this, _1),
          [this](bool timeout, std::chrono::milliseconds) {
            if (timeout) acceleration_received_ = false;
          },
          timeout_values.value_ms("StateEstimation"));
  sub_steering_ = topic_watchdog_->add_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
      "/core/state/steering", qos, std::bind(&StabilityControllerNode::steering_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) steering_received_ = false;
      },
      timeout_values.value_ms("StateEstimation"));
  sub_gear_request_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMInt8Stamped>(
      "/core/control/gear_request", qos,
      std::bind(&StabilityControllerNode::gear_request_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) gear_request_received_ = false;
      },
      timeout_values.value_ms("Default"));
  sub_gear_report_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMInt8Stamped>(
      "/vehicle/sensor/gear", qos, std::bind(&StabilityControllerNode::gear_report_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) gear_report_received_ = false;
      },
      timeout_values.value_ms("Default"));
  sub_ctrl_cmd_ = topic_watchdog_->add_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/core/control/tracking_controller/control_request", qos,
      std::bind(&StabilityControllerNode::control_command_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) ctrl_cmd_received_ = false;
      },
      timeout_values.value_ms("TrackingController"));
  sub_longitudinal_cmd_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMLongitudinalCmd>(
      "/core/control/longitudinal_controller/longitudinal_request", qos,
      std::bind(&StabilityControllerNode::longitudinal_cmd_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) longitudinal_cmd_received_ = false;
      },
      timeout_values.default_or_ms("LongitudinalController"));
  sub_additional_esc_targets_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMAdditionalEspTargets>(
      "/core/control/optional/additional_esp_targets", qos,
      std::bind(&StabilityControllerNode::additional_esc_targets_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) {
          esc_->set_additional_esc_targets(tam::types::control::AdditionalEspTargets{});
        }
      },
      timeout_values.value_ms("TrackingController"));
  sub_omega_engine_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat32Stamped>(
      "/vehicle/sensor/omega_engine_radps", qos, std::bind(&StabilityControllerNode::omega_engine_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) slip_calculation_->set_omega_engine_ok(false);
      },
      timeout_values.value_ms("Default"));
  sub_wheelspeed_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
      "/vehicle/sensor/wheelspeed_radps", qos, std::bind(&StabilityControllerNode::wheelspeed_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) {
          wheelspeed_received_ = false;
          slip_calculation_->set_wheelspeed_ok(false);
        }
      },
      timeout_values.value_ms("StateEstimation"));
  sub_brake_pressure_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
      "/vehicle/sensor/brake_pressure_Pa", qos, std::bind(&StabilityControllerNode::brake_pressure_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) brake_pressure_received_ = false;
      },
      timeout_values.value_ms("Default"));
  sub_brake_temperature_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
      "/vehicle/sensor/brake_temperature_degree", qos,
      std::bind(&StabilityControllerNode::brake_temperature_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) brake_temperatures_ok_ = false;
      },
      timeout_values.value_ms("Default"));

  // Publishers
  ctrl_cmd_pub_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/core/control/control_command", qos);
  longitudinal_cmd_pub_ = this->create_publisher<tum_msgs::msg::TUMLongitudinalCmd>(
      "/core/control/longitudinal_request", qos);
  slip_control_active_pub_ = this->create_publisher<tum_msgs::msg::TUMBoolStamped>(
      "/core/control/slip_control_active", qos);
}
void StabilityControllerNode::function_queue_callback()
{
  topic_watchdog_->check_timeouts();

  model_update_callback();
  if (odometry_received_ && acceleration_received_ && steering_received_ && ctrl_cmd_received_ &&
      gear_request_received_ && gear_report_received_ && wheelspeed_received_ &&
      brake_pressure_received_ && longitudinal_cmd_received_)
  {
    node_monitor_->set_error_lvl(node_monitor_key_, tam::types::ErrorLvl::OK);

    node_monitor_->set_message("Ok");

    node_monitor_->set_error_lvl("Ok", tam::types::ErrorLvl::OK);
    node_monitor_->set_status_code(0);
    node_monitor_key_ = "Ok";
  }else if (odometry_received_ && ctrl_cmd_received_ && longitudinal_cmd_received_) {
    node_monitor_->set_error_lvl(node_monitor_key_, tam::types::ErrorLvl::OK);
    node_monitor_->set_message("Warn: Optional messages missing");
    node_monitor_->set_error_lvl("Warning", tam::types::ErrorLvl::WARN);
    node_monitor_->set_status_code(1);
    node_monitor_key_ = "Warning";
  }else{
    node_monitor_->set_error_lvl(node_monitor_key_, tam::types::ErrorLvl::OK);
    node_monitor_->set_message("Error: Essential messages missing");
    node_monitor_->set_error_lvl("Error", tam::types::ErrorLvl::ERROR);
    node_monitor_->set_status_code(2);
    node_monitor_key_ = "Error";
  }
  node_monitor_->update();
}
void StabilityControllerNode::model_update_callback()
{
  std::chrono::steady_clock::time_point callback_start_time = std::chrono::steady_clock::now();

  if (!odometry_received_ ||
      !ctrl_cmd_received_ || !longitudinal_cmd_received_)
  {
    return;
  }

  cycle_count_ += 1;

  if (param_manager_->get_state_hash() != previous_param_state_hash_) {
    declare_and_update_parameters();
  }

  // Determine slip-signal validity.
  const bool slip_angle_valid = odometry_received_ && (steering_received_ || ctrl_cmd_received_);
  const bool slip_valid = slip_angle_valid && wheelspeed_received_;
  slip_control_->set_slip_valid(slip_valid);
  esc_->set_slip_angle_valid(slip_angle_valid);
  countersteer_system_->set_slip_angle_valid(slip_angle_valid);

  slip_calculation_->set_steering_angle_valid(steering_received_);
  countersteer_system_->set_steering_valid(steering_received_);


  // Calculate slips before supplying them to the stability-control components.
  slip_calculation_->step();
  const auto wheelslips = slip_calculation_->get_wheelslips();
  const auto slip_angles = slip_calculation_->get_slip_angles();
  const auto vertical_tire_loads = slip_calculation_->get_vertical_tire_loads();

  // Interpolate brake friction from temperature when available.
  tam::types::common::DataPerWheel<double> brake_friction{};
  if (brake_temperatures_ok_ && brake_friction_map_valid_) {
    for (size_t i = 0; i < brake_friction.size(); ++i) {
      brake_friction[i] = tam::helpers::numerical::interp(
          brake_temperature_degree_[i], p_.brake_friction_temperature, p_.brake_friction_coeff);
    }
  } else {
    brake_friction = tam::types::common::DataPerWheel<double>(p_.brake_friction_default);
  }
  logger_->log("brake_friction_coefficient", brake_friction);
  slip_control_->set_brake_friction_coefficients(brake_friction);
  esc_->set_brake_friction_coefficients(brake_friction);

  esc_->step();

  slip_control_->set_current_timestamp(std::chrono::steady_clock::now());
  slip_control_->set_slips(
      wheelslips, slip_calculation_->get_virtual_wheelslips(),
      vertical_tire_loads, slip_angles);
  slip_control_->set_esc_active(esc_->get_esc_active());
  slip_control_->set_target_brake_pressure(esc_->get_brake_pressure_target_bar());
  slip_control_->step();

  countersteer_system_->step();

  builtin_interfaces::msg::Time stamp = get_clock()->now();

  // Publish control requests.
  autoware_auto_control_msgs::msg::AckermannControlCommand ctrl_out = ctrl_cmd_input_;
  ctrl_out.stamp = stamp;
  ctrl_out.lateral.stamp = stamp;
  ctrl_out.longitudinal.stamp = stamp;
  ctrl_out.lateral.steering_tire_angle = countersteer_system_->get_steering_request_rad();
  ctrl_cmd_pub_->publish(ctrl_out);

  tum_msgs::msg::TUMLongitudinalCmd longitudinal_cmd_out = longitudinal_cmd_input_;
  longitudinal_cmd_out.stamp = stamp;
  longitudinal_cmd_out.brake_pressure_pa = tam::type_conversions::data_per_wheel_msg_from_type(
      slip_control_->get_brake_pressure_target_bar() * 1e5);
  longitudinal_cmd_out.throttle = static_cast<float>(slip_control_->get_throttle_request());
  longitudinal_cmd_pub_->publish(longitudinal_cmd_out);

  tum_msgs::msg::TUMBoolStamped slip_control_active;
  slip_control_active.stamp = stamp;
  slip_control_active.data = slip_control_->get_status();
  slip_control_active_pub_->publish(slip_control_active);

  // Log timing diagnostics.
  logger_->log(
      "timing/exec_time_timer_callback_us",
      std::chrono::duration_cast<std::chrono::microseconds>(
          (std::chrono::steady_clock::now() - callback_start_time))
          .count());
  logger_->log(
      "timing/time_since_last_callback_us",
      std::chrono::duration_cast<std::chrono::microseconds>((callback_start_time - last_call_time))
          .count());
  logger_->log("timing/cycle_count", cycle_count_);

  tsl_publisher_->trigger();
  last_call_time = callback_start_time;
}
void StabilityControllerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!odometry_received_)
    odometry_received_ = true;
  odometry_ = tam::type::conversions::cpp::Odometry_type_from_msg(msg);
  slip_calculation_->set_feedback_odometry(odometry_);
  slip_control_->set_feedback_odometry(odometry_);
  esc_->set_feedback_odometry(odometry_);
  countersteer_system_->set_feedback_odometry(odometry_);
}
void StabilityControllerNode::acceleration_callback(
    const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  if (!acceleration_received_)
    acceleration_received_ = true;
  acceleration_ = tam::type_conversions::accel_with_covariance_stamped_type_from_msg(*msg);
  slip_calculation_->set_feedback_acceleration(acceleration_);
  countersteer_system_->set_feedback_acceleration(acceleration_);
}
void StabilityControllerNode::steering_callback(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
{
  if (!steering_received_)
    steering_received_ = true;
  steering_report_.steering_angle_tire_rad = msg->steering_tire_angle;
  slip_calculation_->set_feedback_steering_rad(steering_report_);
  slip_calculation_->set_steering_angle_valid(true);
  countersteer_system_->set_feedback_steering(steering_report_);
  countersteer_system_->set_steering_valid(true);
}
void StabilityControllerNode::gear_request_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg)
{
  if(!gear_request_received_)
    gear_request_received_ = true;
  slip_control_->set_gear_request(msg->data);
}
void StabilityControllerNode::gear_report_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg)
{
  if(!gear_report_received_)
    gear_report_received_ = true;
  drivetrain_feedback_.gear_engaged = msg->data;
  slip_calculation_->set_feedback_drivetrain(drivetrain_feedback_);
  slip_control_->set_feedback_gear(msg->data);
}
void StabilityControllerNode::control_command_callback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  if (!ctrl_cmd_received_)
    ctrl_cmd_received_ = true;
  ctrl_cmd_input_ = *msg;
  esc_->set_steering_target_rad(msg->lateral.steering_tire_angle);
  slip_calculation_->set_motion_control_steering_request(
      msg->lateral.steering_tire_angle);
  countersteer_system_->set_motion_control_steering_request(
      msg->lateral.steering_tire_angle);
}
void StabilityControllerNode::longitudinal_cmd_callback(
    const tum_msgs::msg::TUMLongitudinalCmd::SharedPtr msg)
{
  if (!longitudinal_cmd_received_)
    longitudinal_cmd_received_ = true;
  longitudinal_cmd_input_ = *msg;
  // Convert brake pressure from Pa to bar.
  tam::types::common::DataPerWheel<double> target_brake_pressure =
      tam::type_conversions::data_per_wheel_type_from_msg(msg->brake_pressure_pa) * 1e-5;
  slip_control_->set_target_brake_pressure(target_brake_pressure);
  esc_->set_target_brake_pressure(target_brake_pressure);
  slip_control_->set_throttle_target(msg->throttle);
}
void StabilityControllerNode::wheelspeed_callback(
    const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  wheelspeed_received_ = true;
  slip_calculation_->set_feedback_wheelspeed_radps(
      tam::type_conversions::data_per_wheel_type_from_msg(msg->data));
  slip_calculation_->set_wheelspeed_ok(true);
}
void StabilityControllerNode::omega_engine_callback(
    const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg)
{
  drivetrain_feedback_.omega_engine_radps = msg->data;
  slip_calculation_->set_feedback_drivetrain(drivetrain_feedback_);
  slip_calculation_->set_omega_engine_ok(true);
}
void StabilityControllerNode::brake_pressure_callback(
    const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  brake_pressure_received_ = true;
  slip_calculation_->set_feedback_brake_pressure_Pa(
      tam::type_conversions::data_per_wheel_type_from_msg(msg->data));
}
void StabilityControllerNode::brake_temperature_callback(
    const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  brake_temperature_degree_ = tam::type_conversions::data_per_wheel_type_from_msg(msg->data);
  brake_temperatures_ok_ = true;
}

void StabilityControllerNode::declare_and_update_parameters()
{
  p_.brake_friction_default =
      param_manager_
          ->declare_and_get_value(
              "vehicle.brake.friction_coeff", 0.35, tam::pmg::ParameterType::DOUBLE, "")
          .as_double();
  p_.brake_friction_temperature =
      param_manager_
          ->declare_and_get_value(
              "vehicle.brake.friction.temperature", std::vector<double>{0.0, 1000.0},
              tam::pmg::ParameterType::DOUBLE_ARRAY, "")
          .as_double_array();
  p_.brake_friction_coeff =
      param_manager_
          ->declare_and_get_value(
              "vehicle.brake.friction.coefficient", std::vector<double>{0.35, 0.35},
              tam::pmg::ParameterType::DOUBLE_ARRAY, "")
          .as_double_array();
  brake_friction_map_valid_ =
    !p_.brake_friction_temperature.empty() &&
    p_.brake_friction_temperature.size() == p_.brake_friction_coeff.size();
  previous_param_state_hash_ = param_manager_->get_state_hash();
}
void StabilityControllerNode::additional_esc_targets_callback(
    const tum_msgs::msg::TUMAdditionalEspTargets::SharedPtr msg)
{
  // Convert optional ESC-target flags.
  tam::types::control::AdditionalEspTargets additional_esc_targets{};
  additional_esc_targets.ay_request_mps2 =
    msg->ay_request_set ? std::make_optional<double>(msg->ay_request) : std::nullopt;
  additional_esc_targets.yaw_rate_request_radps =
    msg->yaw_rate_request_set ? std::make_optional<double>(msg->yaw_rate_request) : std::nullopt;
  additional_esc_targets.slip_angle_request_rad =
    msg->slip_angle_request_set ? std::make_optional<double>(msg->slip_angle_request)
                                : std::nullopt;

  esc_->set_additional_esc_targets(additional_esc_targets);
}
