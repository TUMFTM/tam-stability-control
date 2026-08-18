// Copyright 2025 Phillip Pitschi
#include "stability_control_tam_cpp/slip_control.hpp"

#include <cmath>
namespace tam::control
{
SlipController::SlipController() { declare_and_update_parameters(); }
void SlipController::declare_and_update_parameters()
{
  // Parameter helpers
  auto decl_double = [this](std::string name, double val) {
    return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  };
  auto decl_operation_mode = [this](const std::string & name, const OperationMode default_mode) {
    const int value = param_manager_
      ->declare_and_get_value(
        name, static_cast<int>(default_mode), tam::pmg::ParameterType::INTEGER, "")
      .as_int();
    if (
      value < static_cast<int>(OperationMode::individual) ||
      value > static_cast<int>(OperationMode::rear_only))
    {
      return default_mode;
    }
    return static_cast<OperationMode>(value);
  };

  // ABS parameters
  slip_control_params.abs_enabled =
    param_manager_
      ->declare_and_get_value("SlipControl.ABS.enabled", false, tam::pmg::ParameterType::BOOL, "")
      .as_bool();
  slip_control_params.abs_slip_vertical_load_weighted =
    param_manager_
      ->declare_and_get_value(
        "SlipControl.ABS.slip_vertical_load_weighted", false, tam::pmg::ParameterType::BOOL, "")
      .as_bool();
  slip_control_params.abs_use_virtual_slips =
    param_manager_
      ->declare_and_get_value(
        "SlipControl.ABS.use_virtual_slips", false, tam::pmg::ParameterType::BOOL, "")
      .as_bool();

  // TC parameters
  slip_control_params.tc_enabled =
    param_manager_
      ->declare_and_get_value("SlipControl.TC.enabled", false, tam::pmg::ParameterType::BOOL, "")
      .as_bool();
  slip_control_params.tc_slip_vertical_load_weighted =
    param_manager_
      ->declare_and_get_value(
        "SlipControl.TC.slip_vertical_load_weighted", false, tam::pmg::ParameterType::BOOL, "")
      .as_bool();
  slip_control_params.tc_use_virtual_slips =
    param_manager_
      ->declare_and_get_value(
        "SlipControl.TC.use_virtual_slips", false, tam::pmg::ParameterType::BOOL, "")
      .as_bool();

  // Distribution modes
  slip_control_params.abs_operation_mode =
    decl_operation_mode("SlipControl.ABS.operation_mode", OperationMode::front_rear_split);
  slip_control_params.tc_operation_mode =
    decl_operation_mode("SlipControl.TC.operation_mode", OperationMode::rear_only);

  // Shared timing and filtering
  slip_control_params.tS = decl_double("tS", 0.01);
  slip_control_params.slip_angle_filter_pole =
    std::clamp(decl_double("SlipControl.slip_angle_filter_pole", 0.7), 0.0, 1.0);

  // ABS limits
  slip_control_params.abs_min_activation_velocity =
    decl_double("SlipControl.ABS.min_activation_velocity", 5.0);
  slip_control_params.abs_cooldown_gear_change =
    decl_double("SlipControl.ABS.activation_time_after_gearshift", 0.8);

  // TC limits
  slip_control_params.tc_min_activation_velocity =
    decl_double("SlipControl.TC.min_activation_velocity", 0.0);
  slip_control_params.tc_cooldown_gear_change =
    decl_double("SlipControl.TC.activation_time_after_gearshift", 0.8);
  slip_control_params.tc_max_slip_throttle_cut =
    decl_double("SlipControl.TC.max_slip_throttle_cut", 25.0);

  // Gearshift pressure reduction
  slip_control_params.duration_gearshift_initiation =
    decl_double("SlipControl.shift_pressure_reduction.duration_gearshift_initiation", 0.1);
  slip_control_params.duration_gearshift_pressure_reduction =
    decl_double("SlipControl.shift_pressure_reduction.duration_pressure_reduction", 0.1);
  slip_control_params.gearshift_pressure_reduction_factor =
    decl_double("SlipControl.shift_pressure_reduction.reduction_factor", 0.7);

  // Update the slip-angle filter.
  slip_angle_filter.set_tf_pole(Dpw{slip_control_params.slip_angle_filter_pole});

  previous_param_state_hash = param_manager_->get_state_hash();
}
void SlipController::check_allowed()
{
  // Track gear changes for cooldowns.
  if (gear_request_change_detector.check_and_update(slip_control_state.gear_request)) {
    slip_control_state.time_gear_request_changed = current_timestamp_;
  }
  if (gear_change_detector.check_and_update(slip_control_state.gear)) {
    slip_control_state.time_gear_changed = current_timestamp_;
  }

  // Enable ABS only with valid slip above its speed limit.
  Bpw abs_allowed{
    (slip_control_params.abs_enabled && slip_control_inputs.slip_valid &&
     (slip_control_inputs.odometry.velocity_mps.x >
      slip_control_params.abs_min_activation_velocity))};
  // Apply the ABS gear-change cooldown.
  abs_allowed =
    abs_allowed &&
    Bpw{}.from_front_and_rear(
      true,
      std::chrono::duration<double>(current_timestamp_ - slip_control_state.time_gear_changed) >
          std::chrono::duration<double>(slip_control_params.abs_cooldown_gear_change) ||
        abs_controlled_wheels[2].get_is_active() || abs_controlled_wheels[3].get_is_active());

  // Enable TC only with valid slip above its speed limit.
  Bpw tc_allowed{
    slip_control_params.tc_enabled && slip_control_inputs.slip_valid &&
    (slip_control_inputs.odometry.velocity_mps.x > slip_control_params.tc_min_activation_velocity)};

  // Apply the TC cooldown and rear-only mode.
  tc_allowed =
    tc_allowed &&
    Bpw{}.from_front_and_rear(
      slip_control_params.tc_operation_mode != OperationMode::rear_only,
      std::chrono::duration<double>(current_timestamp_ - slip_control_state.time_gear_changed) >
          std::chrono::duration<double>(slip_control_params.tc_cooldown_gear_change) ||
        tc_controlled_wheels[2].get_is_active() || tc_controlled_wheels[3].get_is_active());

  // Propagate per-wheel enable flags.
  for (size_t i = 0; i < abs_inputs.size(); i++) {
    abs_inputs[i].allowed = abs_allowed[i];
    tc_inputs[i].allowed = tc_allowed[i];
  }
}
SlipController::Dpw SlipController::shift_pressure_reduction(
  const Dpw & brake_pressure_target) const
{
  // Time since the gear-change request.
  double time_after_gearshift_request =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      current_timestamp_ - slip_control_state.time_gear_request_changed)
      .count();

  // Reduce rear pressure during the configured gearshift interval.
  if (
    time_after_gearshift_request > slip_control_params.duration_gearshift_initiation * 1000.0 &&
    time_after_gearshift_request < (slip_control_params.duration_gearshift_initiation +
                                    slip_control_params.duration_gearshift_pressure_reduction) *
                                     1000.0) {
    // Keep front pressure and reduce rear pressure.
    return Dpw::from_front_and_rear(
             1.0, std::clamp(slip_control_params.gearshift_pressure_reduction_factor, 0.0, 1.0)) *
           brake_pressure_target;
  }
  return brake_pressure_target;
}
void SlipController::step()
{
  // Refresh parameters.
  if (param_manager_->get_state_hash() != previous_param_state_hash) {
    declare_and_update_parameters();
  }
  check_allowed();

  // Pass the requested throttle through unless valid slip data requires a TC intervention.
  slip_control_state.throttle_request = slip_control_state.throttle_target_in;

  // Run ABS.
  for (size_t i = 0; i < abs_controlled_wheels.size(); i++) {
    abs_controlled_wheels[i].set_generic_inputs(slip_control_inputs);
    abs_controlled_wheels[i].set_wheel_individual_inputs(abs_inputs[i]);
    abs_controlled_wheels[i].step();
  }

  // Run TC.
  for (size_t i = 0; i < tc_controlled_wheels.size(); i++) {
    tc_controlled_wheels[i].set_generic_inputs(slip_control_inputs);
    tc_controlled_wheels[i].set_wheel_individual_inputs(tc_inputs[i]);
    tc_controlled_wheels[i].step();
  }

  // Aggregate controller status.
  slip_control_state.abs_latched =
    abs_controlled_wheels[0].get_is_active() || abs_controlled_wheels[1].get_is_active() ||
    abs_controlled_wheels[2].get_is_active() || abs_controlled_wheels[3].get_is_active();
  slip_control_state.tc_latched =
    slip_control_params.tc_operation_mode != OperationMode::rear_only
      ? tc_controlled_wheels[0].get_is_active() || tc_controlled_wheels[1].get_is_active() ||
          tc_controlled_wheels[2].get_is_active() || tc_controlled_wheels[3].get_is_active()
      : tc_controlled_wheels[2].get_is_active() || tc_controlled_wheels[3].get_is_active();

  // Select ABS/TC pressure outputs.
  if (
    slip_control_state.abs_latched && (slip_control_inputs.long_fx < 0.0) &&
    !slip_control_inputs.esc_active) {
    for (size_t i = 0; i < abs_controlled_wheels.size(); i++) {
      slip_control_state.brake_pressure_target_bar[i] =
        abs_controlled_wheels[i].get_target_brake_pressure();
    }
  } else if (
    slip_control_state.tc_latched && slip_control_inputs.long_fx > 0.0 &&
    !slip_control_inputs.esc_active) {
    for (size_t i = 0; i < tc_controlled_wheels.size(); i++) {
      slip_control_state.brake_pressure_target_bar[i] =
        tc_controlled_wheels[i].get_target_brake_pressure();
    }
    // ESC can combine front ABS with rear TC.
  } else if (slip_control_inputs.esc_active) {
    if (slip_control_state.abs_latched) {
      for (size_t i = 0; i < abs_controlled_wheels.size(); i++) {
        slip_control_state.brake_pressure_target_bar[i] =
          abs_controlled_wheels[i].get_target_brake_pressure();
      }
    } else {
      slip_control_state.brake_pressure_target_bar = slip_control_state.brake_pressure_input_bar;
    }
    if (slip_control_state.tc_latched) {
      slip_control_state.brake_pressure_target_bar.rear_left =
        tc_controlled_wheels.rear_left.get_target_brake_pressure();
      slip_control_state.brake_pressure_target_bar.rear_right =
        tc_controlled_wheels.rear_right.get_target_brake_pressure();
    }
  } else {
    slip_control_state.brake_pressure_target_bar = slip_control_state.brake_pressure_input_bar;
  }

  // Apply gearshift pressure reduction.
  slip_control_state.brake_pressure_target_bar =
    shift_pressure_reduction(slip_control_state.brake_pressure_target_bar);

  // Cut throttle for excessive valid slip.
  if (slip_control_inputs.slip_valid) {
    slip_control_state.throttle_request =
      slip_control_state.throttle_target_in *
      (std::max(
         {tc_inputs.front_left.slip, tc_inputs.front_right.slip, tc_inputs.rear_left.slip,
          tc_inputs.rear_right.slip}) <= slip_control_params.tc_max_slip_throttle_cut);
  }

  logger_->log("ABS/state", slip_control_state.abs_latched);
  logger_->log("TC/state", slip_control_state.tc_latched);
}
void SlipController::distribute_slips(
  Dpw & slip, Dpw & slip_angle, Dpw & slip_rate,
  tam::types::common::DataPerWheel<WheelIndividualInputs> & input_abstc,
  OperationMode operation_mode, std::function<double(const double, const double)> minmax)
{
  // minmax is std::min for ABS and std::max for TC.
  for (size_t i = 0; i < input_abstc.size(); i++) {
    if (operation_mode == OperationMode::individual) {
      input_abstc[i].slip = slip[i];
      input_abstc[i].slip_angle = slip_angle[i];
      input_abstc[i].slip_rate = slip_rate[i];
    }
    else if (operation_mode == OperationMode::front_rear_split) {
      if (i < 2) {
        input_abstc[i].slip = minmax(slip.front_left, slip.front_right);
        input_abstc[i].slip_angle =
          std::max(std::abs(slip_angle.front_left), std::abs(slip_angle.front_right));
        input_abstc[i].slip_rate = std::abs(slip.front_left) > std::abs(slip.front_right)
                                     ? slip_rate.front_left
                                     : slip_rate.front_right;
      } else {
        input_abstc[i].slip = minmax(slip.rear_left, slip.rear_right);
        input_abstc[i].slip_angle =
          std::max(std::abs(slip_angle.rear_left), std::abs(slip_angle.rear_right));
        input_abstc[i].slip_rate = std::abs(slip.rear_left) > std::abs(slip.rear_right)
                                     ? slip_rate.rear_left
                                     : slip_rate.rear_right;
      }
    } else if (operation_mode == OperationMode::all_together) {
      const double selected_slip =
        minmax(minmax(minmax(slip.front_left, slip.front_right), slip.rear_left), slip.rear_right);
      input_abstc[i].slip = selected_slip;
      input_abstc[i].slip_angle = std::max(
        {std::abs(slip_angle.front_left), std::abs(slip_angle.front_right),
         std::abs(slip_angle.rear_left), std::abs(slip_angle.rear_right)});
      auto slip_array = slip.to_array();
      const auto selected_slip_it = std::find(
        slip_array.begin(), slip_array.end(), selected_slip);
      input_abstc[i].slip_rate =
        slip_rate[std::distance(slip_array.begin(), selected_slip_it)];
    } else if (operation_mode == OperationMode::rear_only) {
      if (i < 2) {
        input_abstc[i].slip = 0.0;
        input_abstc[i].slip_angle = 0.0;
        input_abstc[i].slip_rate = 0.0;
      } else {
        input_abstc[i].slip = minmax(slip.rear_left, slip.rear_right);
        input_abstc[i].slip_angle =
          std::max(std::abs(slip_angle.rear_left), std::abs(slip_angle.rear_right));
        input_abstc[i].slip_rate = std::abs(slip.rear_left) > std::abs(slip.rear_right)
                                     ? slip_rate.rear_left
                                     : slip_rate.rear_right;
      }
    }
  }
}
void SlipController::set_slips(
  const Dpw & slip, const Dpw & virtual_slip, const Dpw & vertical_load,
  const Dpw & slip_angle)
{
  // Derive slip rate from the raw signal.
  Dpw slip_rate_raw =
    (slip - slip_control_state.slip) / std::max(slip_control_params.tS, 1e-6);
  slip_control_state.slip = slip;

  // Use raw per-wheel slip if an axle load is unavailable.
  Dpw vertical_load_weighted_slip = slip;
  const double front_vertical_load = vertical_load.front_left + vertical_load.front_right;
  if (front_vertical_load > 1e-6) {
    const double front_slip =
      (slip.front_left * vertical_load.front_left + slip.front_right * vertical_load.front_right) /
      front_vertical_load;
    vertical_load_weighted_slip.front_left = front_slip;
    vertical_load_weighted_slip.front_right = front_slip;
  }
  const double rear_vertical_load = vertical_load.rear_left + vertical_load.rear_right;
  if (rear_vertical_load > 1e-6) {
    const double rear_slip =
      (slip.rear_left * vertical_load.rear_left + slip.rear_right * vertical_load.rear_right) /
      rear_vertical_load;
    vertical_load_weighted_slip.rear_left = rear_slip;
    vertical_load_weighted_slip.rear_right = rear_slip;
  }

  // Filter slip-angle magnitude.
  Dpw filtered_slip_angle = std::abs(slip_angle_filter.step(slip_angle));

  // Select the configured ABS slip source.
  Dpw slips{};
  Dpw slip_rate{};
  if (
    slip_control_params.abs_use_virtual_slips &&
    slip_control_params.abs_slip_vertical_load_weighted) {
    slips =
      Dpw::from_front_and_rear(vertical_load_weighted_slip.front_left, virtual_slip.rear_left);
    double slip_rate_front =
      slip.front_left < slip.front_right ? slip_rate_raw.front_left : slip_rate_raw.front_right;
    double slip_rate_rear =
      slip.rear_left < slip.rear_right ? slip_rate_raw.rear_left : slip_rate_raw.rear_right;
    slip_rate = Dpw{}.from_front_and_rear(slip_rate_front, slip_rate_rear);
  } else if (slip_control_params.abs_use_virtual_slips) {
    slips = virtual_slip;
    double slip_rate_rear =
      slip.rear_left < slip.rear_right ? slip_rate_raw.rear_left : slip_rate_raw.rear_right;
    slip_rate =
      Dpw{slip_rate_raw.front_left, slip_rate_raw.front_right, slip_rate_rear, slip_rate_rear};
  } else if (slip_control_params.abs_slip_vertical_load_weighted) {
    double slip_rate_front =
      slip.front_left < slip.front_right ? slip_rate_raw.front_left : slip_rate_raw.front_right;
    double slip_rate_rear =
      slip.rear_left < slip.rear_right ? slip_rate_raw.rear_left : slip_rate_raw.rear_right;
    slip_rate = Dpw{}.from_front_and_rear(slip_rate_front, slip_rate_rear);
    slips = vertical_load_weighted_slip;
  } else {
    slips = slip;
    slip_rate = slip_rate_raw;
  }

  // ABS uses the minimum slip in each control group.
  distribute_slips(
    slips, filtered_slip_angle, slip_rate, abs_inputs, slip_control_params.abs_operation_mode,
    [](const double & a, const double & b) { return std::min(a, b); });

  // Select the configured TC slip source.
  if (
    slip_control_params.tc_use_virtual_slips &&
    slip_control_params.tc_slip_vertical_load_weighted) {
    slips =
      Dpw::from_front_and_rear(vertical_load_weighted_slip.front_left, virtual_slip.rear_left);
    double slip_rate_front =
      slip.front_left > slip.front_right ? slip_rate_raw.front_left : slip_rate_raw.front_right;
    double slip_rate_rear =
      slip.rear_left > slip.rear_right ? slip_rate_raw.rear_left : slip_rate_raw.rear_right;
    slip_rate = Dpw{}.from_front_and_rear(slip_rate_front, slip_rate_rear);
  } else if (slip_control_params.tc_use_virtual_slips) {
    slips = virtual_slip;
    double slip_rate_rear =
      slip.rear_left > slip.rear_right ? slip_rate_raw.rear_left : slip_rate_raw.rear_right;
    slip_rate =
      Dpw{slip_rate_raw.front_left, slip_rate_raw.front_right, slip_rate_rear, slip_rate_rear};
  } else if (slip_control_params.tc_slip_vertical_load_weighted) {
    double slip_rate_front =
      slip.front_left > slip.front_right ? slip_rate_raw.front_left : slip_rate_raw.front_right;
    double slip_rate_rear =
      slip.rear_left > slip.rear_right ? slip_rate_raw.rear_left : slip_rate_raw.rear_right;
    slip_rate = Dpw{}.from_front_and_rear(slip_rate_front, slip_rate_rear);
    slips = vertical_load_weighted_slip;
  } else {
    slips = slip;
    slip_rate = slip_rate_raw;
  }

  // TC uses the maximum slip in each control group.
  distribute_slips(
    slips, filtered_slip_angle, slip_rate, tc_inputs, slip_control_params.tc_operation_mode,
    [](const double & a, const double & b) { return std::max(a, b); });

  logger_->log("vertical_load", vertical_load);
  logger_->log("vertical_load_weighted_slip", vertical_load_weighted_slip);
  logger_->log("slip_rate", slip_rate);
  logger_->log("slip_rate_raw", slip_rate_raw);
}
void SlipController::set_slip_valid(const bool slip_valid)
{
  slip_control_inputs.slip_valid = slip_valid;
}
void SlipController::set_long_fx(const double long_fx)
{
  slip_control_inputs.long_fx = long_fx;
}
void SlipController::set_target_brake_pressure(Dpw target_brake_pressure)
{
  slip_control_state.brake_pressure_input_bar = target_brake_pressure;
  for (size_t i = 0; i < abs_inputs.size(); i++) {
    abs_inputs[i].brake_pressure = target_brake_pressure[i];
    tc_inputs[i].brake_pressure = target_brake_pressure[i];
  }
}
void SlipController::set_brake_friction_coefficients(const Dpw & brake_friction)
{
  // Clamp brake friction to a positive finite value.
  const auto brake_friction_values = brake_friction.to_array();
  for (size_t i = 0; i < abs_inputs.size(); i++) {
    const double validated_friction =
      std::isfinite(brake_friction_values[i]) ? std::max(brake_friction_values[i], 1e-6) : 1e-6;
    abs_inputs[i].brake_friction = validated_friction;
    tc_inputs[i].brake_friction = validated_friction;
  }
}
void SlipController::set_throttle_target(const double throttle_target)
{
  slip_control_state.throttle_target_in = throttle_target;
}
void SlipController::set_gear_request(const int8_t gear)
{
  slip_control_state.gear_request = gear;
}
void SlipController::set_feedback_gear(const int8_t gear)
{
  slip_control_state.gear = gear;
}
void SlipController::set_feedback_odometry(const tam::types::control::Odometry & odometry)
{
  slip_control_inputs.odometry = odometry;
}
void SlipController::set_esc_active(const bool esc_active)
{
  slip_control_inputs.esc_active = esc_active;
}
void SlipController::set_current_timestamp(const std::chrono::steady_clock::time_point & timestamp)
{
  // Propagate the timestamp to TC controllers.
  current_timestamp_ = timestamp;
  for (size_t i = 0; i < abs_controlled_wheels.size(); i++) {
    tc_controlled_wheels[i].set_current_timestamp(timestamp);
  }
}
tam::types::common::DataPerWheel<double> SlipController::get_brake_pressure_target_bar() const
{
  return slip_control_state.brake_pressure_target_bar;
}
double SlipController::get_throttle_request() const
{
  return slip_control_state.throttle_request;
}
bool SlipController::get_status() const
{
  return slip_control_state.abs_latched || slip_control_state.tc_latched;
}
tam::tsl::LoggerAccessInterface::SharedPtr SlipController::get_debug_out() const
{
  return logger_composer_;
}
tam::pmg::MgmtInterface::SharedPtr SlipController::get_param_handler() const
{
  return param_manager_composer_;
}
}  // namespace tam::control
