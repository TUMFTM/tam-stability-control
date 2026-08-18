// Copyright 2025 Phillip Pitschi
#include "stability_control_tam_cpp/abs.hpp"

#include <algorithm>
#include <cmath>

namespace tam::control
{
bool ABSControlledWheel::activate() const
{
  // Activate for excessive braking slip or an ESC intervention.
  return (
    (abs_inputs_.slip <= abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_reduce &&
     abs_inputs_.slip_rate < 0.0) &&
    (params_.min_long_force - 100 > slip_control_inputs_.long_fx ||
     slip_control_inputs_.esc_active) &&
    abs_inputs_.allowed);
}
bool ABSControlledWheel::deactivate() const
{
  // Deactivate when braking demand ends, ESC recovers, or control is disallowed.
  return (
    (params_.min_long_force < slip_control_inputs_.long_fx && !abs_state_.esp_trigger) ||
    (abs_inputs_.slip > abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_reduce &&
     abs_state_.esp_trigger) ||
    !abs_inputs_.allowed);
}
void ABSControlledWheel::log_debug_values()
{
  logger_->log("slip", abs_inputs_.slip);
  logger_->log(
    "slip_threshold_hold_reduce",
    abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_reduce);
  logger_->log(
    "slip_threshold_reduce_hold",
    abs_state_.slip_threshold_reduction * params_.slip_threshold_reduce_hold);
  logger_->log(
    "slip_threshold_hold_increase",
    abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_increase);
  logger_->log(
    "slip_threshold_increase_hold",
    abs_state_.slip_threshold_reduction * params_.slip_threshold_increase_hold);
  logger_->log(
    "slip_threshold_safe", abs_state_.slip_threshold_reduction * params_.slip_threshold_safe);
  logger_->log("wheel_state", static_cast<int>(abs_state_.wheel_state));
  logger_->log("allowed", abs_inputs_.allowed);
  logger_->log("long_fx_latched", abs_state_.long_fx_latched);
  logger_->log("long_fx_output", abs_state_.long_fx_output);
  logger_->log("brake_pressure_latched", abs_state_.brake_pressure_latched);
  logger_->log("brake_pressure_output", abs_state_.brake_pressure_output);
  logger_->log("pressure_rate", abs_state_.pressure_rate);
  logger_->log("safety_feature_active", abs_state_.safety_feature_is_active);
  logger_->log("reduction_factor", abs_state_.reduction_factor);
  logger_->log("slip_threshold_reduction", abs_state_.slip_threshold_reduction);
  logger_->log("slip_rate", abs_inputs_.slip_rate);
  logger_->log("slip_error", abs_state_.slip_error);
}
void ABSControlledWheel::update_latched_brake_pressure()
{
  // Preserve braking force as estimated brake friction changes.
  abs_state_.brake_pressure_latched = abs_state_.current_brake_friction /
                                      abs_inputs_.brake_friction *
                                      abs_state_.brake_pressure_latched;
  abs_state_.current_brake_friction = abs_inputs_.brake_friction;
}
void ABSControlledWheel::reduce_pressure_ratio(const double reduction_factor)
{
  // Reduce brake pressure by the normalized slip error.
  abs_state_.pressure_rate =
    std::clamp(abs_state_.pressure_rate - abs_state_.slip_error * reduction_factor, 0.0, 1.0);
}
void ABSControlledWheel::increase_pressure_ratio(const double increase_factor)
{
  // Increase brake pressure by a fixed or error-weighted step.
  double addition =
    params_.use_error_in_increase ? abs_state_.slip_error * increase_factor : increase_factor;
  abs_state_.pressure_rate = abs_state_.pressure_rate + addition;
}
Wheel_States ABSControlledWheel::transitions(const Wheel_States & state)
{
  switch (state) {
    case Wheel_States::Inactive:
      if (activate()) {
        // Latch the activation conditions.
        abs_state_.long_fx_latched = slip_control_inputs_.long_fx;
        abs_state_.brake_pressure_latched = abs_inputs_.brake_pressure;
        abs_state_.v_latched = slip_control_inputs_.odometry.velocity_mps.x;

        // Detect an ESC-triggered intervention.
        abs_state_.esp_trigger = params_.min_long_force - 100 <= slip_control_inputs_.long_fx;

        // Start with the configured reduction.
        abs_state_.pressure_rate = params_.reduce_initial_pressure_ratio;
        return Wheel_States::Hold;
      }
      return Wheel_States::Inactive;
    case Wheel_States::Hold:
      if (deactivate()) return Wheel_States::Inactive;
      // Reduce pressure for falling low slip.
      if (
        abs_inputs_.slip <
          abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_reduce &&
        abs_inputs_.slip_rate < 0.0) {
        reduce_pressure_ratio(params_.reduce_step);
        return Wheel_States::Reduce;
      }
      // Increase pressure for rising high slip.
      if (
        abs_inputs_.slip >
          abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_increase &&
        abs_inputs_.slip_rate > 0.0) {
        increase_pressure_ratio(params_.increase_step);
        abs_state_.pressure_rate = std::clamp(abs_state_.pressure_rate, 0.0, 1.0);
        abs_state_.pressure_rate =
          std::max(params_.increase_initial_pressure_ratio, abs_state_.pressure_rate);
        return Wheel_States::Increase;
      }
      return Wheel_States::Hold;
    case Wheel_States::Reduce:
      if (deactivate()) return Wheel_States::Inactive;
      // Increase pressure for rising high slip.
      if (
        abs_inputs_.slip >
          abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_increase &&
        abs_inputs_.slip_rate > 0.0) {
        increase_pressure_ratio(params_.increase_step);
        abs_state_.pressure_rate = std::clamp(abs_state_.pressure_rate, 0.0, 1.0);
        abs_state_.pressure_rate =
          std::max(params_.increase_initial_pressure_ratio, abs_state_.pressure_rate);
        return Wheel_States::Increase;
      }
      // Hold once slip recovers or starts rising.
      if (
        abs_inputs_.slip_rate > 0.0 || abs_inputs_.slip > abs_state_.slip_threshold_reduction *
                                                            params_.slip_threshold_reduce_hold) {
        return Wheel_States::Hold;
      }
      // Continue reducing pressure.
      reduce_pressure_ratio(params_.reduce_step);
      return Wheel_States::Reduce;
    case Wheel_States::Increase:
      if (deactivate()) {
        return Wheel_States::Inactive;
      }
      // Reduce pressure for falling low slip.
      if (
        abs_inputs_.slip <
          abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_reduce &&
        abs_inputs_.slip_rate < 0.0) {
        abs_state_.pressure_rate =
          std::min(abs_state_.pressure_rate, params_.reduce_initial_pressure_ratio);
        // Relatch after pressure-rate overshoot.
        if (abs_state_.pressure_rate > 1.001) {
          abs_state_.reduction_factor = 0.9;
          abs_state_.long_fx_latched = abs_state_.long_fx_output;
          abs_state_.brake_pressure_latched = abs_state_.brake_pressure_output;
          abs_state_.pressure_rate = params_.reduce_initial_pressure_ratio;
          abs_state_.v_latched = slip_control_inputs_.odometry.velocity_mps.x;
        }
        return Wheel_States::Reduce;
      }
      // Hold at the upper slip limit.
      if (
        (abs_inputs_.slip_rate < 0.0 ||
         abs_inputs_.slip <
           abs_state_.slip_threshold_reduction * params_.slip_threshold_increase_hold) &&
        abs_inputs_.slip < abs_state_.slip_threshold_reduction * params_.slip_threshold_safe) {
        return Wheel_States::Hold;
      }
      // Use a conservative step after saturation.
      if (abs_state_.pressure_rate > 0.999) {
        if (
          (abs_inputs_.slip > abs_state_.slip_threshold_reduction * params_.slip_threshold_safe) ||
          (abs_inputs_.slip >
             abs_state_.slip_threshold_reduction * params_.slip_threshold_reduce_hold &&
           abs_inputs_.slip_rate > 0.0)) {
          increase_pressure_ratio(params_.increase_step_target_increase);
        }
      } else {
        increase_pressure_ratio(params_.increase_step);
        abs_state_.pressure_rate = std::clamp(abs_state_.pressure_rate, 0.0, 1.0);
      }
      return Wheel_States::Increase;
    default:
      std::cout << "[ABS]: Error: Invalid state!" << std::endl;
      return Wheel_States::Inactive;
  }
}
void ABSControlledWheel::update_safety_feature()
{
  // Limit excessive ABS force reduction.
  abs_state_.safety_feature_is_active = false;
  double long_fx_output_average{}, long_fx_input_average{};
  abs_state_.long_fx_output_vector.insert(
    abs_state_.long_fx_output > 0.0 ? 0.0 : abs_state_.long_fx_output);
  long_fx_output_average =
    (std::reduce(abs_state_.long_fx_output_vector.begin(), abs_state_.long_fx_output_vector.end()) /
     params_.MovingAverageFxWindowLength);
  abs_state_.long_fx_input_vector.insert(
    slip_control_inputs_.long_fx > 0.0 ? 0.0 : slip_control_inputs_.long_fx);
  long_fx_input_average =
    (std::reduce(abs_state_.long_fx_input_vector.begin(), abs_state_.long_fx_input_vector.end()) /
     params_.MovingAverageFxWindowLength);

  if (
    (long_fx_output_average > params_.fx_safety_threshold_factor_ * long_fx_input_average) &&
    (abs_state_.wheel_state > Wheel_States::Inactive)) {
    abs_state_.long_fx_output = std::min(
      params_.fx_safety_threshold_factor_ * slip_control_inputs_.long_fx,
      abs_state_.long_fx_output);
    abs_state_.safety_feature_is_active = true;
  }

  logger_->log("long_fx_output_average", long_fx_output_average);
  logger_->log("long_fx_input_average", long_fx_input_average);
}
void ABSControlledWheel::calculate_reduction_factor()
{
  // Account for aerodynamic load changes since ABS activation.
  if (abs_state_.wheel_state == Wheel_States::Inactive) {
    abs_state_.reduction_factor = 0.9;
  } else {
    double factor_fz_aero =
      -0.5 * params_.air_density * params_.cross_track_area * params_.lift_coeff;
    double Fz_static = params_.mass * 9.81;
    const double vertical_load =
      Fz_static + factor_fz_aero * std::pow(abs_state_.v_latched, 2);
    if (std::abs(vertical_load) < 1e-6) {
      abs_state_.reduction_factor = 0.9;
    } else {
      abs_state_.reduction_factor =
        0.9 * std::clamp(
                1.0 - (factor_fz_aero * (std::pow(abs_state_.v_latched, 2) -
                                         std::pow(slip_control_inputs_.odometry.velocity_mps.x, 2))) /
                        vertical_load,
                0.33, 1.0);
    }
  }
}
void ABSControlledWheel::update_slip_threshold_reduction()
{
  // Tighten longitudinal-slip thresholds as lateral slip grows.
  const double slip_angle_max = std::max(params_.slip_angle_max, 1e-6);
  abs_state_.slip_threshold_reduction =
    (1.0 - std::clamp(abs_inputs_.slip_angle / slip_angle_max, 0.0, 1.0) *
             params_.slip_threshold_reduction_factor);
}
void ABSControlledWheel::convert_fx_to_brake_pressure()
{
  // Convert the ABS force command to brake pressure.
  if (!slip_control_inputs_.esc_active || abs_state_.safety_feature_is_active) {
    double fx_to_brake_pressure_factor = params_.get_fx_to_brake_pressure_factor(
      slip_control_inputs_.odometry.velocity_mps.x, abs_inputs_.brake_friction);
    double brake_pressure_reduction =
      fx_to_brake_pressure_factor *
      (abs_state_.long_fx_output - abs_state_.reduction_factor * abs_state_.long_fx_latched);
    abs_state_.brake_pressure_output =
      (abs_state_.wheel_state == Wheel_States::Inactive)
        ? abs_inputs_.brake_pressure
        : std::max(
            abs_state_.reduction_factor * abs_state_.brake_pressure_latched -
              brake_pressure_reduction,
            0.0);
  } else {
    abs_state_.brake_pressure_output =
      (abs_state_.wheel_state == Wheel_States::Inactive)
        ? abs_inputs_.brake_pressure
        : std::max(
            abs_state_.reduction_factor * abs_state_.pressure_rate *
              abs_state_.brake_pressure_latched,
            0.0);
  }
  // ABS cannot exceed the baseline pressure.
  abs_state_.brake_pressure_output =
    std::min(abs_inputs_.brake_pressure, abs_state_.brake_pressure_output);
}
void ABSControlledWheel::calculate_slip_error()
{
  // Normalize slip error to the hold-reduce threshold.
  const double target_slip_threshold =
    abs_state_.slip_threshold_reduction * params_.slip_threshold_hold_reduce;
  const double target_slip_threshold_magnitude =
    std::max(std::abs(target_slip_threshold), 1e-6);
  abs_state_.slip_error = std::clamp(
    std::abs((target_slip_threshold - abs_inputs_.slip) /
             target_slip_threshold_magnitude),
    1 / 10.0, 1.0);
}
void ABSControlledWheel::step()
{
  // Refresh parameters and moving-average buffers.
  if (params_.param_changed()) {
    params_.declare_and_update_parameters();
    abs_state_.long_fx_output_vector.resize(params_.MovingAverageFxWindowLength);
    abs_state_.long_fx_input_vector.resize(params_.MovingAverageFxWindowLength);
  }

  calculate_reduction_factor();

  update_slip_threshold_reduction();

  calculate_slip_error();

  update_latched_brake_pressure();

  abs_state_.wheel_state = transitions(abs_state_.wheel_state);

  // Scale the latched force by the ABS control ratio.
  abs_state_.long_fx_output =
    abs_state_.pressure_rate * abs_state_.reduction_factor * abs_state_.long_fx_latched;

  // ABS cannot increase braking force.
  abs_state_.long_fx_output = std::max(abs_state_.long_fx_output, slip_control_inputs_.long_fx);

  // Pass through the baseline force while inactive.
  if (abs_state_.wheel_state == Wheel_States::Inactive) {
    abs_state_.long_fx_output = slip_control_inputs_.long_fx;
  }

  update_safety_feature();

  convert_fx_to_brake_pressure();

  log_debug_values();
}
void ABSControlledWheel::set_wheel_individual_inputs(const WheelIndividualInputs & abs_inputs)
{
  abs_inputs_ = abs_inputs;
}
void ABSControlledWheel::set_generic_inputs(const SlipControlInputs & generic_inputs)
{
  slip_control_inputs_ = generic_inputs;
}
double ABSControlledWheel::get_target_brake_pressure() const
{
  return abs_state_.brake_pressure_output;
}
bool ABSControlledWheel::get_is_active() const
{
  // Hold, Reduce, and Increase are active ABS states.
  if (abs_state_.wheel_state >= Wheel_States::Reduce) {
    return true;
  }
  return false;
}
tam::pmg::MgmtInterface::SharedPtr ABSControlledWheel::get_param_manager() const
{
  return params_.get_param_manager();
}
tam::tsl::LoggerAccessInterface::SharedPtr ABSControlledWheel::get_debug_out() const
{
  return logger_;
}
}  // namespace tam::control
