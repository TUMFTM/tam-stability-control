// Copyright 2025 Phillip Pitschi
#include "stability_control_tam_cpp/tc.hpp"

#include <algorithm>
#include <cmath>

namespace tam::control
{
bool TCControlledWheel::activate() const
{
  // Activate for excessive drive-wheel slip.
  return (
    tc_state_.current_slip_ >=
      tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_reduce &&
    tc_inputs_.allowed && slip_control_inputs_.long_fx > 0.0);
}
bool TCControlledWheel::deactivate() const
{
  // Deactivate after the slip timeout or when control is disallowed.
  return (
    (std::chrono::duration<double>(current_timestamp_ - tc_state_.last_time_threshold_exceeded)) >
      std::chrono::duration<double>(tc_params_.deactivation_duration) ||
    !tc_inputs_.allowed);
}
void TCControlledWheel::log_debug_values()
{
  logger_->log("slip", tc_state_.current_slip_);
  logger_->log("slip_angle", tc_inputs_.slip_angle);
  logger_->log("slip_unfiltered", tc_inputs_.slip);
  logger_->log(
    "slip_threshold_hold_reduce",
    tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_reduce);
  logger_->log(
    "slip_threshold_reduce_hold",
    tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_reduce_hold);
  logger_->log(
    "slip_threshold_hold_increase",
    tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_increase);
  logger_->log(
    "slip_threshold_increase_hold",
    tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_increase_hold);
  logger_->log(
    "slip_threshold_safe", tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_safe);
  logger_->log("wheel_state", static_cast<int>(tc_state_.wheel_state));
  logger_->log("allowed", tc_inputs_.allowed);
  logger_->log("long_fx_output", tc_state_.long_fx_output);
  logger_->log("target_brake_pressure_latched", tc_state_.target_brake_pressure_latched);
  logger_->log("target_brake_pressure_output", tc_state_.target_brake_pressure_output);
  logger_->log("pressure_rate", tc_state_.pressure_rate);
  logger_->log("slip_threshold_reduction", tc_state_.slip_threshold_reduction);
  logger_->log("slip_error", tc_state_.slip_error);
  logger_->log("slip_rate", tc_state_.slip_rate);
}
void TCControlledWheel::reduce_target()
{
  // Reduce retained drive force to add braking.
  tc_state_.pressure_rate =
    std::clamp(tc_state_.pressure_rate - tc_state_.slip_error * tc_params_.reduce_step, 0.0, 1.0);
}
void TCControlledWheel::increase_target()
{
  // Increase retained drive force to release braking.
  double addition = tc_params_.use_error_in_increase
                      ? tc_state_.slip_error * tc_params_.increase_step
                      : tc_params_.increase_step;
  tc_state_.pressure_rate = std::clamp(tc_state_.pressure_rate + addition, 0.0, 1.0);
}
Wheel_States TCControlledWheel::transitions(const Wheel_States & state)
{
  switch (state) {
    case Wheel_States::Inactive:
      if (activate()) {
        tc_state_.pressure_rate = tc_params_.reduce_initial_pressure_ratio;
        tc_state_.target_brake_pressure_latched = tc_inputs_.brake_pressure;
        return Wheel_States::Hold;
      }
      return Wheel_States::Inactive;

    case Wheel_States::Hold:
      if (deactivate()) return Wheel_States::Inactive;

      // Add braking for high, rising slip.
      if (
        tc_state_.current_slip_ >
          tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_reduce &&
        tc_state_.slip_rate > 0.0) {
        reduce_target();
        return Wheel_States::Reduce;
      }

      // Release braking for low, falling slip.
      if (
        tc_state_.current_slip_ <=
          tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_increase &&
        tc_state_.slip_rate < 0.0) {
        increase_target();
        return Wheel_States::Increase;
      }
      return Wheel_States::Hold;

    case Wheel_States::Reduce:
      if (deactivate()) return Wheel_States::Inactive;

      // Release braking for low, falling slip.
      if (
        tc_state_.current_slip_ <=
          tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_increase &&
        tc_state_.slip_rate < 0.0) {
        increase_target();
        return Wheel_States::Increase;
      }

      // Hold after slip recovery or a trend reversal.
      if (
        tc_state_.slip_rate < 0.0 ||
        tc_state_.current_slip_ <
          tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_reduce_hold) {
        return Wheel_States::Hold;
      }

      // Continue adding braking.
      reduce_target();
      return Wheel_States::Reduce;

    case Wheel_States::Increase:
      if (deactivate()) return Wheel_States::Inactive;

      // Add braking for high, rising slip.
      if (
        tc_state_.current_slip_ >
          tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_reduce &&
        tc_state_.slip_rate > 0.0) {
        tc_state_.pressure_rate =
          std::min(tc_state_.pressure_rate, tc_params_.reduce_initial_pressure_ratio);
        return Wheel_States::Reduce;
      }

      // Hold at the safe-slip boundary.
      if (
        (tc_state_.slip_rate > 0.0 ||
         tc_state_.current_slip_ >
           tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_increase_hold) &&
        tc_state_.current_slip_ >
          tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_safe) {
        return Wheel_States::Hold;
      }

      // Continue releasing braking.
      increase_target();
      return Wheel_States::Increase;

    default:
      std::cout << "[TC]: Error: Invalid state!" << std::endl;
      return Wheel_States::Inactive;
  }
}
void TCControlledWheel::update_slip_threshold_reduction()
{
  // Tighten longitudinal-slip thresholds as lateral slip grows.
  const double slip_angle_max = std::max(tc_params_.slip_angle_max, 1e-6);
  tc_state_.slip_threshold_reduction =
    (1.0 - std::clamp(tc_inputs_.slip_angle / slip_angle_max, 0.0, 1.0) *
             tc_params_.slip_threshold_reduction_factor);
}
void TCControlledWheel::calculate_slip_error()
{
  // Normalize slip error to the active hold-reduce threshold.
  const double target_slip_threshold =
    tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_reduce;
  const double target_slip_threshold_magnitude =
    std::max(std::abs(target_slip_threshold), 1e-6);
  tc_state_.slip_error = std::clamp(
    std::abs((target_slip_threshold - tc_state_.current_slip_) /
             target_slip_threshold_magnitude),
    1 / 10.0, 1.0);
}
void TCControlledWheel::convert_fx_to_brake_pressure()
{
  // Convert the requested drive-force reduction to brake pressure.
  double fx_to_brake_pressure_factor = tc_params_.get_fx_to_brake_pressure_factor(
    slip_control_inputs_.odometry.velocity_mps.x, tc_inputs_.brake_friction);

  tc_state_.target_brake_pressure_output =
    (tc_state_.long_fx_output <= slip_control_inputs_.long_fx) * (-1.0) *
    fx_to_brake_pressure_factor * (tc_state_.long_fx_output - slip_control_inputs_.long_fx);

  // TC may add brake pressure but never reduce the baseline request.
  tc_state_.target_brake_pressure_output =
    std::max(tc_inputs_.brake_pressure, tc_state_.target_brake_pressure_output);
}
void TCControlledWheel::step()
{
  // Refresh the filter after parameter changes.
  if (tc_params_.param_changed()) {
    tc_params_.declare_and_update_parameters();
    slip_filter_.set_tf_pole(tc_params_.slip_filter_pole);
  }

  update_slip_threshold_reduction();

  calculate_slip_error();

  tc_state_.wheel_state = transitions(tc_state_.wheel_state);

  // pressure_rate is the retained drive-force fraction; lower values add braking.
  tc_state_.long_fx_output = tc_state_.pressure_rate * slip_control_inputs_.long_fx;

  convert_fx_to_brake_pressure();

  // Pass through braking requests and inactive TC.
  if (tc_state_.wheel_state == Wheel_States::Inactive || slip_control_inputs_.long_fx < 0.0) {
    tc_state_.long_fx_output = slip_control_inputs_.long_fx;
    tc_state_.target_brake_pressure_output = tc_inputs_.brake_pressure;
  }

  // Track the latest threshold crossing for deactivation.
  if (
    tc_state_.current_slip_ >=
    tc_state_.slip_threshold_reduction * tc_params_.slip_threshold_hold_reduce) {
    tc_state_.last_time_threshold_exceeded = current_timestamp_;
  }

  log_debug_values();
}
void TCControlledWheel::set_wheel_individual_inputs(const WheelIndividualInputs & inputs)
{
  tc_inputs_ = inputs;

  // Filter slip before calculating its rate.
  double new_slip = slip_filter_.step(tc_inputs_.slip);

  tc_state_.slip_rate =
    (new_slip - tc_state_.current_slip_) / std::max(tc_params_.tS, 1e-6);

  tc_state_.current_slip_ = new_slip;

  // Preserve force reduction when the baseline force changes.
  if (
    tc_state_.wheel_state != Wheel_States::Inactive &&
    std::abs(slip_control_inputs_.long_fx) >= 1e-6) {
    tc_state_.pressure_rate = std::clamp(
      1 - tc_state_.long_fx_input_latched / slip_control_inputs_.long_fx *
            (1 - tc_state_.pressure_rate),
      0.0, 1.0);
  }

  tc_state_.long_fx_input_latched = slip_control_inputs_.long_fx;
}
void TCControlledWheel::set_generic_inputs(const SlipControlInputs & inputs)
{
  slip_control_inputs_ = inputs;
}
void TCControlledWheel::set_current_timestamp(
  const std::chrono::steady_clock::time_point & timestamp)
{
  current_timestamp_ = timestamp;
}
double TCControlledWheel::get_target_brake_pressure() const
{
  return tc_state_.target_brake_pressure_output;
}
bool TCControlledWheel::get_is_active() const
{
  // Hold, Reduce, and Increase are active TC states.
  if (tc_state_.wheel_state >= Wheel_States::Reduce) {
    return true;
  }
  return false;
}
tam::pmg::MgmtInterface::SharedPtr TCControlledWheel::get_param_manager() const
{
  return tc_params_.get_param_manager();
}
tam::tsl::LoggerAccessInterface::SharedPtr TCControlledWheel::get_debug_out() const
{
  return logger_;
}
}  // namespace tam::control
