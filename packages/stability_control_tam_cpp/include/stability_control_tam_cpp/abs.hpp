// Copyright 2025 Phillip Pitschi
#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "param_management_cpp/base.hpp"
#include "stability_control_tam_cpp/types.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
/// @class ABSControlledWheel
/// @brief Per-wheel anti-lock braking controller.
class ABSControlledWheel
{
private:
  // Inputs and state
  WheelIndividualInputs abs_inputs_{};       ///< Per-wheel ABS inputs (slip, pressure, etc.)
  SlipControlInputs slip_control_inputs_{};  ///< Global slip control inputs
  AbsState abs_state_{};                     ///< ABS state machine and control variables

  AbsParams params_;  ///< Per-wheel ABS parameters

  // Logger
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  // Control logic
  bool activate() const;                 ///< Check if ABS should activate
  bool deactivate() const;               ///< Check if ABS should deactivate
  void log_debug_values();               ///< Log all ABS control variables
  void update_latched_brake_pressure();  ///< Adjust latched pressure for friction changes
  void reduce_pressure_ratio(const double reduction_factor);    ///< Reduce brake pressure
  void increase_pressure_ratio(const double reduction_factor);  ///< Increase brake pressure
  Wheel_States transitions(const Wheel_States & state);         ///< State machine transition logic
  void update_safety_feature();            ///< Apply safety feature to limit output
  void calculate_reduction_factor();       ///< Calculate pressure reduction factor
  void update_slip_threshold_reduction();  ///< Adjust slip thresholds based on slip angle
  void convert_fx_to_brake_pressure();     ///< Convert longitudinal force to brake pressure
  void calculate_slip_error();             ///< Calculate normalized slip error for control

public:
  explicit ABSControlledWheel(const Wheel_Position & wheel_position) : params_(wheel_position)
  {
    abs_state_.long_fx_input_vector.resize(params_.MovingAverageFxWindowLength);
    abs_state_.long_fx_output_vector.resize(params_.MovingAverageFxWindowLength);
  }
  void step();  ///< Execute ABS control cycle

  // Inputs
  void set_wheel_individual_inputs(const WheelIndividualInputs & abs_inputs);
  void set_generic_inputs(const SlipControlInputs & generic_inputs);

  // Outputs
  double get_target_brake_pressure() const;  ///< Get calculated brake pressure target
  bool get_is_active() const;                ///< Check if ABS is actively controlling
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const;      ///< Access parameter manager
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const;  ///< Access debug logger
};
}  // namespace tam::control
