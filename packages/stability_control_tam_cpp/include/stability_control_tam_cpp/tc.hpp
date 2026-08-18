// Copyright 2025 Phillip Pitschi
#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/base.hpp"
#include "stability_control_tam_cpp/types.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
/// @class TCControlledWheel
/// @brief Per-wheel traction controller for acceleration.
class TCControlledWheel
{
private:
  // Parameters and state
  TCParams tc_params_;  ///< TC parameters for this wheel
  TcState tc_state_{};  ///< TC state machine and control variables

  // Inputs
  WheelIndividualInputs tc_inputs_{};        ///< Per-wheel TC inputs
  SlipControlInputs slip_control_inputs_{};  ///< Global slip control inputs

  // Slip filtering
  tam::helpers::control::FirstOrderLowPass<double>
    slip_filter_{};  ///< Low-pass filter for wheel slip

  // State timing
  std::chrono::steady_clock::time_point current_timestamp_{};

  // Logger
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  // Control logic
  bool activate() const;                                 ///< Check if TC should activate
  bool deactivate() const;                               ///< Check if TC should deactivate
  void log_debug_values();                               ///< Log all TC control variables
  void reduce_target();                                  ///< Reduce retained drive force
  void increase_target();                                ///< Increase retained drive force
  Wheel_States transitions(const Wheel_States & state);  ///< State machine transition logic
  void update_slip_threshold_reduction();  ///< Adjust slip thresholds based on slip angle
  void calculate_slip_error();             ///< Calculate normalized slip error for control
  void convert_fx_to_brake_pressure();     ///< Convert longitudinal force to brake pressure

public:
  explicit TCControlledWheel(const Wheel_Position & wheel_position) : tc_params_(wheel_position) {}
  void step();  ///< Execute TC control cycle

  // Inputs
  void set_wheel_individual_inputs(const WheelIndividualInputs & inputs);
  void set_generic_inputs(const SlipControlInputs & inputs);
  void set_current_timestamp(const std::chrono::steady_clock::time_point & timestamp);

  // Outputs
  double get_target_brake_pressure() const;  ///< Get calculated brake pressure target
  bool get_is_active() const;                ///< Check if TC is actively controlling
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const;      ///< Access parameter manager
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const;  ///< Access debug logger
};
}  // namespace tam::control
