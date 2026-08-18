// Copyright 2025 Phillip Pitschi
#pragma once
#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "stability_control_tam_cpp/abs.hpp"
#include "stability_control_tam_cpp/tc.hpp"
#include "stability_control_tam_cpp/types.hpp"
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
/// @class SlipController
/// @brief Coordinates per-wheel ABS and traction control.
class SlipController
{
private:
  // Type aliases
  using Dpw = tam::types::common::DataPerWheel<double>;  ///< Data per wheel (double)
  using Bpw = tam::types::common::DataPerWheel<bool>;    ///< Data per wheel (bool)
  template <typename T>
  using Cyclic_Vector = tam::helpers::control::Cyclic_Vector<T>;  ///< Fixed-size circular buffer
  template <typename T>
  using Flank_Detector =
    tam::helpers::control::Flank_Detector<T>;  ///< Rising/falling edge detector

  // Gear-change detectors
  Flank_Detector<int8_t> gear_change_detector =
    Flank_Detector<int8_t>();  ///< Detects actual gear changes
  Flank_Detector<int8_t> gear_request_change_detector =
    Flank_Detector<int8_t>();  ///< Detects gear shift requests

  // State and parameters
  SlipControlInputs
    slip_control_inputs{};  ///< Global inputs (slip valid, ESC active, forces, odometry)
  SlipControlState slip_control_state{};    ///< Internal state (gear, outputs, timing)
  SlipControlParams slip_control_params{};  ///< Configuration parameters

  // Slip-angle filter
  tam::helpers::control::FirstOrderLowPass<Dpw> slip_angle_filter{
    Dpw{0.0}, Dpw{0.7}};  ///< Slip angle LPF

  // Per-wheel ABS controllers
  tam::types::common::DataPerWheel<ABSControlledWheel> abs_controlled_wheels{
    ABSControlledWheel{Wheel_Position::Front_Left}, ABSControlledWheel{Wheel_Position::Front_Right},
    ABSControlledWheel{Wheel_Position::Rear_Left}, ABSControlledWheel{Wheel_Position::Rear_Right}};

  // Per-wheel TC controllers
  tam::types::common::DataPerWheel<TCControlledWheel> tc_controlled_wheels{
    TCControlledWheel{Wheel_Position::Front_Left}, TCControlledWheel{Wheel_Position::Front_Right},
    TCControlledWheel{Wheel_Position::Rear_Left}, TCControlledWheel{Wheel_Position::Rear_Right}};

  // Per-wheel inputs
  tam::types::common::DataPerWheel<WheelIndividualInputs> abs_inputs;  ///< ABS inputs
  tam::types::common::DataPerWheel<WheelIndividualInputs> tc_inputs;   ///< TC inputs

  // Per-wheel brake friction
  tam::types::common::DataPerWheel<double> brake_friction_{};

  // Event timing
  std::chrono::steady_clock::time_point current_timestamp_{};  ///< Current system time

  // Parameters
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer_ =
    std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<tam::pmg::MgmtInterface::SharedPtr>{
        abs_controlled_wheels[0].get_param_manager(), abs_controlled_wheels[1].get_param_manager(),
        abs_controlled_wheels[2].get_param_manager(), abs_controlled_wheels[3].get_param_manager(),
        tc_controlled_wheels[0].get_param_manager(), tc_controlled_wheels[1].get_param_manager(),
        tc_controlled_wheels[2].get_param_manager(), tc_controlled_wheels[3].get_param_manager(),
        param_manager_});
  std::size_t previous_param_state_hash = 0;

  // Logging
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  tam::tsl::LoggerComposer::SharedPtr logger_composer_ = std::make_shared<tam::tsl::LoggerComposer>(
    std::vector<std::pair<std::string, tam::tsl::LoggerAccessInterface::SharedPtr>>{
      std::make_pair("", logger_),  ///< Top-level logs (e.g., system status)
      std::make_pair("ABS/front_left/", abs_controlled_wheels.front_left.get_debug_out()),
      std::make_pair("ABS/front_right/", abs_controlled_wheels.front_right.get_debug_out()),
      std::make_pair("ABS/rear_left/", abs_controlled_wheels.rear_left.get_debug_out()),
      std::make_pair("ABS/rear_right/", abs_controlled_wheels.rear_right.get_debug_out()),
      std::make_pair("TC/front_left/", tc_controlled_wheels.front_left.get_debug_out()),
      std::make_pair("TC/front_right/", tc_controlled_wheels.front_right.get_debug_out()),
      std::make_pair("TC/rear_left/", tc_controlled_wheels.rear_left.get_debug_out()),
      std::make_pair("TC/rear_right/", tc_controlled_wheels.rear_right.get_debug_out())});

  // Control logic
  void declare_and_update_parameters();  ///< Load configuration parameters
  void check_allowed();                  ///< Determine if ABS/TC allowed based on conditions
  Dpw shift_pressure_reduction(
    const Dpw & brake_pressure_target) const;  ///< Apply gearshift pressure reduction
  void distribute_slips(  ///< Distribute individual wheel slips to ABS/TC based on mode
    Dpw & slip, Dpw & slip_angle, Dpw & slip_rate,
    tam::types::common::DataPerWheel<WheelIndividualInputs> & input_abstc,
    OperationMode operation_mode, std::function<double(const double, const double)> minmax);

public:
  SlipController();

  // Control loop
  void step();  ///< Execute one control cycle for all ABS/TC controllers

  // Inputs
  void set_slips(
    const Dpw & slip, const Dpw & virtual_slip, const Dpw & vertical_load,
    const Dpw & slip_angle);                   ///< Set wheel slip measurements and tire loads
  void set_slip_valid(const bool slip_valid);  ///< Set validity flag for slip data
  void set_long_fx(const double long_fx);      ///< Set longitudinal force (N)
  void set_target_brake_pressure(
    Dpw target_brake_pressure);  ///< Set target brake pressure from motion planner
  void set_brake_friction_coefficients(
    const Dpw & brake_friction);                           ///< Set friction coefficients per wheel
  void set_throttle_target(const double throttle_target);  ///< Set target throttle command
  void set_gear_request(const int8_t gear);                ///< Set requested gear
  void set_feedback_gear(const int8_t gear);               ///< Set current gear feedback
  void set_feedback_odometry(
    const tam::types::control::Odometry & odometry);  ///< Set vehicle motion data
  void set_esc_active(const bool esc_active);         ///< Set ESC active status
  void set_current_timestamp(
    const std::chrono::steady_clock::time_point & timestamp);  ///< Set current time

  // Outputs
  Dpw get_brake_pressure_target_bar() const;  ///< Get target brake pressure per wheel (bar)
  double get_throttle_request() const;        ///< Get throttle command after TC throttle cuts
  bool get_status() const;                    ///< True if ABS or TC is active
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const;  ///< Access debug logs
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const;      ///< Access parameter manager
};
}  // namespace tam::control
