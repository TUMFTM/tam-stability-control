#pragma once
#include <algorithm>
#include <chrono>
#include <numeric>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tum_helpers_cpp/constants.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
/// @brief Vehicle wheel positions.
enum class Wheel_Position { Front_Left = 0, Front_Right = 1, Rear_Left = 2, Rear_Right = 3 };

/// @brief ABS and TC state-machine states.
enum class Wheel_States {
  Inactive = 0,  ///< Controller inactive
  Reduce = 1,    ///< Apply the reduce action
  Hold = 2,      ///< Hold the current control value
  Increase = 3,  ///< Apply the increase action
};

/// @brief ABS/TC distribution mode across wheels.
enum class OperationMode {
  individual,        ///< Each wheel controlled independently
  front_rear_split,  ///< Front and rear axles controlled together (min for ABS, max for TC)
  all_together,      ///< All wheels controlled together with most restrictive control
  rear_only          ///< Only the rear axle is controlled
};
/// @brief Inputs for one ABS/TC wheel controller.
struct WheelIndividualInputs
{
  bool allowed{};           ///< Control enabled for this wheel
  double slip{};            ///< Wheel slip
  double slip_angle{};      ///< Tire slip angle (rad)
  double slip_rate{};       ///< Rate of change of slip
  double brake_pressure{};  ///< Current brake pressure (bar)
  double brake_friction{};  ///< Brake friction coefficient
};
/// @brief Inputs shared by all slip controllers.
struct SlipControlInputs
{
  bool slip_valid{};                         ///< Slip estimate validity
  bool esc_active{};                         ///< ESC intervention active
  double long_fx{};                          ///< Longitudinal force (N)
  tam::types::control::Odometry odometry{};  ///< Vehicle motion data
};
/// @brief Slip-controller state, gear data, and outputs.
struct SlipControlState
{
  bool abs_latched{};           ///< Any wheel under ABS control
  bool tc_latched{};            ///< Any wheel under TC control
  int8_t gear{};                ///< Current gear feedback
  int8_t gear_request{};        ///< Requested gear
  double throttle_target_in{};  ///< Throttle input target
  double throttle_request{};    ///< Throttle output after TC cuts
  tam::types::common::DataPerWheel<double> brake_pressure_input_bar{};   ///< Brake pressure input
  tam::types::common::DataPerWheel<double> brake_pressure_target_bar{};  ///< Brake pressure output
  std::chrono::steady_clock::time_point time_gear_changed{};             ///< Last gear change time
  std::chrono::steady_clock::time_point
    time_gear_request_changed{};                    ///< Last gear request change time
  tam::types::common::DataPerWheel<double> slip{};  ///< Current slip per wheel
};
/// @brief Global ABS/TC parameters.
struct SlipControlParams
{
  bool abs_enabled{};                              ///< Enable ABS system
  bool abs_slip_vertical_load_weighted{};          ///< Weight slip by tire loads for ABS
  OperationMode abs_operation_mode{};              ///< How ABS controls wheels
  double abs_min_activation_velocity{};            ///< Minimum velocity for ABS (m/s)
  double abs_cooldown_gear_change{};               ///< Cooldown after gear change (s)
  double tc_cooldown_gear_change{};                ///< TC cooldown after gear change (s)
  bool tc_enabled{};                               ///< Enable TC system
  bool tc_slip_vertical_load_weighted{};           ///< Weight slip by tire loads for TC
  bool tc_use_virtual_slips{};                     ///< Use virtual slip estimates for TC
  bool abs_use_virtual_slips{};                    ///< Use virtual slip estimates for ABS
  OperationMode tc_operation_mode{};               ///< How TC controls wheels
  double tc_min_activation_velocity{};             ///< Minimum velocity for TC (m/s)
  double tc_max_slip_throttle_cut{};               ///< Max slip before throttle cut (%)
  double slip_angle_filter_pole{};                 ///< Low-pass filter pole for slip angle
  double tS{};                                     ///< Control loop sampling time (s)
  double duration_gearshift_initiation{};          ///< Time until pressure reduction starts (s)
  double duration_gearshift_pressure_reduction{};  ///< Duration of pressure reduction (s)
  double gearshift_pressure_reduction_factor{};    ///< Pressure reduction multiplier (0-1)
};
/// @brief ABS state for one wheel.
struct AbsState
{
  Wheel_States wheel_state{Wheel_States::Inactive};  ///< Current ABS state
  bool safety_feature_is_active{false};              ///< Safety limiter is active
  bool esp_trigger{};                                ///< ABS was triggered by ESC
  double pressure_rate{};                            ///< ABS control ratio
  double reduction_factor{};                         ///< Reduction factor based on vehicle dynamics
  double slip_threshold_reduction{};                 ///< Slip threshold scaling based on slip angle
  double long_fx_latched{};                          ///< Latched longitudinal force at activation
  double long_fx_reference{};                        ///< Reference longitudinal force
  double long_fx_output{};                           ///< Output longitudinal force
  double brake_pressure_latched{};                   ///< Latched brake pressure at activation
  double brake_pressure_output{};                    ///< Output brake pressure target
  double v_latched{};                                ///< Latched velocity at activation
  double slip_error{};                               ///< Normalized slip error (0-1)
  double current_brake_friction{0.3};                ///< Current brake friction coefficient
  tam::helpers::control::Cyclic_Vector<double>
    long_fx_input_vector;  ///< Moving average buffer for input force
  tam::helpers::control::Cyclic_Vector<double>
    long_fx_output_vector{};  ///< Moving average buffer for output force
};
/// @brief TC state for one wheel.
struct TcState
{
  Wheel_States wheel_state{Wheel_States::Inactive};  ///< Current TC state
  bool tc_allowed_{false};                           ///< TC allowed for this wheel
  double current_slip_{};                            ///< Filtered current wheel slip
  double pressure_rate{};                            ///< Retained drive-force fraction
  double long_fx_input_latched{};                    ///< Latched input longitudinal force
  double long_fx_output{};                           ///< Output longitudinal force
  double target_brake_pressure_latched{};            ///< Latched brake pressure target
  double target_brake_pressure_output{};             ///< Output brake pressure target
  double slip_threshold_reduction{};                 ///< Slip threshold scaling based on slip angle
  double slip_error{};                               ///< Normalized slip error (0-1)
  double slip_rate{};                                ///< Rate of change of slip
  tam::types::control::Odometry odometry{};          ///< Vehicle motion data
  std::chrono::time_point<std::chrono::steady_clock>
    last_time_threshold_exceeded{};  ///< Last time slip exceeded threshold
};
class TCParams
{
public:
  Wheel_Position wheel_position_{Wheel_Position::Front_Left};
  bool is_front_wheel{true};
  bool use_error_in_increase{false};
  double nominal_brake_bias_front{};
  std::vector<double> brake_bias_shift_interp_vel_mps{};
  std::vector<double> brake_bias_shift_values{};
  double r_BrakePadsMeanLeverFrRe_m__2{};
  double n_BrakePadsNumberFrRe__2{};
  double d_BrakeActuatorBoreFrRe_m__2{};
  double tyreradius_front{};
  double tyreradius_rear{};
  double reduce_initial_pressure_ratio{};
  double reduce_step{};
  double increase_step{};
  double slip_threshold_hold_reduce{};
  double slip_threshold_hold_increase{};
  double slip_threshold_reduce_hold{};
  double slip_threshold_increase_hold{};
  double slip_threshold_safe{};
  double fx_to_total_brake_pressure_factor{};
  double slip_threshold_reduction_factor{};
  double slip_angle_max{};
  double slip_filter_pole{};
  double deactivation_duration{};
  double tS{};
  std::size_t previous_param_state_hash = 0;
  TCParams(Wheel_Position wheel_position) : wheel_position_(wheel_position)
  {
    declare_and_update_parameters();
    double get_fx_to_brake_pressure_factor(double vel_mps);
  }
  void declare_and_update_parameters()
  {
    auto decl = [this](std::string name, double val) {
      return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    };
    auto decl_array = [this](std::string name, std::vector<double> val) {
      return param_manager_
        ->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE_ARRAY, "")
        .as_double_array();
    };
    tyreradius_front = decl("tires.front_left.radius.radius_20mps", 0.293475);
    tyreradius_rear = decl("tires.rear_left.radius.radius_20mps", 0.3074348);
    nominal_brake_bias_front = decl("P_VDC_brake_bias_front", 0.55);
    brake_bias_shift_interp_vel_mps = decl_array(
      "brake_bias_shift.vel_interp_mps", std::vector<double>{0.0});  // inactive by default
    brake_bias_shift_values =
      decl_array("brake_bias_shift.values", std::vector<double>{0.0});  // inactive by default
    r_BrakePadsMeanLeverFrRe_m__2 = decl("vehicle.brake.pad_mean_radius", 0.134);
    n_BrakePadsNumberFrRe__2 = decl("vehicle.brake.pad_number", 1.0);
    d_BrakeActuatorBoreFrRe_m__2 = decl("vehicle.brake.piston_diameter", 0.0798);
    slip_filter_pole = decl("SlipControl.TC.slip_filter_pole", 0.85);
    slip_threshold_reduction_factor = decl("SlipControl.TC.slip_threshold_reduction_factor", 0.5);
    slip_angle_max = decl("SlipControl.TC.slip_angle_max", 0.07);
    deactivation_duration = decl("SlipControl.TC.deactivation_duration", 2.0);
    tS = decl("tS", 0.01);
    use_error_in_increase =
      param_manager_
        ->declare_and_get_value(
          "SlipControl.TC.use_error_in_increase", false, tam::pmg::ParameterType::BOOL, "")
        .as_bool();
    previous_param_state_hash = param_manager_->get_state_hash();
    const auto calculate_fx_to_brake_pressure_factor = [this](const double tyre_radius) {
      const double brake_geometry_factor =
        2.0 * r_BrakePadsMeanLeverFrRe_m__2 * n_BrakePadsNumberFrRe__2 * M_PI *
        std::pow(d_BrakeActuatorBoreFrRe_m__2 / 2.0, 2) * tam::constants::pascal_per_bar;
      return tyre_radius / std::max(brake_geometry_factor, 1e-6);
    };
    if (
      (wheel_position_ == Wheel_Position::Front_Left) ||
      (wheel_position_ == Wheel_Position::Front_Right)) {
      reduce_initial_pressure_ratio =
        decl("SlipControl.TC.front.reduce_initial_pressure_ratio", 0.95);
      reduce_step = decl("SlipControl.TC.front.reduce_step", 0.2);
      increase_step = decl("SlipControl.TC.front.increase_step", 0.05);
      slip_threshold_reduce_hold = decl("SlipControl.TC.front.slip_threshold_reduce_hold", 12.0);
      slip_threshold_hold_increase = decl("SlipControl.TC.front.slip_threshold_hold_increase", 5.0);
      slip_threshold_hold_reduce = decl("SlipControl.TC.front.slip_threshold_hold_reduce", 10.0);
      slip_threshold_increase_hold = decl("SlipControl.TC.front.slip_threshold_increase_hold", 5.0);
      slip_threshold_safe = decl("SlipControl.TC.front.slip_threshold_safe", 7.0);
      fx_to_total_brake_pressure_factor =
        calculate_fx_to_brake_pressure_factor(tyreradius_front);
      is_front_wheel = true;
    } else if (
      (wheel_position_ == Wheel_Position::Rear_Left) ||
      (wheel_position_ == Wheel_Position::Rear_Right)) {
      reduce_initial_pressure_ratio =
        decl("SlipControl.TC.rear.reduce_initial_pressure_ratio", 0.95);
      reduce_step = decl("SlipControl.TC.rear.reduce_step", 0.1);
      increase_step = decl("SlipControl.TC.rear.increase_step", 0.05);
      slip_threshold_reduce_hold = decl("SlipControl.TC.rear.slip_threshold_reduce_hold", 12.0);
      slip_threshold_hold_increase = decl("SlipControl.TC.rear.slip_threshold_hold_increase", 5.0);
      slip_threshold_hold_reduce = decl("SlipControl.TC.rear.slip_threshold_hold_reduce", 10.0);
      slip_threshold_increase_hold = decl("SlipControl.TC.rear.slip_threshold_increase_hold", 5.0);
      slip_threshold_safe = decl("SlipControl.TC.rear.slip_threshold_safe", 7.0);
      fx_to_total_brake_pressure_factor =
        calculate_fx_to_brake_pressure_factor(tyreradius_rear);
      is_front_wheel = false;
    }
  }
  bool param_changed() { return param_manager_->get_state_hash() != previous_param_state_hash; }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const { return param_manager_; }
  double get_fx_to_brake_pressure_factor(const double vel_mps, const double brake_friction) const
  {
    // The interpolation helper clamps to the parameter range.
    const bool brake_bias_shift_valid =
      !brake_bias_shift_interp_vel_mps.empty() &&
      brake_bias_shift_interp_vel_mps.size() == brake_bias_shift_values.size();
    const double brake_bias_shift =
      brake_bias_shift_valid
        ? tam::helpers::numerical::interp(
            vel_mps, brake_bias_shift_interp_vel_mps, brake_bias_shift_values)
        : 0.0;
    const double current_brake_bias = std::clamp(
      nominal_brake_bias_front + brake_bias_shift, 0.0, 1.0);
    return is_front_wheel
             ? fx_to_total_brake_pressure_factor / brake_friction * current_brake_bias
             : fx_to_total_brake_pressure_factor / brake_friction * (1 - current_brake_bias);
  }

private:
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
};
class AbsParams
{
public:
  Wheel_Position wheel_position_{Wheel_Position::Front_Left};
  bool is_front_wheel{true};
  bool use_error_in_increase{false};
  int MovingAverageFxWindowLength{};
  double nominal_brake_bias_front{};
  std::vector<double> brake_bias_shift_interp_vel_mps{};
  std::vector<double> brake_bias_shift_values{};
  double r_BrakePadsMeanLeverFrRe_m__2{};
  double n_BrakePadsNumberFrRe__2{};
  double d_BrakeActuatorBoreFrRe_m__2{};
  double reduce_initial_pressure_ratio{};
  double increase_initial_pressure_ratio{};
  double reduce_step{};
  double increase_step{};
  double increase_step_target_increase{};
  double slip_threshold_hold_increase{};
  double slip_threshold_hold_reduce{};
  double slip_threshold_increase_hold{};
  double slip_threshold_reduce_hold{};
  double slip_threshold_safe{};
  double fx_safety_threshold_factor_{};
  double fx_to_total_brake_pressure_factor{};
  double tyreradius_front{};
  double tyreradius_rear{};
  double min_long_force{};
  double air_density{};
  double cross_track_area{};
  double lift_coeff{};
  double mass{};
  double slip_angle_max{};
  double slip_threshold_reduction_factor{};
  double tS{};
  std::size_t previous_param_state_hash = 0;
  AbsParams(Wheel_Position wheel_position) : wheel_position_(wheel_position)
  {
    declare_and_update_parameters();
    double get_fx_to_brake_pressure_factor(double vel_mps);
  }
  void declare_and_update_parameters()
  {
    auto decl = [this](std::string name, double val) {
      return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    };
    auto decl_array = [this](std::string name, std::vector<double> val) {
      return param_manager_
        ->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE_ARRAY, "")
        .as_double_array();
    };
    use_error_in_increase =
      param_manager_
        ->declare_and_get_value(
          "SlipControl.ABS.use_error_in_increase", false, tam::pmg::ParameterType::BOOL, "")
        .as_bool();
    tyreradius_front = decl("tires.front_left.radius.radius_20mps", 0.293475);
    tyreradius_rear = decl("tires.rear_left.radius.radius_20mps", 0.3074348);
    nominal_brake_bias_front = decl("P_VDC_brake_bias_front", 0.55);
    brake_bias_shift_interp_vel_mps = decl_array(
      "brake_bias_shift.vel_interp_mps", std::vector<double>{0.0});  // inactive by default
    brake_bias_shift_values =
      decl_array("brake_bias_shift.values", std::vector<double>{0.0});  // inactive by default
    r_BrakePadsMeanLeverFrRe_m__2 = decl("vehicle.brake.pad_mean_radius", 0.134);
    n_BrakePadsNumberFrRe__2 = decl("vehicle.brake.pad_number", 1.0);
    d_BrakeActuatorBoreFrRe_m__2 = decl("vehicle.brake.piston_diameter", 0.0798);
    air_density = decl("vehicle.aero.air_density", 1.22);
    cross_track_area = decl("vehicle.aero.cross_track_area", 1.0);
    lift_coeff = decl("vehicle.aero.lift_coeff", -1.65);
    mass = decl("vehicle.mass.total", 800.0);
    min_long_force = decl("SlipControl.ABS.min_long_force", -4000.0);
    slip_angle_max = decl("SlipControl.ABS.slip_angle_max", 0.07);
    slip_threshold_reduction_factor = decl("SlipControl.ABS.slip_threshold_reduction_factor", 0.3);
    tS = decl("tS", 0.01);
    MovingAverageFxWindowLength = static_cast<int>(std::max<int64_t>(
      1, param_manager_
           ->declare_and_get_value(
             "SlipControl.ABS.moving_average_fx_window_length", 50,
             tam::pmg::ParameterType::INTEGER, "")
           .as_int()));
    previous_param_state_hash = param_manager_->get_state_hash();
    const auto calculate_fx_to_brake_pressure_factor = [this](const double tyre_radius) {
      const double brake_geometry_factor =
        2.0 * r_BrakePadsMeanLeverFrRe_m__2 * n_BrakePadsNumberFrRe__2 * M_PI *
        std::pow(d_BrakeActuatorBoreFrRe_m__2 / 2.0, 2) * tam::constants::pascal_per_bar;
      return tyre_radius / std::max(brake_geometry_factor, 1e-6);
    };
    if (
      (wheel_position_ == Wheel_Position::Front_Left) ||
      (wheel_position_ == Wheel_Position::Front_Right)) {
      reduce_initial_pressure_ratio =
        decl("SlipControl.ABS.front.reduce_initial_pressure_ratio", 0.6);
      increase_initial_pressure_ratio =
        decl("SlipControl.ABS.front.increase_initial_pressure_ratio", 0.2);
      reduce_step = decl("SlipControl.ABS.front.reduce_step", 0.1);
      increase_step = decl("SlipControl.ABS.front.increase_step", 0.25);
      increase_step_target_increase =
        decl("SlipControl.ABS.front.increase_step_target_increase", 0.1);
      slip_threshold_hold_increase =
        decl("SlipControl.ABS.front.slip_threshold_hold_increase", 0.5);
      slip_threshold_hold_reduce = decl("SlipControl.ABS.front.slip_threshold_hold_reduce", 0.5);
      slip_threshold_increase_hold =
        decl("SlipControl.ABS.front.slip_threshold_increase_hold", 0.5);
      slip_threshold_safe = decl("SlipControl.ABS.front.slip_threshold_safe", 0.5);
      slip_threshold_reduce_hold = decl("SlipControl.ABS.front.slip_threshold_reduce_hold", 0.5);
      fx_safety_threshold_factor_ = decl("SlipControl.ABS.front.fx_safety_threshold_factor", 0.7);
      fx_to_total_brake_pressure_factor =
        calculate_fx_to_brake_pressure_factor(tyreradius_front);
      is_front_wheel = true;
    } else if (
      (wheel_position_ == Wheel_Position::Rear_Left) ||
      (wheel_position_ == Wheel_Position::Rear_Right)) {
      reduce_initial_pressure_ratio =
        decl("SlipControl.ABS.rear.reduce_initial_pressure_ratio", 0.6);
      increase_initial_pressure_ratio =
        decl("SlipControl.ABS.rear.increase_initial_pressure_ratio", 0.2);
      reduce_step = decl("SlipControl.ABS.rear.reduce_step", 0.1);
      increase_step = decl("SlipControl.ABS.rear.increase_step", 0.25);
      increase_step_target_increase =
        decl("SlipControl.ABS.rear.increase_step_target_increase", 0.1);
      slip_threshold_hold_increase = decl("SlipControl.ABS.rear.slip_threshold_hold_increase", 0.5);
      slip_threshold_hold_reduce = decl("SlipControl.ABS.rear.slip_threshold_hold_reduce", 0.5);
      slip_threshold_increase_hold = decl("SlipControl.ABS.rear.slip_threshold_increase_hold", 0.5);
      slip_threshold_safe = decl("SlipControl.ABS.rear.slip_threshold_safe", 0.5);
      slip_threshold_reduce_hold = decl("SlipControl.ABS.rear.slip_threshold_reduce_hold", 0.5);
      fx_safety_threshold_factor_ = decl("SlipControl.ABS.rear.fx_safety_threshold_factor", 0.7);
      fx_to_total_brake_pressure_factor =
        calculate_fx_to_brake_pressure_factor(tyreradius_rear);
      is_front_wheel = false;
    }
  }
  bool param_changed() { return param_manager_->get_state_hash() != previous_param_state_hash; }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const { return param_manager_; }
  double get_fx_to_brake_pressure_factor(const double vel_mps, const double brake_friction) const
  {
    // The interpolation helper clamps to the parameter range.
    const bool brake_bias_shift_valid =
      !brake_bias_shift_interp_vel_mps.empty() &&
      brake_bias_shift_interp_vel_mps.size() == brake_bias_shift_values.size();
    const double brake_bias_shift =
      brake_bias_shift_valid
        ? tam::helpers::numerical::interp(
            vel_mps, brake_bias_shift_interp_vel_mps, brake_bias_shift_values)
        : 0.0;
    const double current_brake_bias = std::clamp(
      nominal_brake_bias_front + brake_bias_shift, 0.0, 1.0);
    return is_front_wheel
             ? fx_to_total_brake_pressure_factor / brake_friction * current_brake_bias
             : fx_to_total_brake_pressure_factor / brake_friction * (1 - current_brake_bias);
  }

private:
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
};
}  // namespace tam::control
