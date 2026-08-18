#pragma once
#include <algorithm>
#include <map>
#include <memory>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_helpers_cpp/constants.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
/// @class ESC
/// @brief Electronic stability control for yaw rate and sideslip.
class ESC
{
private:
  // Parameters
  struct
  {
    bool esc_enabled;                          ///< ESC system enable flag
    double min_activation_velocity;            ///< Minimum velocity for ESC activation
    std::vector<double> threshold_velocities;  ///< Velocity breakpoints for threshold interpolation
    std::vector<double> beta_threshold_activate;  ///< Sideslip angle activation thresholds
    std::vector<double>
      beta_error_threshold_activate;  ///< Sideslip angle error activation thresholds
    std::vector<double> psi_dot_error_threshold_activate;  ///< Yaw rate error activation thresholds
    double beta_threshold_deactivate;           ///< Sideslip angle deactivation threshold
    double beta_error_threshold_deactivate;     ///< Sideslip error deactivation threshold
    double psi_dot_error_threshold_deactivate;  ///< Yaw rate error deactivation threshold
    double kp_psi_dot, ki_psi_dot, kd_psi_dot;  ///< Yaw rate PID gains
    double kp_beta, ki_beta, kd_beta;           ///< Sideslip angle PID gains
    double tS;                                  ///< Control loop sampling time
    double max_integrator_yaw_moment;           ///< Integrator anti-windup for yaw moment
    double max_brake_pressure;                  ///< Maximum achievable brake pressure
    double wheelbase, trackwidth_front;         ///< Vehicle geometric parameters
    double mass, yaw_inertia;                   ///< Vehicle mass and inertia
    double lf;                                  ///< Distance to front axle
    double cf, cr;                              ///< Tire stiffness front/rear
    double tire_radius_front;                   ///< Front tire radius
    double brake_pad_mean_radius;               ///< Brake system geometry
    double
      force_to_frictionless_brake_pressure_bar_per_N;  ///< Conversion factor for force to pressure
    double brake_pads_number, brake_piston_diameter;   ///< Brake system parameters
    double tS_psi_dot, tS_beta;                        ///< Filter time constants
  } p_;

  using Dpw = tam::types::common::DataPerWheel<double>;

  // State
  bool esc_active_{false};           ///< ESC currently active flag
  bool esc_enabled_{false};          ///< ESC enabled for this cycle
  double beta_{};                    ///< Current vehicle sideslip angle (rad)
  double steering_target_{};         ///< Target steering angle input
  double beta_error_filtered_{};     ///< Filtered sideslip error
  double psi_dot_{};                 ///< Current vehicle yaw rate (rad/s)
  double velocity_{};                ///< Current vehicle longitudinal velocity
  double psi_dot_error_filtered_{};  ///< Filtered yaw rate error
  Dpw brake_pressure_target_bar_{};  ///< ESC output: target brake pressure per wheel
  Dpw brake_pressure_input_bar_{};   ///< Input: brake pressure from motion planner
  Dpw brake_friction_{};             ///< Friction coefficient per wheel

  tam::types::control::AdditionalEspTargets
    additional_esc_targets_{};  ///< Optional additional control targets

  // Controllers
  tam::helpers::control::PIDControl<double> pid_psi_dot_{};  ///< Yaw rate tracking PID
  tam::helpers::control::PIDControl<double> pid_beta_{};     ///< Sideslip angle tracking PID

  // Filters
  tam::helpers::control::FirstOrderLowPass<double> filter_psi_dot_{0.0, 0.9};  ///< Yaw rate filter
  tam::helpers::control::FirstOrderLowPass<double> filter_beta_{0.0, 0.7};     ///< Sideslip filter

  // Support services
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash = 0;
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  // Control logic
  void declare_and_update_parameters();            ///< Load parameters from configuration
  std::pair<double, double> calculate_references(  ///< Calculate target yaw rate and sideslip
    const tam::types::control::AdditionalEspTargets & additional_esc_targets,
    const double steering_target, const double velocity) const;
  bool has_valid_activation_thresholds() const;
  bool deactivate_esc() const;  ///< Check if ESC should deactivate
  bool activate_esc() const;    ///< Check if ESC should activate

public:
  ESC();
  void step();  ///< Execute ESC control cycle

  // Inputs
  void set_target_brake_pressure(const Dpw & target_brake_pressure);  ///< Input from motion planner
  void set_feedback_odometry(
    const tam::types::control::Odometry & odometry);  ///< Vehicle motion feedback
  void set_steering_target_rad(const double steering_target) { steering_target_ = steering_target; }
  void set_brake_friction_coefficients(
    const tam::types::common::DataPerWheel<double> & brake_friction);
  void set_additional_esc_targets(
    const tam::types::control::AdditionalEspTargets & additional_esc_targets);
  void set_slip_angle_valid(const bool slip_angle_valid);

  // Outputs
  bool get_esc_active() const;  ///< Check if ESC is currently active
  tam::types::common::DataPerWheel<double> get_brake_pressure_target_bar()
    const;                                                           ///< Get output brake pressures
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const;      ///< Access parameter manager
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const;  ///< Access debug logger
};
}  // namespace tam::control
