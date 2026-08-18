// Copyright 2026 Phillip Pitschi

#include "stability_control_tam_cpp/esc.hpp"

#include <algorithm>
#include <cmath>
namespace tam::control
{
ESC::ESC() { declare_and_update_parameters(); }
void ESC::declare_and_update_parameters()
{
  auto decl_double = [this](std::string name, double val) {
    return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  };

  p_.esc_enabled =
    param_manager_->declare_and_get_value("ESC.enabled", true, tam::pmg::ParameterType::BOOL, "")
      .as_bool();

  p_.min_activation_velocity = decl_double("ESC.min_activation_velocity", 10.0);
  p_.beta_threshold_deactivate = decl_double("ESC.beta_threshold_deactivate", 0.0524);
  p_.beta_error_threshold_deactivate = decl_double("ESC.beta_error_threshold_deactivate", 0.0873);
  p_.psi_dot_error_threshold_deactivate =
    decl_double("ESC.psi_dot_error_threshold_deactivate", 0.1745);

  p_.tS_psi_dot = std::clamp(decl_double("ESC.tS_psi_dot", 0.9), 0.0, 1.0);
  p_.tS_beta = std::clamp(decl_double("ESC.tS_beta", 0.7), 0.0, 1.0);

  p_.kp_psi_dot = decl_double("ESC.kp_psi_dot", 50.0);
  p_.ki_psi_dot = decl_double("ESC.ki_psi_dot", 0.0);
  p_.kd_psi_dot = decl_double("ESC.kd_psi_dot", 0.0);

  p_.kp_beta = decl_double("ESC.kp_beta", 50.0);
  p_.ki_beta = decl_double("ESC.ki_beta", 0.0);
  p_.kd_beta = decl_double("ESC.kd_beta", 0.0);

  p_.tS = std::max(decl_double("tS", 0.01), 1e-6);

  p_.max_integrator_yaw_moment = decl_double("ESC.max_integrator_yaw_moment", 1325.0);
  p_.max_brake_pressure = std::max(decl_double("ESC.max_brake_pressure", 80.0), 0.0);

  p_.wheelbase = decl_double("vehicle.dimension.wheelbase", 2.971);
  p_.trackwidth_front = decl_double("vehicle.dimension.trackwidth", 1.606);

  p_.mass = decl_double("vehicle.mass.total", 800.0);
  p_.yaw_inertia = decl_double("vehicle.inertia.yaw", 1000.0);
  p_.lf = decl_double("vehicle.dimension.distance_to_front_axle", 1.724);

  p_.cf = decl_double("tire_stiffness_front", 100000.0);
  p_.cr = decl_double("tire_stiffness_rear", 200000.0);

  p_.tire_radius_front = decl_double("tires.front_left.radius.radius_20mps", 0.293475);
  p_.brake_pad_mean_radius = decl_double("vehicle.brake.pad_mean_radius", 0.134);
  p_.brake_pads_number = decl_double("vehicle.brake.pad_number", 1.0);
  p_.brake_piston_diameter = decl_double("vehicle.brake.piston_diameter", 0.0798);

  p_.threshold_velocities = param_manager_
                              ->declare_and_get_value(
                                "ESC.threshold_velocities", std::vector<double>{0.0, 0.0},
                                tam::pmg::ParameterType::DOUBLE_ARRAY, "")
                              .as_double_array();
  p_.beta_threshold_activate = param_manager_
                                 ->declare_and_get_value(
                                   "ESC.beta_threshold_activate", std::vector<double>{0.0, 0.0},
                                   tam::pmg::ParameterType::DOUBLE_ARRAY, "")
                                 .as_double_array();
  p_.beta_error_threshold_activate =
    param_manager_
      ->declare_and_get_value(
        "ESC.beta_error_threshold_activate", std::vector<double>{0.0, 0.0},
        tam::pmg::ParameterType::DOUBLE_ARRAY, "")
      .as_double_array();
  p_.psi_dot_error_threshold_activate =
    param_manager_
      ->declare_and_get_value(
        "ESC.psi_dot_error_threshold_activate", std::vector<double>{0.0, 0.0},
        tam::pmg::ParameterType::DOUBLE_ARRAY, "")
      .as_double_array();

  // Convert front-tire force to brake pressure.
  p_.force_to_frictionless_brake_pressure_bar_per_N =
    p_.tire_radius_front /
    (p_.brake_pad_mean_radius * p_.brake_pads_number * M_PI *
     std::pow(p_.brake_piston_diameter / 2.0, 2) * tam::constants::pascal_per_bar);
  previous_param_state_hash = param_manager_->get_state_hash();

  pid_psi_dot_.set_params(
    0.0, p_.kp_psi_dot, p_.ki_psi_dot, p_.kd_psi_dot, p_.tS, -p_.max_integrator_yaw_moment,
    p_.max_integrator_yaw_moment);
  pid_beta_.set_params(
    0.0, p_.kp_beta, p_.ki_beta, p_.kd_beta, p_.tS, -p_.max_integrator_yaw_moment,
    p_.max_integrator_yaw_moment);

  filter_psi_dot_.set_tf_pole(p_.tS_psi_dot);
  filter_beta_.set_tf_pole(p_.tS_beta);
}
std::pair<double, double> ESC::calculate_references(
  const tam::types::control::AdditionalEspTargets & additional_esc_targets,
  const double steering_target, const double velocity) const
{
  double psi_dot_target{}, beta_target{}, kappa_target{};

  // Derive missing references from steering or lateral-acceleration demand.
  if (
    !additional_esc_targets.yaw_rate_request_radps.has_value() ||
    !additional_esc_targets.slip_angle_request_rad.has_value()) {
        if (additional_esc_targets.ay_request_mps2.has_value()) {
      kappa_target =
        additional_esc_targets.ay_request_mps2.value() / std::pow(std::max(velocity, 3.0), 2);
    } else {
      kappa_target =
        steering_target / (p_.wheelbase + (p_.mass * std::pow(velocity, 2) *
                                           ((p_.wheelbase - p_.lf) * p_.cr - p_.lf * p_.cf) /
                                           (p_.cf * p_.cr * p_.wheelbase)));
    }
  }

  if (additional_esc_targets.yaw_rate_request_radps.has_value()) {
    psi_dot_target = additional_esc_targets.yaw_rate_request_radps.value();
  } else {
    psi_dot_target = kappa_target * velocity;
  }

  if (additional_esc_targets.slip_angle_request_rad.has_value()) {
    beta_target = additional_esc_targets.slip_angle_request_rad.value();
  } else {
    beta_target =
      kappa_target *
      ((p_.wheelbase - p_.lf) - (p_.lf * p_.mass * std::pow(velocity, 2)) / (p_.cr * p_.wheelbase));
  }

  return {psi_dot_target, beta_target};
}
bool ESC::has_valid_activation_thresholds() const
{
  const size_t size = p_.threshold_velocities.size();
  return size > 0 && p_.beta_threshold_activate.size() == size &&
         p_.beta_error_threshold_activate.size() == size &&
         p_.psi_dot_error_threshold_activate.size() == size;
}
bool ESC::deactivate_esc() const
{
  // Deactivate below the speed limit or once all errors are small.
  return !esc_enabled_ || velocity_ < p_.min_activation_velocity ||
         ((std::abs(beta_) < p_.beta_threshold_deactivate) &&
          (std::abs(beta_error_filtered_) < p_.beta_error_threshold_deactivate) &&
          (std::abs(psi_dot_error_filtered_) < p_.psi_dot_error_threshold_deactivate));
}
bool ESC::activate_esc() const
{
  if (!has_valid_activation_thresholds()) {
    return false;
  }

  double beta_threshold_activate_ =
    tam::helpers::numerical::interp(velocity_, p_.threshold_velocities, p_.beta_threshold_activate);
  double beta_error_threshold_activate_ = tam::helpers::numerical::interp(
    velocity_, p_.threshold_velocities, p_.beta_error_threshold_activate);
  double psi_dot_error_threshold_activate_ = tam::helpers::numerical::interp(
    velocity_, p_.threshold_velocities, p_.psi_dot_error_threshold_activate);

  logger_->log("beta_threshold_activate", beta_threshold_activate_);
  logger_->log("beta_error_threshold_activate", beta_error_threshold_activate_);
  logger_->log("psi_dot_error_threshold_activate", psi_dot_error_threshold_activate_);

  // Require large, directionally consistent yaw-rate and sideslip errors.
  return esc_enabled_ && velocity_ > p_.min_activation_velocity &&
         std::abs(beta_) > beta_threshold_activate_ &&
         std::abs(beta_error_filtered_) > beta_error_threshold_activate_ &&
         std::abs(psi_dot_error_filtered_) > psi_dot_error_threshold_activate_ &&
         psi_dot_ * psi_dot_error_filtered_ < 0.0 && beta_ * beta_error_filtered_ < 0.0 &&
         beta_error_filtered_ * psi_dot_error_filtered_ < 0.0;
}
void ESC::step()
{
  if (param_manager_->get_state_hash() != previous_param_state_hash) {
    declare_and_update_parameters();
  }

  // Start from the requested brake pressures.
  brake_pressure_target_bar_ = brake_pressure_input_bar_;

  auto [psi_dot_target, beta_target] =
    calculate_references(additional_esc_targets_, steering_target_, velocity_);

  double psi_dot_error = psi_dot_target - psi_dot_;
  double beta_error = beta_target - beta_;

  psi_dot_error_filtered_ = filter_psi_dot_.step(psi_dot_error);
  beta_error_filtered_ = filter_beta_.step(beta_error);

  // Apply activation hysteresis.
  if (!has_valid_activation_thresholds()) {
    esc_active_ = false;
  } else if (esc_active_) {
    esc_active_ = !deactivate_esc();
  } else {
    esc_active_ = activate_esc();
  }

  tam::helpers::control::PIDFeedback<double> pid_feedback_psi_dot{}, pid_feedback_beta;
  pid_feedback_psi_dot = pid_psi_dot_.step(
    psi_dot_error_filtered_, true, esc_active_, esc_active_, !esc_active_, esc_active_);
  pid_feedback_beta = pid_beta_.step(
    -beta_error_filtered_, true, esc_active_, esc_active_, !esc_active_, esc_active_);

  // Convert yaw-moment demand to differential front braking.
  double yaw_moment{};
  if (esc_active_) {
    yaw_moment = p_.yaw_inertia * (pid_feedback_psi_dot.feedback + pid_feedback_beta.feedback);

    double additional_brake_pressure =
      yaw_moment * 2.0 / p_.trackwidth_front * p_.force_to_frictionless_brake_pressure_bar_per_N;

    if (additional_brake_pressure > 0.0) {
      // Shift pressure from right to left.
      double additional_pressure_right = std::min(
        brake_pressure_target_bar_.front_right,
        additional_brake_pressure / brake_friction_.front_right / 2.0);
      brake_pressure_target_bar_.front_right -= additional_pressure_right;

      const double left_pressure_headroom = std::max(
        0.0, p_.max_brake_pressure - brake_pressure_target_bar_.front_left);
      brake_pressure_target_bar_.front_left += std::min(
        (additional_brake_pressure - additional_pressure_right * brake_friction_.front_right) /
          brake_friction_.front_left,
        left_pressure_headroom);
    } else {
      // Shift pressure from left to right.
      double additional_pressure_left = std::min(
        brake_pressure_target_bar_.front_left,
        std::abs(additional_brake_pressure / brake_friction_.front_left) / 2.0);
      brake_pressure_target_bar_.front_left -= additional_pressure_left;

      const double right_pressure_headroom = std::max(
        0.0, p_.max_brake_pressure - brake_pressure_target_bar_.front_right);
      brake_pressure_target_bar_.front_right += std::min(
        (std::abs(additional_brake_pressure) -
         additional_pressure_left * brake_friction_.front_left) /
          brake_friction_.front_right,
        right_pressure_headroom);
    }
  }

  brake_pressure_target_bar_.front_left =
    std::clamp(brake_pressure_target_bar_.front_left, 0.0, p_.max_brake_pressure);
  brake_pressure_target_bar_.front_right =
    std::clamp(brake_pressure_target_bar_.front_right, 0.0, p_.max_brake_pressure);
  brake_pressure_target_bar_.rear_left =
    std::clamp(brake_pressure_target_bar_.rear_left, 0.0, p_.max_brake_pressure);
  brake_pressure_target_bar_.rear_right =
    std::clamp(brake_pressure_target_bar_.rear_right, 0.0, p_.max_brake_pressure);

  logger_->log("active", esc_active_);
  logger_->log("psi_dot_target", psi_dot_target);
  logger_->log("psi_dot", psi_dot_);
  logger_->log("psi_dot_error", psi_dot_error_filtered_);
  logger_->log("beta_target", beta_target);
  logger_->log("beta", beta_);
  logger_->log("beta_error", beta_error_filtered_);
  logger_->log("brake_pressure_input_bar", brake_pressure_input_bar_);
  logger_->log("brake_pressure_output_bar", brake_pressure_target_bar_);
  logger_->log("feedback_psi_dot", pid_feedback_psi_dot.feedback);
  logger_->log("feedback_psi_dot_p", pid_feedback_psi_dot.feedback_p);
  logger_->log("feedback_psi_dot_i", pid_feedback_psi_dot.feedback_i);
  logger_->log("feedback_psi_dot_d", pid_feedback_psi_dot.feedback_d);
  logger_->log("feedback_beta", pid_feedback_beta.feedback);
  logger_->log("feedback_beta_p", pid_feedback_beta.feedback_p);
  logger_->log("feedback_beta_i", pid_feedback_beta.feedback_i);
  logger_->log("feedback_beta_d", pid_feedback_beta.feedback_d);
  logger_->log("yaw_moment", yaw_moment);
}
void ESC::set_target_brake_pressure(const Dpw & target_brake_pressure)
{
  brake_pressure_input_bar_ = target_brake_pressure;
}
void ESC::set_feedback_odometry(const tam::types::control::Odometry & odometry)
{
  velocity_ = std::hypot(odometry.velocity_mps.x, odometry.velocity_mps.y);
  beta_ = std::atan2(odometry.velocity_mps.y, odometry.velocity_mps.x);
  psi_dot_ = odometry.angular_velocity_radps.z;
}
void ESC::set_brake_friction_coefficients(
  const tam::types::common::DataPerWheel<double> & brake_friction)
{
  const auto brake_friction_values = brake_friction.to_array();
  for (size_t i = 0; i < brake_friction_.size(); ++i) {
    brake_friction_[i] =
      std::isfinite(brake_friction_values[i]) ? std::max(brake_friction_values[i], 1e-6) : 1e-6;
  }
}
void ESC::set_additional_esc_targets(
  const tam::types::control::AdditionalEspTargets & additional_esc_targets)
{
  additional_esc_targets_ = additional_esc_targets;
}
void ESC::set_slip_angle_valid(const bool slip_angle_valid)
{
  esc_enabled_ = p_.esc_enabled && slip_angle_valid;
}
bool ESC::get_esc_active() const
{
  return esc_active_;
}
tam::types::common::DataPerWheel<double> ESC::get_brake_pressure_target_bar() const
{
  return brake_pressure_target_bar_;
}
tam::pmg::MgmtInterface::SharedPtr ESC::get_param_handler() const
{
  return param_manager_;
}
tam::tsl::LoggerAccessInterface::SharedPtr ESC::get_debug_out() const
{
  return logger_;
}
}  // namespace tam::control
