// Copyright Phillip Pitschi 2026

#include "stability_control_tam_cpp/countersteer.hpp"

#include <algorithm>
#include <cmath>

namespace tam::control
{
CountersteerSystem::CountersteerSystem() { declare_and_update_parameter(); }
void CountersteerSystem::declare_and_update_parameter()
{
  // Parameter helper
  auto decl_double = [this](std::string name, double val) {
    return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  };

  // Controller parameters
  p_.slip_angle_filter_Ts =
    std::max(decl_double("countersteer_system.slip_angle_filter_Ts", 0.15), 1e-6);
  p_.countersteer_enable_sideslip = decl_double("countersteer_system.enable_sideslip", 0.009);
  p_.countersteer_steering_factor = decl_double("countersteer_system.steering_factor", 1.0);
  p_.tS = std::max(decl_double("tS", 0.01), 1e-6);
  p_.steering_angle_min_rad = decl_double("vehicle.steering.min_angle", -0.43);
  p_.steering_angle_max_rad = decl_double("vehicle.steering.max_angle", 0.43);
  p_.countersteer_enabled =
    param_manager_
      ->declare_and_get_value(
        "countersteer_system.enabled", false, tam::pmg::ParameterType::BOOL, "")
      .as_bool();
  previous_param_state_hash = param_manager_->get_state_hash();

  // Update filter poles.
  slip_angle_filter_front_.set_tf_pole(std::exp(-p_.tS / p_.slip_angle_filter_Ts));
  slip_angle_filter_rear_.set_tf_pole(std::exp(-p_.tS / p_.slip_angle_filter_Ts));
}
bool CountersteerSystem::calc_slip_angles(
  const tam::types::control::Odometry & odometry, const double steering_angle, const double lat_acc,
  const double long_acc)
{
  tam::types::common::DataPerWheel<double> alpha_rad =
    vehicle_handler_->calc_slip_angles(odometry, steering_angle);

  tam::types::common::DataPerWheel<double> tire_loads =
    vehicle_handler_->estimate_vertical_forces(odometry.velocity_mps, long_acc, lat_acc);

  // Compute load-weighted axle sideslip.
  const double front_tire_load = tire_loads.front_left + tire_loads.front_right;
  const double rear_tire_load = tire_loads.rear_left + tire_loads.rear_right;
  if (
    !std::isfinite(front_tire_load) || !std::isfinite(rear_tire_load) ||
    front_tire_load <= 1e-6 || rear_tire_load <= 1e-6) {
    return false;
  }

  weighted_sideslip_front_ = (tire_loads.front_left * alpha_rad.front_left +
                              tire_loads.front_right * alpha_rad.front_right) /
                             front_tire_load;
  weighted_sideslip_rear_ =
    (tire_loads.rear_left * alpha_rad.rear_left + tire_loads.rear_right * alpha_rad.rear_right) /
    rear_tire_load;

  // Filter axle sideslip.
  weighted_sideslip_front_filtered_ = slip_angle_filter_front_.step(weighted_sideslip_front_);
  weighted_sideslip_rear_filtered_ = slip_angle_filter_rear_.step(weighted_sideslip_rear_);
  return true;
}
double CountersteerSystem::calc_countersteer_steering_angle(
  const double steering_request_rad, const double sideslip_front, const double sideslip_rear)
{
  double steering_request_rad_countersteer = steering_request_rad;
  countersteer_active_ = false;

  // Countersteer when rear sideslip dominates.
  if (
    std::abs(sideslip_rear) > std::abs(sideslip_front) &&
    std::abs(sideslip_rear) > p_.countersteer_enable_sideslip && p_.countersteer_enabled) {
    countersteer_active_ = true;
    steering_request_rad_countersteer +=
      p_.countersteer_steering_factor * (sideslip_front - sideslip_rear);
  }
  return steering_request_rad_countersteer;
}
void CountersteerSystem::step()
{
  // Refresh parameters.
  if (param_manager_->get_state_hash() != previous_param_state_hash) {
    declare_and_update_parameter();
  }

  // Fall back to the motion-control steering request.
  if (!steering_feedback_available_) {
    steering_angle_rad_ = steering_request_motion_control_rad_;
  }

  if (
    slip_angle_valid_ &&
    calc_slip_angles(odometry_, steering_angle_rad_, lat_acc_mps2_, long_acc_mps2_)) {
    steering_request_rad_ = calc_countersteer_steering_angle(
      steering_request_motion_control_rad_, weighted_sideslip_front_filtered_,
      weighted_sideslip_rear_filtered_);
  } else {
    // Pass through the motion-control request.
    countersteer_active_ = false;
    steering_request_rad_ = steering_request_motion_control_rad_;
  }

  steering_request_rad_ = std::clamp(
    steering_request_rad_, p_.steering_angle_min_rad, p_.steering_angle_max_rad);

  logger_->log("steering_input", steering_request_motion_control_rad_);
  logger_->log(
    "delta_steering_countersteer", steering_request_rad_ - steering_request_motion_control_rad_);
  logger_->log("steering_request_rad", steering_request_rad_);
  logger_->log("active", countersteer_active_);
  logger_->log("weighted_slip_angle_front", weighted_sideslip_front_);
  logger_->log("weighted_slip_angle_rear", weighted_sideslip_rear_);
  logger_->log("weighted_slip_angle_front_filtered", weighted_sideslip_front_filtered_);
  logger_->log("weighted_slip_angle_rear_filtered", weighted_sideslip_rear_filtered_);
}
void CountersteerSystem::set_feedback_odometry(const tam::types::control::Odometry & odometry)
{
  odometry_ = odometry;
}
void CountersteerSystem::set_feedback_acceleration(
  const tam::types::control::AccelerationwithCovariances & acceleration)
{
  long_acc_mps2_ = acceleration.acceleration_mps2.x;
  lat_acc_mps2_ = acceleration.acceleration_mps2.y;
}
void CountersteerSystem::set_feedback_steering(
  const tam::types::control::AutowareSteeringReport & steering_feedback)
{
  steering_angle_rad_ = steering_feedback.steering_angle_tire_rad;
}
void CountersteerSystem::set_steering_valid(const bool valid)
{
  steering_feedback_available_ = valid;
}
void CountersteerSystem::set_slip_angle_valid(const bool valid)
{
  slip_angle_valid_ = valid;
}
void CountersteerSystem::set_motion_control_steering_request(const double steering_request)
{
  steering_request_motion_control_rad_ = steering_request;
}
double CountersteerSystem::get_steering_request_rad() const
{
  return steering_request_rad_;
}
double CountersteerSystem::get_weighted_sideslip_front() const
{
  return weighted_sideslip_front_filtered_;
}
double CountersteerSystem::get_weighted_sideslip_rear() const
{
  return weighted_sideslip_rear_filtered_;
}
bool CountersteerSystem::get_countersteer_active() const
{
  return countersteer_active_;
}
tam::pmg::MgmtInterface::SharedPtr CountersteerSystem::get_param_handler() const
{
  return param_manager_;
}
tam::tsl::LoggerAccessInterface::SharedPtr CountersteerSystem::get_debug_out() const
{
  return logger_;
}
}  // namespace tam::control
