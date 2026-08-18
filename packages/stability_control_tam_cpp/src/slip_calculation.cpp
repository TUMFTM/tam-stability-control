// Copyright 2026 Phillip Pitschi

#include <algorithm>
#include <cmath>

#include "stability_control_tam_cpp/slip_calculation.hpp"

namespace tam::control
{
SlipCalculation::SlipCalculation() { declare_and_update_parameters(); }

tam::pmg::MgmtInterface::SharedPtr SlipCalculation::get_param_handler() const
{
  return param_manager_;
}

void SlipCalculation::step()
{
  if (param_manager_->get_state_hash() != previous_param_state_hash) {
    declare_and_update_parameters();
  }

  const double steering_angle_rad =
    steering_angle_valid_ ? steering_report_.steering_angle_tire_rad : steering_request_;
  dynamic_tire_radius_m_ = vehicle_handler_->calc_dynamic_tire_radius(odometry_.velocity_mps);

  const dpw v_tire_omega = wheelspeed_radps_ * dynamic_tire_radius_m_;
  const dpw vy_vehicle_mps =
    dpw::from_front_and_rear(p_.l_front_m, -(p_.wheelbase_m - p_.l_front_m)) *
      odometry_.angular_velocity_radps.z +
    odometry_.velocity_mps.y;
  const dpw sine = dpw::from_front_and_rear(std::sin(-steering_angle_rad), 0.0);
  const dpw cose = dpw::from_front_and_rear(std::cos(-steering_angle_rad), 1.0);
  const dpw yaw_vx_vehicle_mps =
    dpw::from_front_and_rear(p_.tw_front_m, p_.tw_rear_m) * dpw::from_left_and_right(-1, 1) *
    0.5 * odometry_.angular_velocity_radps.z;

  const dpw vx_vehicle_mps_no_long_slip = (v_tire_omega + sine * vy_vehicle_mps) / cose;
  const dpw vx_cog_vehicle_mps_no_long_slip_per_wheel =
    vx_vehicle_mps_no_long_slip - yaw_vx_vehicle_mps;
  const double vx_cog_vehicle_mps_no_long_slip =
    0.5 * (vx_cog_vehicle_mps_no_long_slip_per_wheel.front_left +
           vx_cog_vehicle_mps_no_long_slip_per_wheel.front_right);

  const double max_brake_pressure =
    std::max(brake_pressure_Pa_.front_left, brake_pressure_Pa_.front_right);
  const double accel_current_long = pt1_filter_accel_long_.step(acceleration_.acceleration_mps2.x);
  const double interp_factor = std::max(
    std::max(
      std::clamp(max_brake_pressure / 5e5, 0.0, 1.0),
      std::clamp(-(accel_current_long + 5.0), 0.0, 1.0)),
    std::clamp(
      std::max(std::abs(wheelslip_.front_left), std::abs(wheelslip_.front_right)) / 5.0 - 1.0,
      0.0, 1.0));
  const double interp_factor_filtered = pt1_filter_speed_interp_.step(interp_factor);
  const double vehicle_velocity_x_mps = interp_factor_filtered * odometry_.velocity_mps.x +
                                        (1.0 - interp_factor_filtered) *
                                          vx_cog_vehicle_mps_no_long_slip;

  constexpr double slip_fade = 2.0;
  const double low_speed_scale = std::clamp(
    std::pow(std::max(vehicle_velocity_x_mps - std::max(p_.v_min - slip_fade, 0.0), 0.0), 8) /
      std::pow(std::max(slip_fade, 0.1), 8),
    0.0, 1.0);

  tam::types::control::Odometry odometry = odometry_;
  odometry.velocity_mps.x = vehicle_velocity_x_mps;
  wheelslip_ =
    vehicle_handler_->calc_long_slip(odometry, steering_angle_rad, wheelspeed_radps_) *
    low_speed_scale * 100.0;
  slip_angle_ =
    vehicle_handler_->calc_slip_angles(odometry, steering_angle_rad) * low_speed_scale;

  virtual_wheelspeed_radps_ = calc_virtual_wheelspeed(drivetrain_feedback_, yaw_vx_vehicle_mps);
  virtual_wheelspeed_radps_.front_left = wheelspeed_radps_.front_left;
  virtual_wheelspeed_radps_.front_right = wheelspeed_radps_.front_right;
  wheelslip_virtual_ =
    vehicle_handler_->calc_long_slip(odometry, steering_angle_rad, virtual_wheelspeed_radps_) *
    low_speed_scale * 100.0;

  if (!wheelspeed_ok_) {
    wheelslip_ = dpw(0.0);
    wheelslip_virtual_.front_left = 0.0;
    wheelslip_virtual_.front_right = 0.0;
  }
  if (!omega_engine_ok_) {
    wheelslip_virtual_.front_left = wheelslip_.front_left;
    wheelslip_virtual_.front_right = wheelslip_.front_right;
    wheelslip_virtual_.rear_left = 0.0;
    wheelslip_virtual_.rear_right = 0.0;
  }

  vertical_load_ = vehicle_handler_->estimate_vertical_forces(
    odometry_.velocity_mps, acceleration_.acceleration_mps2.x, acceleration_.acceleration_mps2.y);

  logger_->log("long_slip", wheelslip_);
  logger_->log("vertical_load_N", vertical_load_);
  logger_->log("long_slip_virtual", wheelslip_virtual_);
  logger_->log("virtual_wheelspeed_radps", virtual_wheelspeed_radps_);
  logger_->log("slip_angle_rad", slip_angle_);
  logger_->log("low_speed_scale", low_speed_scale);
  logger_->log("velocity_interpolation/factor", interp_factor);
  logger_->log("velocity_interpolation/factor_filtered", interp_factor_filtered);
  logger_->log(
    "velocity_interpolation/vx_cog_no_long_slip_front", vx_cog_vehicle_mps_no_long_slip);
  logger_->log("velocity_interpolation/vx_state_est", odometry_.velocity_mps.x);
  logger_->log(
    "tire_rolling_radius/at_20mps",
    dpw::from_front_and_rear(p_.tireradius_front_m_20mps, p_.tireradius_rear_m_20mps));
  logger_->log("tire_rolling_radius/dynamic", dynamic_tire_radius_m_);
}

void SlipCalculation::set_feedback_wheelspeed_radps(const dpw & wheelspeed_radps)
{
  wheelspeed_radps_ = wheelspeed_radps;
}

void SlipCalculation::set_feedback_drivetrain(
  const tam::types::control::DriveTrainFeedback & drivetrain_feedback)
{
  drivetrain_feedback_ = drivetrain_feedback;
}

void SlipCalculation::set_wheelspeed_ok(bool status) { wheelspeed_ok_ = status; }

void SlipCalculation::set_omega_engine_ok(bool status) { omega_engine_ok_ = status; }

void SlipCalculation::set_feedback_odometry(const tam::types::control::Odometry & odometry)
{
  odometry_ = odometry;
}

void SlipCalculation::set_feedback_acceleration(
  const tam::types::control::AccelerationwithCovariances & acceleration)
{
  acceleration_ = acceleration;
}

void SlipCalculation::set_feedback_brake_pressure_Pa(const dpw & brake_pressure_Pa)
{
  brake_pressure_Pa_ = brake_pressure_Pa;
}

void SlipCalculation::set_feedback_steering_rad(
  const tam::types::control::AutowareSteeringReport & steering_report)
{
  steering_report_ = steering_report;
}

void SlipCalculation::set_motion_control_steering_request(double steering_request)
{
  steering_request_ = steering_request;
}

void SlipCalculation::set_steering_angle_valid(bool valid) { steering_angle_valid_ = valid; }

SlipCalculation::dpw SlipCalculation::get_wheelslips() const { return wheelslip_; }

SlipCalculation::dpw SlipCalculation::get_virtual_wheelslips() const
{
  return wheelslip_virtual_;
}

SlipCalculation::dpw SlipCalculation::get_vertical_tire_loads() const { return vertical_load_; }

SlipCalculation::dpw SlipCalculation::get_dynamic_tire_radius_m() const
{
  return dynamic_tire_radius_m_;
}

SlipCalculation::dpw SlipCalculation::get_slip_angles() const { return slip_angle_; }

tam::tsl::LoggerAccessInterface::SharedPtr SlipCalculation::get_debug_out() const
{
  return logger_;
}

SlipCalculation::dpw SlipCalculation::calc_virtual_wheelspeed(
  const tam::types::control::DriveTrainFeedback & feedback, const dpw & yaw_vx_vehicle_mps) const
{
  const int gear_index = std::max(0, static_cast<int>(feedback.gear_engaged) - 1);
  if (
    gear_index >= static_cast<int>(p_.i_gearset_table.size()) ||
    std::abs(p_.final_drive_ratio) < 1e-9 ||
    std::abs(p_.i_gearset_table[gear_index]) < 1e-9 ||
    std::abs(dynamic_tire_radius_m_.rear_left) < 1e-6 ||
    std::abs(dynamic_tire_radius_m_.rear_right) < 1e-6) {
    return dpw(0.0);
  }

  const double ratio = p_.final_drive_ratio * p_.i_gearset_table[gear_index];
  const double virtual_wheelspeed_rear_radps = feedback.omega_engine_radps / ratio;
  return dpw(
    0.0, 0.0,
    virtual_wheelspeed_rear_radps +
      yaw_vx_vehicle_mps.rear_left / dynamic_tire_radius_m_.rear_left,
    virtual_wheelspeed_rear_radps +
      yaw_vx_vehicle_mps.rear_right / dynamic_tire_radius_m_.rear_right);
}

void SlipCalculation::declare_and_update_parameters()
{
  p_.v_min = param_manager_
               ->declare_and_get_value(
                 "P_VDC_MinVelSlipCalc_mps", 3.0, tam::pmg::ParameterType::DOUBLE, "")
               .as_double();
  p_.tw_front_m =
    param_manager_
      ->declare_and_get_value(
        "vehicle.dimension.track_width_front", 1.639, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  p_.tw_rear_m =
    param_manager_
      ->declare_and_get_value(
        "vehicle.dimension.track_width_rear", 1.524, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  p_.l_front_m =
    param_manager_
      ->declare_and_get_value(
        "vehicle.dimension.distance_to_front_axle", 1.724, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  p_.wheelbase_m = param_manager_
                       ->declare_and_get_value(
                         "vehicle.dimension.wheelbase", 2.971, tam::pmg::ParameterType::DOUBLE, "")
                       .as_double();
  p_.tireradius_front_m_20mps =
    param_manager_
      ->declare_and_get_value(
        "tires.front_left.radius.radius_20mps", 0.293475, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  p_.tireradius_rear_m_20mps =
    param_manager_
      ->declare_and_get_value(
        "tires.rear_left.radius.radius_20mps", 0.3074348, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  p_.final_drive_ratio =
    param_manager_
      ->declare_and_get_value(
        "vehicle.drivetrain.transmission_ratio", 3.0, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  p_.tire_radius_velocity_scaling_vel_points =
    param_manager_
      ->declare_and_get_value(
        "tires.front_left.radius.velocity_scaling.velocity", std::vector<double>{20, 50, 80},
        tam::pmg::ParameterType::DOUBLE_ARRAY,
        "Velocity points for the corresponding scaling of the tire rolling diameter")
      .as_double_array();
  p_.tire_radius_velocity_scaling_scale_factors =
    param_manager_
      ->declare_and_get_value(
        "tires.front_left.radius.velocity_scaling.factor", std::vector<double>{1.0, 1.0065, 1.0205},
        tam::pmg::ParameterType::DOUBLE_ARRAY,
        "Scaling factor for the velocity induced increase in tire rolling diameter")
      .as_double_array();
  p_.i_gearset_table =
    param_manager_
      ->declare_and_get_value(
        "vehicle.drivetrain.gear_ratios",
        std::vector<double>{0.0, 2.9167, 1.875, 1.3809, 1.1154, 0.96, 0.8889},
        tam::pmg::ParameterType::DOUBLE_ARRAY, "")
      .as_double_array();
  previous_param_state_hash = param_manager_->get_state_hash();
}
}  // namespace tam::control
