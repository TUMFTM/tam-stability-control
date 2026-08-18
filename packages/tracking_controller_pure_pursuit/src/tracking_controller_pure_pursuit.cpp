// Copyright 2026 Phillip Pitschi

#include "tracking_controller_pure_pursuit/tracking_controller_pure_pursuit.hpp"

#include <algorithm>

tam::control::TrackingControllerPurePursuitCpp::TrackingControllerPurePursuitCpp()
{
  declare_and_update_parameters();
}
void tam::control::TrackingControllerPurePursuitCpp::declare_and_update_parameters()
{
  auto decl = [this](std::string name, double val)
  {
    return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
  };

  p_.tS = std::max(decl("tS", 0.01), 1e-6);
  p_.static_steering_offset_rad = decl("P_VDC_SteeringOvalComp_rad", 0.0);
  p_.velocity_p_gain = decl("velocity_p_gain", 2.0);
  p_.vel_error_filter_time_constant =
      std::max(decl("vel_error_filter_time_constant", 0.5), 1e-6);
  p_.lookahead_time_long_ff_s = decl("lookahead_time_long_ff_s", 0.05);
  p_.lookahead_time_lat_s = decl("lookahead_time_lat_s", 0.1);
  p_.lookahead_time_lat_ff_s = decl("lookahead_time_lat_ff_s", 0.1);
  p_.minimum_lookahead_distance_lat_m = decl("min_lookahead_distance_lat_m", 1.0);
  p_.enable_lat_feedforward_perc = decl("enable_lat_feedforward_perc", 0.0);
  p_.curvature_lookahead_gain = decl("curvature_lookahead_gain", 0.0);
  p_.curvature_filter_Ts = std::max(decl("curvature_filter_Ts", 0.05), 1e-6);
  p_.wheelbase_m = decl("vehicle.dimension.wheelbase_m", 2.971);
  p_.steering_angle_max_rad = decl("vehicle.steering.max_angle", 0.43);
  p_.steering_angle_min_rad = decl("vehicle.steering.min_angle", -0.43);

  curvature_filter.set_tf_pole(std::exp(-p_.tS / p_.curvature_filter_Ts));

  // Configure the velocity-error filter.
  std::vector<double> vel_error_filter_num_coeffs = {-p_.tS, -p_.tS};
  std::vector<double> vel_error_filter_den_coeffs = {
      -p_.tS - 2 * p_.vel_error_filter_time_constant,
      -p_.tS + 2 * p_.vel_error_filter_time_constant};
  vel_error_filter_ = tam::helpers::control::DiscreteTransferFunction(
      vel_error_filter_num_coeffs, vel_error_filter_den_coeffs, 1.0);

  previous_param_state_hash_ = param_manager_->get_state_hash();
}
void tam::control::TrackingControllerPurePursuitCpp::step()
{
  if (param_manager_->get_state_hash() != previous_param_state_hash_)
  {
    declare_and_update_parameters();
  }

  if (
      trajectory_buffer_.points.size() < 2 || constraints_buffer_.points.size() < 2 ||
      additional_trajectory_info_buffer_.points.size() < 2)
  {
    return;
  }

  // Optionally advance the longitudinal matching point.
  if (cosy_)
  {
    tam::types::control::Odometry odometry_lookahead = odometry_buffer_;
    odometry_lookahead.position_m.x += std::cos(odometry_buffer_.orientation_rad.z) *
                                       std::max(odometry_buffer_.velocity_mps.x, 3.0) *
                                       p_.lookahead_time_long_ff_s;
    odometry_lookahead.position_m.y += std::sin(odometry_buffer_.orientation_rad.z) *
                                       std::max(odometry_buffer_.velocity_mps.x, 3.0) *
                                       p_.lookahead_time_long_ff_s;
    std::tuple<Eigen::Vector3d, float> return_val = cosy_->convert_to_sn_and_get_idx_global(
        odometry_lookahead, std::numeric_limits<double>::max());
    idx_current_ = std::get<1>(return_val);
  }

  const float idx_current_clamped = std::clamp(
      idx_current_, 0.0F, static_cast<float>(trajectory_buffer_.points.size() - 1));
  const size_t idx_current_floor = std::min(
      static_cast<size_t>(std::floor(idx_current_clamped)), trajectory_buffer_.points.size() - 2);
  const double idx_current_remainder = std::clamp(
      static_cast<double>(idx_current_clamped) - static_cast<double>(idx_current_floor), 0.0, 1.0);

  double current_long_acc_target = std::lerp(
      trajectory_buffer_.points[idx_current_floor].acceleration_mps2.x,
      trajectory_buffer_.points[idx_current_floor + 1].acceleration_mps2.x,
      idx_current_remainder);
  double current_velocity_target = std::lerp(
      trajectory_buffer_.points[idx_current_floor].velocity_mps.x,
      trajectory_buffer_.points[idx_current_floor + 1].velocity_mps.x,
      idx_current_remainder);
  const auto current_constraints =
      tam::helpers::control::find_constraint_point(constraints_buffer_, idx_current_clamped);
  const double current_long_acc_max = current_constraints.a_x_max_mps2.x;
  const double current_long_acc_min = current_constraints.a_x_min_mps2.x;
  const double current_velocity_error = current_velocity_target - current_velocity_mps_;
  const double current_velocity_error_filtered = vel_error_filter_.step(current_velocity_error);

  // Add proportional velocity feedback to planned acceleration.
  longitudinal_acceleration_request_mps2_ =
      current_long_acc_target + p_.velocity_p_gain * current_velocity_error_filtered;

  // Hold the vehicle for a standstill trajectory.
  if (current_velocity_target < 0.05)
  {
    longitudinal_acceleration_request_mps2_ =
        std::min(longitudinal_acceleration_request_mps2_, -4.5);
  }

  // Enforce the local longitudinal constraints.
//   longitudinal_acceleration_request_mps2_ =
//       std::clamp(longitudinal_acceleration_request_mps2_, current_long_acc_min, current_long_acc_max);

  // Choose a curvature-dependent pure-pursuit lookahead.
  std::vector<double> kappa_traj_vec;
  std::transform(
      additional_trajectory_info_buffer_.points.begin(),
      additional_trajectory_info_buffer_.points.end(), std::back_inserter(kappa_traj_vec),
      [](const tam::types::control::AdditionalInfoPoint &point)
      { return point.kappa_1pm; });
  double kappa_traj_current =
      tam::helpers::numerical::interp_from_idx(kappa_traj_vec, idx_current_);

  double lookahead_distance =
      p_.minimum_lookahead_distance_lat_m + p_.lookahead_time_lat_s * current_velocity_mps_ +
      p_.curvature_lookahead_gain * 1.0 / std::max(std::abs(kappa_traj_current), 1e-8);
  lookahead_distance = std::max(lookahead_distance, 1e-6);
  double s_lookahead_m = s_current_m_ + lookahead_distance;

  std::vector<double> x_traj_vec;
  std::transform(
      trajectory_buffer_.points.begin(), trajectory_buffer_.points.end(),
      std::back_inserter(x_traj_vec),
      [](const tam::types::control::TrajectoryPoint &point)
      { return point.position_m.x; });
  std::vector<double> y_traj_vec;
  std::transform(
      trajectory_buffer_.points.begin(), trajectory_buffer_.points.end(),
      std::back_inserter(y_traj_vec),
      [](const tam::types::control::TrajectoryPoint &point)
      { return point.position_m.y; });
  std::vector<double> s_vec;
  std::transform(
      additional_trajectory_info_buffer_.points.begin(),
      additional_trajectory_info_buffer_.points.end(), std::back_inserter(s_vec),
      [](const tam::types::control::AdditionalInfoPoint &point)
      { return point.s_local_m; });
  double x_traj = tam::helpers::numerical::interp(s_lookahead_m, s_vec, x_traj_vec);
  double y_traj = tam::helpers::numerical::interp(s_lookahead_m, s_vec, y_traj_vec);

  // Fit a circular segment from the rear axle to the lookahead point.
  double dy_vehicle =
      -std::sin(odometry_buffer_.orientation_rad.z) * (x_traj - odometry_buffer_.position_m.x) +
      std::cos(odometry_buffer_.orientation_rad.z) * (y_traj - odometry_buffer_.position_m.y);
  double curvature_raw = 2 * dy_vehicle / (lookahead_distance * lookahead_distance);

  double curvature = curvature_filter.step(curvature_raw);

  double steering_request_fb = std::atan(p_.wheelbase_m * curvature);

  // Add curvature feedforward.
  double kappa_traj_lookahead_ = tam::helpers::numerical::interp(
      s_current_m_ + current_velocity_mps_ * p_.lookahead_time_lat_ff_s, s_vec, kappa_traj_vec);
  double steering_request_ff =
      p_.enable_lat_feedforward_perc * std::atan(p_.wheelbase_m * kappa_traj_lookahead_);

  steering_request_rad_ =
      steering_request_ff + steering_request_fb + p_.static_steering_offset_rad;

  // Publish the lateral-acceleration request for ESC.
  ay_esp_target_mps2_ = curvature * current_velocity_mps_ * current_velocity_mps_;

  // Enforce tire-steering limits.
  steering_request_rad_ =
      std::clamp(steering_request_rad_, p_.steering_angle_min_rad, p_.steering_angle_max_rad);

  // Diagnostics.
  logger_->log("dot_d_numerical_mps", 0.0);
  logger_->log("LatAcc_Target_mps2", ay_esp_target_mps2_);
  logger_->log("current_velocity_target_mps", current_velocity_target);
  logger_->log("current_long_acc_target_mps2", current_long_acc_target);
  logger_->log("current_velocity_mps", current_velocity_mps_);
  logger_->log("current_velocity_error_mps", current_velocity_error);
  logger_->log("current_velocity_error_filtered_mps", current_velocity_error_filtered);
  logger_->log("current_long_acc_max_mps2", current_long_acc_max);
  logger_->log("current_long_acc_min_mps2", current_long_acc_min);
  logger_->log("idx_current", idx_current_);
  logger_->log("long_acc_request_mps2", longitudinal_acceleration_request_mps2_);
  logger_->log("lookahead_distance_m", lookahead_distance);
  logger_->log("x_traj_lookahead_m", x_traj);
  logger_->log("y_traj_lookahead_m", y_traj);
  logger_->log("dy_vehicle_m", dy_vehicle);
  logger_->log("curvature_raw", curvature_raw);
  logger_->log("curvature_filtered", curvature);
  logger_->log("steering_request_fb_rad", steering_request_fb);
  logger_->log("curvature_lookahead", kappa_traj_lookahead_);
  logger_->log("steering_request_ff_rad", steering_request_ff);
  logger_->log("steering_request_rad", steering_request_rad_);
}
void tam::control::TrackingControllerPurePursuitCpp::set_target_trajectory(
    const tam::types::control::Trajectory &target_trajectory)
{
  trajectory_buffer_ = target_trajectory;
  cosy_ = tam::helpers::cosy::CurvilinearCosy::create(target_trajectory)->build();
}
void tam::control::TrackingControllerPurePursuitCpp::set_feedback_odometry(
    const tam::types::control::Odometry &current_odometry)
{
  odometry_buffer_ = current_odometry;
  current_velocity_mps_ = current_odometry.velocity_mps.x;
}
void tam::control::TrackingControllerPurePursuitCpp::set_additional_trajectory_info(
    const tam::types::control::AdditionalTrajectoryInfos &additional_trajectory_infos)
{
  additional_trajectory_info_buffer_ = additional_trajectory_infos;
}
void tam::control::TrackingControllerPurePursuitCpp::set_path_matching_result(
    const tam::types::common::FrenetPose &frenet_pose, const float matching_idx)
{
  if (trajectory_buffer_.points.empty()) {
    return;
  }

  s_current_m_ = frenet_pose.s;
  idx_current_ = std::clamp(matching_idx, 0.0f,
                            static_cast<float>(trajectory_buffer_.points.size() - 1));
}
void tam::control::TrackingControllerPurePursuitCpp::set_constraints(
    const tam::types::control::ControlConstraints &control_constraints)
{
  constraints_buffer_ = control_constraints;
}
double tam::control::TrackingControllerPurePursuitCpp::get_longitudinal_command()
{
  return longitudinal_acceleration_request_mps2_;
}
tam::types::control::AdditionalEspTargets
tam::control::TrackingControllerPurePursuitCpp::get_additional_esp_targets()
{
  tam::types::control::AdditionalEspTargets esp_targets{};
  esp_targets.ay_request_mps2 = ay_esp_target_mps2_;
  return esp_targets;
}
tam::types::control::LateralControlCommand
tam::control::TrackingControllerPurePursuitCpp::get_lateral_command()
{
  lat_cmd_.steering_angle_tire_rad = steering_request_rad_;
  return lat_cmd_;
}
