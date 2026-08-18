// Copyright 2026 Phillip Pitschi
#pragma once

#include <memory>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "vehicle_handler_cpp/vehicle_handler.hpp"

namespace tam::control
{
class SlipCalculation
{
  using dpw = tam::types::common::DataPerWheel<double>;

public:
  SlipCalculation();

  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const;

  void step();

  void set_feedback_wheelspeed_radps(const dpw & wheelspeed_radps);
  void set_feedback_drivetrain(
    const tam::types::control::DriveTrainFeedback & drivetrain_feedback);
  void set_wheelspeed_ok(bool status);
  void set_omega_engine_ok(bool status);
  void set_feedback_odometry(const tam::types::control::Odometry & odometry);
  void set_feedback_acceleration(
    const tam::types::control::AccelerationwithCovariances & acceleration);
  void set_feedback_brake_pressure_Pa(const dpw & brake_pressure_Pa);
  void set_feedback_steering_rad(
    const tam::types::control::AutowareSteeringReport & steering_report);
  void set_motion_control_steering_request(double steering_request);
  void set_steering_angle_valid(bool valid);

  dpw get_wheelslips() const;
  dpw get_virtual_wheelslips() const;
  dpw get_vertical_tire_loads() const;
  dpw get_dynamic_tire_radius_m() const;
  dpw get_slip_angles() const;
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const;

private:
  void declare_and_update_parameters();
  dpw calc_virtual_wheelspeed(
    const tam::types::control::DriveTrainFeedback & feedback,
    const dpw & yaw_vx_vehicle_mps) const;

  bool wheelspeed_ok_{};
  bool omega_engine_ok_{};
  bool steering_angle_valid_{};
  double steering_request_{};
  tam::types::control::DriveTrainFeedback drivetrain_feedback_{};
  dpw wheelspeed_radps_{};
  dpw virtual_wheelspeed_radps_{};
  tam::types::control::Odometry odometry_{};
  tam::types::control::AccelerationwithCovariances acceleration_{};
  dpw brake_pressure_Pa_{};
  tam::types::control::AutowareOperationMode operation_mode_{};
  tam::types::control::AutowareSteeringReport steering_report_{};

  dpw wheelslip_{};
  dpw wheelslip_virtual_{};
  dpw slip_angle_{};
  dpw vertical_load_{};

  dpw dynamic_tire_radius_m_{};
  tam::helpers::control::FirstOrderLowPass<double> pt1_filter_accel_long_{0.0, 0.8};
  tam::helpers::control::FirstOrderLowPass<double> pt1_filter_speed_interp_{1.0, 0.8};
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash{};
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  std::unique_ptr<tam::common::VehicleHandler> vehicle_handler_ =
    tam::common::VehicleHandler::from_pkg_config();

  struct
  {
    double v_min;
    double tw_front_m;
    double tw_rear_m;
    double l_front_m;
    double wheelbase_m;
    double tireradius_front_m_20mps;
    double tireradius_rear_m_20mps;
    double final_drive_ratio;
    std::vector<double> tire_radius_velocity_scaling_vel_points{};
    std::vector<double> tire_radius_velocity_scaling_scale_factors{};
    std::vector<double> i_gearset_table{};
  } p_;
};
}  // namespace tam::control
