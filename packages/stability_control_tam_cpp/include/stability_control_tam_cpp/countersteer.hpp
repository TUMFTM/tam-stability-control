// Copyright 2026 Phillip Pitschi

#pragma once

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "vehicle_handler_cpp/vehicle_handler.hpp"
namespace tam::control
{
/// @class CountersteerSystem
/// @brief Applies countersteer from estimated axle sideslip.
class CountersteerSystem
{
private:
  // Parameters
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash = 0;
  // Controller parameters
  struct
  {
    bool countersteer_enabled;            ///< Enable/disable countersteer algorithm
    double slip_angle_filter_Ts;          ///< Time constant for slip angle LPF
    double countersteer_enable_sideslip;  ///< Sideslip threshold to activate countersteer
    double countersteer_steering_factor;  ///< Scaling factor for steering correction
    double tS;                            ///< Control loop sampling time
    double steering_angle_min_rad;        ///< Minimum steering angle command
    double steering_angle_max_rad;        ///< Maximum steering angle command
  } p_;
  // Logger
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  // Vehicle model
  std::unique_ptr<tam::common::VehicleHandler> vehicle_handler_ =
    tam::common::VehicleHandler::from_pkg_config();

  // State
  bool steering_feedback_available_{};            ///< Flag: steering sensor data available
  bool slip_angle_valid_{};                       ///< Flag: slip angle calculation valid
  bool countersteer_active_{};                    ///< Flag: countersteer correction active
  double weighted_sideslip_front_filtered_{};     ///< Filtered front axle sideslip angle
  double weighted_sideslip_rear_filtered_{};      ///< Filtered rear axle sideslip angle
  double weighted_sideslip_front_{};              ///< Unfiltered front axle sideslip angle
  double weighted_sideslip_rear_{};               ///< Unfiltered rear axle sideslip angle
  double steering_request_motion_control_rad_{};  ///< Steering request from motion control
  double steering_request_rad_{};                 ///< Final steering request with countersteer
  tam::types::control::Odometry odometry_{};      ///< Vehicle odometry (position, velocity)
  double steering_angle_rad_{};                   ///< Current steering wheel angle
  double long_acc_mps2_{};                        ///< Longitudinal acceleration from IMU
  double lat_acc_mps2_{};                         ///< Lateral acceleration from IMU

  // Sideslip filters
  tam::helpers::control::FirstOrderLowPass<double> slip_angle_filter_front_{0.0, 0.7};
  tam::helpers::control::FirstOrderLowPass<double> slip_angle_filter_rear_{0.0, 0.7};

  // Calculations
  void declare_and_update_parameter();
  bool calc_slip_angles(
    const tam::types::control::Odometry & odometry, const double steering_angle,
    const double lat_acc, const double long_acc);
  double calc_countersteer_steering_angle(
    const double steering_request_rad, const double sideslip_front, const double sideslip_rear);

public:
  CountersteerSystem();
  void step();

  // Inputs
  void set_feedback_odometry(const tam::types::control::Odometry & odometry);
  void set_feedback_acceleration(
    const tam::types::control::AccelerationwithCovariances & current_acceleration);
  void set_feedback_steering(const tam::types::control::AutowareSteeringReport & current_steering);
  void set_steering_valid(const bool valid);
  void set_slip_angle_valid(const bool valid);
  void set_motion_control_steering_request(const double steering_request);

  // Outputs
  double get_steering_request_rad() const;
  double get_weighted_sideslip_front() const;
  double get_weighted_sideslip_rear() const;
  bool get_countersteer_active() const;
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const;
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const;
};
}  // namespace tam::control
