// Copyright 2026 Phillip Pitschi
#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "limit_handler_cpp/helpers.hpp"
#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_helpers_cpp/numerical.hpp"
#include "vehicle_handler_cpp/vehicle_handler.hpp"

namespace tam::control
{
  class TrackingControllerPurePursuitCpp
  {
  private:
    struct
    {
      // Controller parameters
      double tS;
      double static_steering_offset_rad;
      double lookahead_time_lat_s;
      double lookahead_time_long_ff_s;
      double lookahead_time_lat_ff_s;
      double minimum_lookahead_distance_lat_m;
      double curvature_lookahead_gain;
      double curvature_filter_Ts;
      double enable_lat_feedforward_perc;
      double velocity_p_gain;
      double vel_error_filter_time_constant;
      double steering_angle_max_rad;
      double steering_angle_min_rad;
      double wheelbase_m;
    } p_;

    // Parameters
    tam::pmg::ParamValueManager::SharedPtr param_manager_ =
        std::make_shared<tam::pmg::ParamValueManager>();
    std::size_t previous_param_state_hash_ = 0;

    // Logger
    tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

    // Trajectory data
    tam::types::control::Trajectory trajectory_buffer_{};
    tam::types::control::ControlConstraints constraints_buffer_{};
    tam::types::control::AdditionalTrajectoryInfos additional_trajectory_info_buffer_{};
    // Path matching
    double s_current_m_{0.0};
    float idx_current_{0.0F};
    tam::helpers::cosy::CurvilinearCosyPtr cosy_;

    // Feedback
    double current_velocity_mps_{};
    tam::types::control::Odometry odometry_buffer_{};

    // Outputs
    tam::types::control::LateralControlCommand lat_cmd_;
    double longitudinal_acceleration_request_mps2_{0.0};
    double steering_request_rad_{0.0};
    double ay_esp_target_mps2_{0.0};

    // Filters
    tam::helpers::control::DiscreteTransferFunction vel_error_filter_;
    tam::helpers::control::FirstOrderLowPass<double> curvature_filter{};

    // Vehicle model
    std::unique_ptr<tam::common::VehicleHandler> vehicle_handler_ =
        tam::common::VehicleHandler::from_pkg_config();

    // Internal methods
    void declare_and_update_parameters();

  public:
    TrackingControllerPurePursuitCpp();

    // Control loop
    void step();

    // Inputs
    void set_target_trajectory(const tam::types::control::Trajectory &target_trajectory);
    void set_constraints(
        const tam::types::control::ControlConstraints &control_constraints);
    void set_additional_trajectory_info(
        const tam::types::control::AdditionalTrajectoryInfos &additional_trajectory_infos);
    void set_feedback_odometry(const tam::types::control::Odometry &current_odometry);
    void set_path_matching_result(
        const tam::types::common::FrenetPose &frenet_pose, const float matching_idx);

    // Outputs
    double get_longitudinal_command();
    tam::types::control::AdditionalEspTargets get_additional_esp_targets();
    tam::types::control::LateralControlCommand get_lateral_command();

    // Diagnostics and parameters
    tam::pmg::MgmtInterface::SharedPtr get_param_handler() { return param_manager_; }
    tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() { return logger_; }
  };
} // namespace tam::control
