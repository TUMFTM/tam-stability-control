// Copyright 2026 Phillip Pitschi
#pragma once

// Standard library
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"

// Messages
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "tum_msgs/msg/tum_additional_esp_targets.hpp"
#include "tum_msgs/msg/tum_control_constraints.hpp"

// Project types
#include "tum_types_cpp/control.hpp"

// Controller
#include "tracking_controller_pure_pursuit/tracking_controller_pure_pursuit.hpp"

// Parameters
#include "param_management_ros2_integration_cpp/helper_functions.hpp"

// Diagnostics
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"

// Type conversion
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

// Monitoring
#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TrackingControllerNode : public rclcpp::Node
{
public:
  TrackingControllerNode(const rclcpp::NodeOptions & options);

private:
  // Subscriptions
  message_filters::Subscriber<tier4_planning_msgs::msg::Trajectory> sub_trajectory_;
  message_filters::Subscriber<tum_msgs::msg::TUMControlConstraints> sub_constraints_;
  std::shared_ptr<message_filters::TimeSynchronizer<
    tier4_planning_msgs::msg::Trajectory, tum_msgs::msg::TUMControlConstraints>>
    sync_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  
  // Publishers
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    ctrl_cmd_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMAdditionalEspTargets>::SharedPtr
    additional_esp_targets_pub_{};
  
  // Node monitor
  tam::core::NodeMonitor::UniquePtr node_monitor_;
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_;
  std::string node_monitor_key_{"startup"};

  // Timer
  rclcpp::TimerBase::SharedPtr model_update_timer_;

  // Controller
  std::unique_ptr<tam::control::TrackingControllerPurePursuitCpp> tracking_controller_{};

  // Logging
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  tam::tsl::LoggerComposer::SharedPtr logger_composer_;
  std::shared_ptr<tam::tsl::TSLPublisher> tsl_publisher_;

  // Parameters
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // vehicle_handler
  std::unique_ptr<tam::common::VehicleHandler> vehicle_ = tam::common::VehicleHandler::from_pkg_config();

  // Path matching
  tam::helpers::cosy::CurvilinearCosyPtr cosy_;
  float matching_idx_{};
  
  // Timing diagnostics
  mutable std::chrono::steady_clock::time_point last_call_time;
  mutable uint64_t cycle_count_ = 0;
  // Input availability
  bool odometry_received_{false};
  bool trajectory_received_{false};
  // Input buffers
  tam::types::control::Odometry odometry_{};
  tam::types::control::Trajectory trajectory_buffer_{};
  tam::types::control::ControlConstraints constraints_buffer_{};

  // Internal methods
  void function_queue_callback();
  void model_update_callback();
  tam::types::control::AdditionalTrajectoryInfos prepare_additional_trajectory_infos(
    tam::types::control::Trajectory const & traj) const;
  std::pair<tam::types::common::FrenetPose, double> path_matching();
  void log_trajectory_and_constraint_point(
    tam::types::control::TrajectoryPoint actual_trajectory_point,
    tam::types::control::ControlConstraintPoint control_constraint_point);
  // Subscription callbacks
  void trajectory_callback(
    const tier4_planning_msgs::msg::Trajectory::ConstSharedPtr & traj,
    const tum_msgs::msg::TUMControlConstraints::ConstSharedPtr & constr);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
