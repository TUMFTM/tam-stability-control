// Copyright 2026 Phillip Pitschi
#pragma once

// Standard library
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"

// Messages
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "tum_msgs/msg/tum_additional_esp_targets.hpp"
#include "tum_msgs/msg/tum_bool_stamped.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_msgs/msg/tum_longitudinal_cmd.hpp"
#include "tum_msgs/msg/tum_int8_stamped.hpp"

// Project types
#include "tum_types_cpp/control.hpp"

// Stability control components
#include "stability_control_tam_cpp/slip_control.hpp"
#include "stability_control_tam_cpp/slip_calculation.hpp"
#include "stability_control_tam_cpp/esc.hpp"
#include "stability_control_tam_cpp/countersteer.hpp"

// Parameters
#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"

// Diagnostics
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"

// Type conversion
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

// Monitoring
#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/timeout_value_provider.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"

using std::placeholders::_1;

class StabilityControllerNode : public rclcpp::Node
{
public:
    StabilityControllerNode(const rclcpp::NodeOptions &options);

private:
    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_acceleration_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_;
    rclcpp::Subscription<tum_msgs::msg::TUMInt8Stamped>::SharedPtr sub_gear_request_;
    rclcpp::Subscription<tum_msgs::msg::TUMInt8Stamped>::SharedPtr sub_gear_report_;
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_ctrl_cmd_;
    rclcpp::Subscription<tum_msgs::msg::TUMLongitudinalCmd>::SharedPtr sub_longitudinal_cmd_;
    rclcpp::Subscription<tum_msgs::msg::TUMAdditionalEspTargets>::SharedPtr sub_additional_esc_targets_;
    rclcpp::Subscription<tum_msgs::msg::TUMFloat32Stamped>::SharedPtr sub_omega_engine_;
    rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_wheelspeed_;
    rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_brake_pressure_;
    rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_brake_temperature_;

    // Publishers
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
        ctrl_cmd_pub_{};
    rclcpp::Publisher<tum_msgs::msg::TUMLongitudinalCmd>::SharedPtr longitudinal_cmd_pub_{};
    rclcpp::Publisher<tum_msgs::msg::TUMBoolStamped>::SharedPtr slip_control_active_pub_{};

    // Node monitor
    tam::core::NodeMonitor::UniquePtr node_monitor_ =
        std::make_unique<tam::core::NodeMonitor>(this);
    tam::core::TopicWatchdog::UniquePtr topic_watchdog_;
    std::string node_monitor_key_{"startup"};

    // Timer
    rclcpp::TimerBase::SharedPtr model_update_timer_;

    // Controllers
    std::unique_ptr<tam::control::SlipController> slip_control_ =
        std::make_unique<tam::control::SlipController>();
    std::unique_ptr<tam::control::SlipCalculation> slip_calculation_ =
        std::make_unique<tam::control::SlipCalculation>();
    std::unique_ptr<tam::control::ESC> esc_ = std::make_unique<tam::control::ESC>();
    std::unique_ptr<tam::control::CountersteerSystem> countersteer_system_ =
        std::make_unique<tam::control::CountersteerSystem>();

    tam::pmg::ParamValueManager::SharedPtr param_manager_ =
        std::make_shared<tam::pmg::ParamValueManager>();
    std::size_t previous_param_state_hash_{};

    // Logging
    tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
    tam::tsl::LoggerComposer::SharedPtr logger_composer_ = std::make_shared<tam::tsl::LoggerComposer>(
        std::vector<std::pair<std::string, tam::tsl::LoggerAccessInterface::SharedPtr>>{
            std::make_pair("", logger_),
            std::make_pair("slip_calculation/", slip_calculation_->get_debug_out()),
            std::make_pair("slip_control/", slip_control_->get_debug_out()),
            std::make_pair("esc/", esc_->get_debug_out()),
            std::make_pair("countersteer/", countersteer_system_->get_debug_out())});
    std::shared_ptr<tam::tsl::TSLPublisher> tsl_publisher_ =
        std::make_shared<tam::tsl::TSLPublisher>(this, logger_composer_);

    // vehicle_handler
    std::unique_ptr<tam::common::VehicleHandler> vehicle_ = tam::common::VehicleHandler::from_pkg_config();

    // Parameters
    tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer_ =
        std::make_shared<tam::pmg::ParamManagerComposer>(std::vector<tam::pmg::MgmtInterface::SharedPtr>{
            slip_calculation_->get_param_handler(),
            slip_control_->get_param_handler(),
            esc_->get_param_handler(),
            countersteer_system_->get_param_handler(),
            param_manager_});
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Timing diagnostics
    mutable std::chrono::steady_clock::time_point last_call_time;
    mutable uint64_t cycle_count_ = 0;
    // Input availability
    bool odometry_received_{false};
    bool acceleration_received_{false};
    bool steering_received_{false};
    bool gear_request_received_{false};
    bool gear_report_received_{false};
    bool ctrl_cmd_received_{false};
    bool longitudinal_cmd_received_{false};
    bool wheelspeed_received_{false};
    bool brake_pressure_received_{false};

    tam::types::control::DriveTrainFeedback drivetrain_feedback_{};
    tam::types::control::Odometry odometry_{};
    tam::types::control::AccelerationwithCovariances acceleration_{};
    tam::types::control::AutowareSteeringReport steering_report_{};
    autoware_auto_control_msgs::msg::AckermannControlCommand ctrl_cmd_input_{};
    tum_msgs::msg::TUMLongitudinalCmd longitudinal_cmd_input_{};
    tam::types::common::DataPerWheel<double> brake_temperature_degree_{};
    bool brake_temperatures_ok_{false};
    bool brake_friction_map_valid_{false};
    struct
    {
      double brake_friction_default;
      std::vector<double> brake_friction_temperature;
      std::vector<double> brake_friction_coeff;
    } p_;
    // Internal methods
    void function_queue_callback();
    void model_update_callback();
    void declare_and_update_parameters();
    // Subscription callbacks
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
    void steering_callback(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
    void gear_request_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg);
    void gear_report_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg);
    void control_command_callback(
        const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
    void longitudinal_cmd_callback(const tum_msgs::msg::TUMLongitudinalCmd::SharedPtr msg);
    void additional_esc_targets_callback(const tum_msgs::msg::TUMAdditionalEspTargets::SharedPtr msg);
    void omega_engine_callback(const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg);
    void wheelspeed_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
    void brake_pressure_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
    void brake_temperature_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
};
