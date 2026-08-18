// Copyright 2024 Phillip Pitschi

#include "tracking_controller_node_cpp/tracking_controller_node.hpp"

#include "ros2_watchdog_cpp/timeout_value_provider.hpp"

TrackingControllerNode::TrackingControllerNode(
    const rclcpp::NodeOptions &options)
    : Node("TrackingController", "/core/control", options)
{
  tracking_controller_ = std::make_unique<tam::control::TrackingControllerPurePursuitCpp>();

  logger_composer_ = std::make_shared<tam::tsl::LoggerComposer>(
      std::vector<tam::tsl::LoggerAccessInterface::SharedPtr>{
          logger_, tracking_controller_->get_debug_out()});

  tsl_publisher_ = std::make_shared<tam::tsl::TSLPublisher>(this, logger_composer_);

  // Monitoring
  node_monitor_ = std::make_unique<tam::core::NodeMonitor>(this);
  topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);
  tam::core::TimeoutValueProvider timeout_values;

  // overwrite vehicle parameters
  vehicle_->load_params(tracking_controller_->get_param_handler().get());

  // Parameters
  callback_handle_ = tam::pmg::connect_param_manager_to_ros_cb(this, tracking_controller_->get_param_handler());
  // Declare controller parameters.
  tam::pmg::declare_ros_params_from_param_manager(this, tracking_controller_->get_param_handler().get());

  // Control-loop timer
  double time = this->get_parameter("tS").as_double();
  model_update_timer_ = tam::create_timer(
      this, std::chrono::microseconds(static_cast<uint64_t>(time * 1e6)),
      std::bind(&TrackingControllerNode::function_queue_callback, this));

  auto qos = tam::ros::get_qos();

  // Subscriptions
  sub_odometry_ = topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
      "/core/state/odometry", qos, std::bind(&TrackingControllerNode::odometry_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) odometry_received_ = false;
      },
      timeout_values.value_ms("StateEstimation"));
  sub_trajectory_.subscribe(
      this, "/core/planning/target_trajectory/trajectory", qos.get_rmw_qos_profile());
  sub_constraints_.subscribe(
      this, "/core/planning/target_trajectory/constraints", qos.get_rmw_qos_profile());

  // Trajectory/constraint synchronization
  sync_ = std::make_shared<message_filters::TimeSynchronizer<
      tier4_planning_msgs::msg::Trajectory, tum_msgs::msg::TUMControlConstraints>>(
      sub_trajectory_, sub_constraints_, 1);
  sync_->registerCallback(topic_watchdog_->timeout_callback(
      std::bind(&TrackingControllerNode::trajectory_callback, this, _1, _2),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) trajectory_received_ = false;
      },
      timeout_values.value_ms("SamplingPlanner")));

  // Publishers
  ctrl_cmd_pub_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/core/control/tracking_controller/control_request", qos);
  additional_esp_targets_pub_ = this->create_publisher<tum_msgs::msg::TUMAdditionalEspTargets>(
      "/core/control/optional/additional_esp_targets", qos);
}
void TrackingControllerNode::function_queue_callback()
{
  topic_watchdog_->check_timeouts();
  model_update_callback();
  if (odometry_received_ && trajectory_received_)
  {
    node_monitor_->set_error_lvl(node_monitor_key_, tam::types::ErrorLvl::OK);

    node_monitor_->set_message("Ok");

    node_monitor_->set_error_lvl("Ok", tam::types::ErrorLvl::OK);
    node_monitor_->set_status_code(0);
    node_monitor_key_ = "Ok";
    node_monitor_->update();
  } else {
    node_monitor_->set_error_lvl(node_monitor_key_, tam::types::ErrorLvl::OK);
    node_monitor_->set_message("Waiting for odometry and trajectory messages");
    node_monitor_->set_error_lvl("Error", tam::types::ErrorLvl::ERROR);
    node_monitor_->set_status_code(1);
    node_monitor_key_ = "Error";
    node_monitor_->update();
  }
}
std::pair<tam::types::common::FrenetPose, double> TrackingControllerNode::path_matching()
{
  if (!cosy_)
    return {tam::types::common::FrenetPose{0.0, 0.0, 0.0}, 0.0};

  std::tuple<Eigen::Vector3d, float> return_val =
      cosy_->convert_to_sn_and_get_idx_global(odometry_, std::numeric_limits<double>::max());
  auto path_matching_result = std::get<0>(return_val);
  matching_idx_ = std::clamp(
      std::get<1>(return_val), 0.0f, static_cast<float>(trajectory_buffer_.points.size() - 1.0f));

  logger_->log("Pathmatching_d", path_matching_result[1]);

  return {tam::types::common::FrenetPose(path_matching_result), matching_idx_};
}
void TrackingControllerNode::model_update_callback()
{
  std::chrono::steady_clock::time_point callback_start_time = std::chrono::steady_clock::now();

  if (!odometry_received_ || !trajectory_received_)
  {
    return;
  }

  cycle_count_ += 1;

  const auto [frenet_pose, matching_idx] = path_matching();
  tracking_controller_->set_path_matching_result(frenet_pose, static_cast<float>(matching_idx));

  tracking_controller_->step();

  builtin_interfaces::msg::Time stamp = get_clock()->now();

  // Publish ESC targets.
  auto additional_esp_targets = tracking_controller_->get_additional_esp_targets();
  tum_msgs::msg::TUMAdditionalEspTargets esp_targets_msg;
  esp_targets_msg.header.stamp = stamp;
  esp_targets_msg.ay_request_set = additional_esp_targets.ay_request_mps2.has_value();
  esp_targets_msg.ay_request = additional_esp_targets.ay_request_mps2.value_or(0.0);
  esp_targets_msg.yaw_rate_request_set = additional_esp_targets.yaw_rate_request_radps.has_value();
  esp_targets_msg.yaw_rate_request = additional_esp_targets.yaw_rate_request_radps.value_or(0.0);
  esp_targets_msg.slip_angle_request_set =
      additional_esp_targets.slip_angle_request_rad.has_value();
  esp_targets_msg.slip_angle_request = additional_esp_targets.slip_angle_request_rad.value_or(0.0);
  additional_esp_targets_pub_->publish(esp_targets_msg);

  // Publish the control request.
  autoware_auto_control_msgs::msg::AckermannControlCommand ctrl_out;
  ctrl_out.stamp = stamp;
  ctrl_out.lateral.stamp = stamp;
  ctrl_out.longitudinal.stamp = stamp;
  tam::types::control::LateralControlCommand lat = tracking_controller_->get_lateral_command();
  ctrl_out.lateral.steering_tire_angle = lat.steering_angle_tire_rad;
  ctrl_out.longitudinal.acceleration = tracking_controller_->get_longitudinal_command();
  ctrl_cmd_pub_->publish(ctrl_out);

  log_trajectory_and_constraint_point(
      tam::helpers::control::find_trajectory_point(trajectory_buffer_, matching_idx_),
      tam::helpers::control::find_constraint_point(constraints_buffer_, matching_idx_));

  // Log timing diagnostics.
  logger_->log(
      "timing/exec_time_timer_callback_us",
      std::chrono::duration_cast<std::chrono::microseconds>(
          (std::chrono::steady_clock::now() - callback_start_time))
          .count());
  logger_->log(
      "timing/time_since_last_callback_us",
      std::chrono::duration_cast<std::chrono::microseconds>((callback_start_time - last_call_time))
          .count());
  logger_->log("timing/cycle_count", cycle_count_);

  tsl_publisher_->trigger();
  last_call_time = callback_start_time;
}
tam::types::control::AdditionalTrajectoryInfos
TrackingControllerNode::prepare_additional_trajectory_infos(
    tam::types::control::Trajectory const &traj) const
{
  tam::types::control::AdditionalTrajectoryInfos add_info_input;
  add_info_input.points.reserve(traj.points.size());
  std::vector<double> x_traj(traj.points.size()), y_traj(traj.points.size()),
      z_traj(traj.points.size());
  std::transform(traj.points.begin(), traj.points.end(), x_traj.begin(), [](auto &p)
                 { return p.position_m.x; });
  std::transform(traj.points.begin(), traj.points.end(), y_traj.begin(), [](auto &p)
                 { return p.position_m.y; });
  std::transform(traj.points.begin(), traj.points.end(), z_traj.begin(), [](auto &p)
                 { return p.position_m.z; });
  std::vector<double> s_local_m =
      tam::helpers::geometry::create_s_coordinate_from_points(x_traj, y_traj, z_traj);

  std::vector<double> curvature;
  curvature.reserve(traj.points.size());
  for (size_t i = 0; i < traj.points.size(); i++)
  {
    if (traj.points[i].velocity_mps.x > 3.0)
    {
      double ay_hat = traj.points[i].acceleration_mps2.y -
                      tam::constants::g_earth * std::cos(traj.points[i].orientation_rad.y) *
                          std::sin(traj.points[i].orientation_rad.x);
      curvature.push_back(ay_hat / std::pow(traj.points[i].velocity_mps.x, 2));
    }
    else
    {
      if (i == 0)
      {
        curvature.push_back(tam::helpers::geometry::get_curvature_from_heading(
            traj.points[i].orientation_rad.z, traj.points[i + 1].orientation_rad.z, s_local_m[i],
            s_local_m[i + 1]));
      }
      else if (i == traj.points.size() - 1)
      {
        curvature.push_back(tam::helpers::geometry::get_curvature_from_heading(
            traj.points[i - 1].orientation_rad.z, traj.points[i].orientation_rad.z, s_local_m[i - 1],
            s_local_m[i]));
      }
      else
      {
        curvature.push_back(tam::helpers::geometry::get_curvature_from_heading(
            traj.points[i - 1].orientation_rad.z, traj.points[i].orientation_rad.z,
            traj.points[i + 1].orientation_rad.z, s_local_m[i - 1], s_local_m[i], s_local_m[i + 1]));
      }
    }
  }

  // Recompute arc length with curvature.
  s_local_m =
      tam::helpers::geometry::create_s_coordinate_from_points(x_traj, y_traj, z_traj, curvature);

  // Store arc length and curvature.
  std::transform(
      s_local_m.begin(), s_local_m.end(), curvature.begin(),
      std::back_inserter(add_info_input.points), [](const double s, const double kappa)
      {
      tam::types::control::AdditionalInfoPoint point;
      point.s_local_m = s;
      point.kappa_1pm = kappa;
      return point; });

  return add_info_input;
}
void TrackingControllerNode::trajectory_callback(
    const tier4_planning_msgs::msg::Trajectory::ConstSharedPtr &traj,
    const tum_msgs::msg::TUMControlConstraints::ConstSharedPtr &constr)
{
  tam::types::control::Trajectory traj_input;
  traj_input = tam::type::conversions::cpp::Trajectory_type_from_msg(traj);

  tam::types::control::ControlConstraints constr_input =
      tam::type_conversions::constraint_type_from_msg(*constr);

  if (
      traj_input.points.size() < 2 || constr_input.points.size() < 2 ||
      traj_input.points.size() != constr_input.points.size())
  {
    trajectory_received_ = false;
    RCLCPP_ERROR(
      get_logger(),
      "Ignoring trajectory: trajectory and constraints require at least two equally sized points.");
    return;
  }

  trajectory_received_ = true;
  trajectory_buffer_ = traj_input;
  tracking_controller_->set_target_trajectory(trajectory_buffer_);

  constraints_buffer_ = constr_input;
  tracking_controller_->set_constraints(constraints_buffer_);

  tam::types::control::AdditionalTrajectoryInfos add_info_input =
      prepare_additional_trajectory_infos(traj_input);

  tracking_controller_->set_additional_trajectory_info(add_info_input);

  if (!traj_input.points.empty())
  {
    cosy_ = tam::helpers::cosy::CurvilinearCosy::create(traj_input)->build();
  }
}
void TrackingControllerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!odometry_received_)
    odometry_received_ = true;
  odometry_ = tam::type::conversions::cpp::Odometry_type_from_msg(msg);
  tracking_controller_->set_feedback_odometry(odometry_);
}
void TrackingControllerNode::log_trajectory_and_constraint_point(
    tam::types::control::TrajectoryPoint actual_trajectory_point,
    tam::types::control::ControlConstraintPoint control_constraint_point)
{
  logger_->log("PathMatching_position_x_m_cpp", actual_trajectory_point.position_m.x);
  logger_->log("PathMatching_position_y_m_cpp", actual_trajectory_point.position_m.y);
  logger_->log("PathMatching_position_z_m_cpp", actual_trajectory_point.position_m.z);
  logger_->log("PathMatching_orientation_x_rad_cpp", actual_trajectory_point.orientation_rad.x);
  logger_->log("PathMatching_orientation_y_rad_cpp", actual_trajectory_point.orientation_rad.y);
  logger_->log("PathMatching_orientation_z_rad_cpp", actual_trajectory_point.orientation_rad.z);
  logger_->log("PathMatching_velocity_x_mps_cpp", actual_trajectory_point.velocity_mps.x);
  logger_->log("PathMatching_velocity_y_mps_cpp", actual_trajectory_point.velocity_mps.y);
  logger_->log("PathMatching_velocity_z_mps_cpp", actual_trajectory_point.velocity_mps.z);
  logger_->log("PathMatching_acceleration_x_mps2_cpp", actual_trajectory_point.acceleration_mps2.x);
  logger_->log("PathMatching_acceleration_y_mps2_cpp", actual_trajectory_point.acceleration_mps2.y);
  logger_->log("PathMatching_acceleration_z_mps2_cpp", actual_trajectory_point.acceleration_mps2.z);
  logger_->log(
      "PathMatching_angular_velocity_x_radps_cpp", actual_trajectory_point.angular_velocity_radps.x);
  logger_->log(
      "PathMatching_angular_velocity_y_radps_cpp", actual_trajectory_point.angular_velocity_radps.y);
  logger_->log(
      "PathMatching_angular_velocity_z_radps_cpp", actual_trajectory_point.angular_velocity_radps.z);
  logger_->log(
      "PathMatching_angular_acceleration_x_radps2_cpp",
      actual_trajectory_point.angular_acceleration_radps2.x);
  logger_->log(
      "PathMatching_angular_acceleration_y_radps2_cpp",
      actual_trajectory_point.angular_acceleration_radps2.y);
  logger_->log(
      "PathMatching_angular_acceleration_z_radps2_cpp",
      actual_trajectory_point.angular_acceleration_radps2.z);
  logger_->log(
      "ConstraintPoint_ax_max_mps2_cpp", control_constraint_point.a_x_max_mps2.x);
  logger_->log(
      "ConstraintPoint_ax_min_mps2_cpp", control_constraint_point.a_x_min_mps2.x);
  logger_->log(
      "ConstraintPoint_ay_max_mps2_cpp", control_constraint_point.a_y_max_mps2.y);
  logger_->log(
      "ConstraintPoint_ay_min_mps2_cpp", control_constraint_point.a_y_min_mps2.y);
  logger_->log(
      "ConstraintPoint_tube_max_m_cpp", control_constraint_point.lateral_error_max_m);
  logger_->log(
      "ConstraintPoint_tube_min_m_cpp", control_constraint_point.lateral_error_min_m);
  logger_->log(
      "ConstraintPoint_shape_factor.0", control_constraint_point.shape_factor[0]);
  logger_->log(
      "ConstraintPoint_shape_factor.1", control_constraint_point.shape_factor[1]);
  logger_->log(
      "ConstraintPoint_shape_factor.2", control_constraint_point.shape_factor[2]);
  logger_->log(
      "ConstraintPoint_shape_factor.3", control_constraint_point.shape_factor[3]);
}
