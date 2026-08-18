// Copyright 2026 Phillip Pitschi
#include "tracking_controller_node_cpp/tracking_controller_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Register the composable node.
using TrackingControllerNodeGeneric = TrackingControllerNode;
namespace tracking_controller_node_cpp
{
struct TrackingControllerNode : public TrackingControllerNodeGeneric
{
  explicit TrackingControllerNode(const rclcpp::NodeOptions & options)
  : TrackingControllerNodeGeneric(options)
  {
  }
};
}  // namespace tracking_controller_node_cpp
RCLCPP_COMPONENTS_REGISTER_NODE(tracking_controller_node_cpp::TrackingControllerNode)
