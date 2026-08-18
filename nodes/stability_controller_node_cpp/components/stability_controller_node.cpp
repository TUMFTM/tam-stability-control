// Copyright 2026 Phillip Pitschi
#include "stability_controller_node_cpp/stability_controller_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Register the composable node.
using StabilityControllerNodeGeneric = StabilityControllerNode;
namespace stability_controller_node_cpp
{
struct StabilityControllerNode : public StabilityControllerNodeGeneric
{
  explicit StabilityControllerNode(const rclcpp::NodeOptions & options)
  : StabilityControllerNodeGeneric(options)
  {
  }
};
}  // namespace stability_controller_node_cpp
RCLCPP_COMPONENTS_REGISTER_NODE(stability_controller_node_cpp::StabilityControllerNode)
