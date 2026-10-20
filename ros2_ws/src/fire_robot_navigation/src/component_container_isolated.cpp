// Copyright 2026 Fire Robot contributors.
// SPDX-License-Identifier: Apache-2.0
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "isolated_component_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Same default execution model as Humble component_container_isolated:
  // one manager executor, one SingleThreadedExecutor per component.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<fire_robot_navigation::ComponentManagerIsolated<>>(executor);
  executor->add_node(manager);
  executor->spin();
  rclcpp::shutdown();
  // manager is destroyed before executor; join its component threads.
  // No changes to signal handling or per-component scheduling.
  return 0;
}
