// Copyright 2026 Fire Robot contributors.
// SPDX-License-Identifier: Apache-2.0
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "retained_navigation_component_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Same default execution model as Humble component_container_isolated:
  // one manager executor, one SingleThreadedExecutor per component.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<fire_robot_navigation::RetainedNavigationComponentManager>(executor);
  rclcpp::TimerBase::SharedPtr status_timer;
  if (manager->declare_parameter<bool>("refresh_bt_action_status", false)) {
    status_timer = manager->create_wall_timer(std::chrono::milliseconds(200),
      [manager]() {manager->refresh_bt_action_status();});
    RCLCPP_INFO(manager->get_logger(), "Nav2 actual action status refresh enabled at 5 Hz");
  }
  executor->add_node(manager);
  executor->spin();
  rclcpp::shutdown();
  status_timer.reset();
  // manager is destroyed before executor; join its component threads.
  // No changes to signal handling or per-component scheduling.
  return 0;
}
