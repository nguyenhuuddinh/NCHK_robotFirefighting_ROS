// Copyright 2026 Fire Robot contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef FIRE_ROBOT_NAVIGATION__ACTION_STATUS_REFRESH_HPP_
#define FIRE_ROBOT_NAVIGATION__ACTION_STATUS_REFRESH_HPP_

#include <rclcpp_action/server.hpp>

namespace fire_robot_navigation
{
// Humble compatibility boundary, matching the protected method in server.hpp.
// This abstract access class is NEVER instantiated or cast onto another object.
// An inherited member's pointer retains the declaring ServerBase type; calling
// it on a real ServerBase uses the original implementation and internal mutex.
// No cached GoalStatusArray, new publisher, fake goal, or memory-layout access.
struct StatusRefreshAccess : rclcpp_action::ServerBase
{
  static auto method() {return &StatusRefreshAccess::publish_status;}
};

inline void refresh_action_status(rclcpp_action::ServerBase & server)
{
  (server.*StatusRefreshAccess::method())();
}
}  // namespace fire_robot_navigation
#endif  // FIRE_ROBOT_NAVIGATION__ACTION_STATUS_REFRESH_HPP_
