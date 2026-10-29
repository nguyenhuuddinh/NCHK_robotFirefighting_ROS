// Copyright 2026 Fire Robot contributors. SPDX-License-Identifier: Apache-2.0
#ifndef FIRE_ROBOT_NAVIGATION__ACTION_STATUS_REFRESH_HPP_
#define FIRE_ROBOT_NAVIGATION__ACTION_STATUS_REFRESH_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/server.hpp"

namespace fire_robot_navigation
{
// Humble exposes status publication to ServerBase subclasses. Obtain its
// base-member pointer legally through a subclass; never downcast an existing
// server to this type, touch private state, or construct another publisher.
// ServerBase::publish_status locks its action-server mutex and publishes the
// actual goal table through its existing status endpoint (same GID and QoS).
class ActionStatusAccess : public rclcpp_action::ServerBase
{
public:
  static void refresh(rclcpp_action::ServerBase & server)
  {
    auto publish = &ActionStatusAccess::publish_status;
    (server.*publish)();
  }
};

inline size_t refresh_action_status(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node)
{
  std::vector<std::shared_ptr<rclcpp_action::ServerBase>> servers;
  node->for_each_callback_group([&servers](const rclcpp::CallbackGroup::SharedPtr & group) {
      group->find_waitable_ptrs_if([&servers](const rclcpp::Waitable::SharedPtr & waitable) {
          auto server = std::dynamic_pointer_cast<rclcpp_action::ServerBase>(waitable);
          if (server) {servers.push_back(server);}
          return false;
        });
    });
  // Release node/group discovery locks before entering the action-server lock.
  for (const auto & server : servers) {ActionStatusAccess::refresh(*server);}
  return servers.size();
}
}  // namespace fire_robot_navigation
#endif
