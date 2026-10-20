// Copyright 2026 Fire Robot contributors
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include <nav2_bt_navigator/bt_navigator.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include "fire_robot_navigation/action_status_refresh.hpp"

namespace fire_robot_navigation
{
class SnapshotNavigator : public nav2_bt_navigator::BtNavigator
{
  using PoseServer = rclcpp_action::Server<nav2_msgs::action::NavigateToPose>;
  using PosesServer = rclcpp_action::Server<nav2_msgs::action::NavigateThroughPoses>;
  std::vector<std::weak_ptr<rclcpp_action::ServerBase>> servers_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    const auto result = BtNavigator::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {return result;}
    servers_.clear();
    size_t pose_count = 0, poses_count = 0;
    get_node_base_interface()->for_each_callback_group(
      [&](rclcpp::CallbackGroup::SharedPtr group) {
        group->find_waitable_ptrs_if([&](rclcpp::Waitable::SharedPtr waitable) {
          if (auto server = std::dynamic_pointer_cast<PoseServer>(waitable)) {
            servers_.push_back(server);
            ++pose_count;
          } else if (auto server = std::dynamic_pointer_cast<PosesServer>(waitable)) {
            servers_.push_back(server);
            ++poses_count;
          }
          return false;  // Visit all waitables once, during configure only.
        });
      });
    if (pose_count != 1 || poses_count != 1) {
      RCLCPP_ERROR(get_logger(), "Snapshot adapter needs exactly one server of each Humble Nav2 type");
      servers_.clear();
      return nav2_util::CallbackReturn::FAILURE;
    }
    return result;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    const auto result = BtNavigator::on_activate(state);
    if (result == nav2_util::CallbackReturn::SUCCESS) {
      // A fresh SOURCE snapshot, not periodic publication of a stored message.
      // Fixed 100ms keeps the 350ms consumer limit; no configurable weaker TTL.
      refresh_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        for (auto & weak : servers_) {
          auto server = weak.lock();
          if (!server) {throw std::runtime_error("Nav2 snapshot action server disappeared");}
          refresh_action_status(*server);
        }
      });
    }
    return result;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    refresh_timer_.reset();
    return BtNavigator::on_deactivate(state);
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    refresh_timer_.reset();
    servers_.clear();
    return BtNavigator::on_cleanup(state);
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    refresh_timer_.reset();
    servers_.clear();
    return BtNavigator::on_shutdown(state);
  }
};
}  // namespace fire_robot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fire_robot_navigation::SnapshotNavigator>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
