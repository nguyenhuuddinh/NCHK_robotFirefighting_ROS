// Copyright 2026 Fire Robot contributors
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "fire_robot_navigation/action_status_refresh.hpp"

struct TestContext
{
  TestContext() {int argc = 0; rclcpp::init(argc, nullptr);}
  ~TestContext() {rclcpp::shutdown();}
};

TEST(ActionStatusRefresh, ReadsLiveServerWithOriginalPublisherAndNoFakeGoals)
{
  using Action = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using namespace std::chrono_literals;
  TestContext context;  // Also tears down correctly after an ASSERT return.
  auto node = std::make_shared<rclcpp::Node>("m3_status_source_test");
  std::shared_ptr<Handle> accepted;
  size_t admissions = 0;
  auto server = rclcpp_action::create_server<Action>(
    node, "/m3_source_test/navigate_to_pose",
    [&](const auto &, const auto &) {++admissions; return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;},
    [](const auto &) {return rclcpp_action::CancelResponse::REJECT;},
    [&](auto handle) {accepted = handle;});
  std::vector<action_msgs::msg::GoalStatusArray> snapshots;
  std::vector<int64_t> stamps;
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto sub = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "/m3_source_test/navigate_to_pose/_action/status", qos,
    [&](action_msgs::msg::GoalStatusArray::SharedPtr msg, const rclcpp::MessageInfo & info) {
      snapshots.push_back(*msg);
      stamps.push_back(info.get_rmw_message_info().source_timestamp);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto wait = [&](auto predicate) {
      const auto until = std::chrono::steady_clock::now() + 3s;
      while (!predicate() && std::chrono::steady_clock::now() < until) {
        executor.spin_some();
        std::this_thread::sleep_for(5ms);
      }
      return predicate();
    };
  ASSERT_TRUE(wait([&]() {return node->count_subscribers(
      "/m3_source_test/navigate_to_pose/_action/status") == 1;}));
  fire_robot_navigation::refresh_action_status(*server);
  ASSERT_TRUE(wait([&]() {return !snapshots.empty();}));
  EXPECT_TRUE(snapshots.back().status_list.empty());
  EXPECT_EQ(admissions, 0u);
  auto client = rclcpp_action::create_client<Action>(node, "/m3_source_test/navigate_to_pose");
  ASSERT_TRUE(client->wait_for_action_server(3s));
  auto future = client->async_send_goal(Action::Goal());
  ASSERT_TRUE(wait([&]() {return accepted != nullptr && future.wait_for(0s) == std::future_status::ready;}));
  ASSERT_EQ(future.wait_for(0s), std::future_status::ready);
  auto client_goal = future.get();
  ASSERT_NE(client_goal, nullptr);
  const auto uuid = client_goal->get_goal_id();
  auto count = snapshots.size();
  fire_robot_navigation::refresh_action_status(*server);
  ASSERT_TRUE(wait([&]() {return snapshots.size() > count;}));
  ASSERT_EQ(snapshots.back().status_list.size(), 1u);
  EXPECT_EQ(snapshots.back().status_list[0].goal_info.goal_id.uuid, uuid);
  EXPECT_EQ(snapshots.back().status_list[0].status, 1);  // Actual ACCEPTED.
  accepted->execute();
  accepted->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(wait([&]() {return !snapshots.back().status_list.empty() &&
      snapshots.back().status_list[0].status == 4;}));
  const auto terminal_stamp = stamps.back();
  std::this_thread::sleep_for(10ms);
  count = snapshots.size();
  fire_robot_navigation::refresh_action_status(*server);
  ASSERT_TRUE(wait([&]() {return snapshots.size() > count;}));
  EXPECT_GT(stamps.back(), terminal_stamp);
  EXPECT_EQ(snapshots.back().status_list[0].goal_info.goal_id.uuid, uuid);
  EXPECT_EQ(snapshots.back().status_list[0].status, 4);
  EXPECT_EQ(admissions, 1u);
  auto publishers = node->get_publishers_info_by_topic(
    "/m3_source_test/navigate_to_pose/_action/status");
  ASSERT_EQ(publishers.size(), 1u);
  EXPECT_EQ(publishers[0].node_name(), "m3_status_source_test");
  executor.remove_node(node);
}
