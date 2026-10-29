// Copyright 2026 Fire Robot contributors. SPDX-License-Identifier: Apache-2.0
// Isolated software action server. No Nav2 instance, motor or actuator topics.
#include <chrono>
#include <thread>
#include <gtest/gtest.h>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "../src/action_status_refresh.hpp"

using namespace std::chrono_literals;
using Action = nav2_msgs::action::NavigateToPose;

class StatusRefresh : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.set_domain_id(201);  // Test-only context; project ROS_DOMAIN_ID is untouched.
    context->init(0, nullptr, init);
    rclcpp::NodeOptions opts;
    opts.context(context);
    node = std::make_shared<rclcpp::Node>("fake_bt_navigator", "/r5_status_qa", opts);
    rclcpp::ExecutorOptions exec_opts;
    exec_opts.context = context;
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_opts);
    executor->add_node(node);
    server = rclcpp_action::create_server<Action>(node, "/r5_status_qa/navigate_to_pose",
      [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto accepted) {handle = accepted;});
    subscription = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/r5_status_qa/navigate_to_pose/_action/status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::SharedPtr msg) {last = msg;});
    client = rclcpp_action::create_client<Action>(node, "/r5_status_qa/navigate_to_pose");
  }

  template<typename Predicate> bool until(Predicate ready)
  {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < end) {
      executor->spin_some();
      if (ready()) {return true;}
      std::this_thread::sleep_for(10ms);
    }
    return ready();
  }

  void TearDown() override
  {
    if (executor && node) {executor->remove_node(node);}
    client.reset();subscription.reset();server.reset();handle.reset();
    node.reset();executor.reset();
    if (context && context->is_valid()) {context->shutdown("isolated status QA complete");}
  }

  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp_action::Client<Action>::SharedPtr client;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription;
  action_msgs::msg::GoalStatusArray::SharedPtr last;
};

TEST_F(StatusRefresh, FreshServerPublishesActualEmptyTableUsingSingleExistingEndpoint)
{
  ASSERT_TRUE(until([this]() {
    fire_robot_navigation::refresh_action_status(node->get_node_base_interface());
    return last != nullptr;
  }));
  EXPECT_TRUE(last->status_list.empty());
  const auto topic = "/r5_status_qa/navigate_to_pose/_action/status";
  auto before = node->get_publishers_info_by_topic(topic);
  ASSERT_EQ(1u, before.size());
  EXPECT_EQ(1u, fire_robot_navigation::refresh_action_status(node->get_node_base_interface()));
  auto after = node->get_publishers_info_by_topic(topic);
  ASSERT_EQ(1u, after.size());
  EXPECT_EQ(before[0].endpoint_gid(), after[0].endpoint_gid());
  server.reset();
  EXPECT_EQ(0u, fire_robot_navigation::refresh_action_status(node->get_node_base_interface()));
}

TEST_F(StatusRefresh, RefreshKeepsRealActiveAndCanceledUUIDInsteadOfInventingNoGoal)
{
  ASSERT_TRUE(client->wait_for_action_server(2s));
  auto sent = client->async_send_goal(Action::Goal());
  ASSERT_TRUE(until([&sent, this]() {
    return sent.wait_for(0s) == std::future_status::ready && handle != nullptr;
  }));
  auto accepted = sent.get();
  ASSERT_NE(nullptr, accepted);
  const auto uuid = accepted->get_goal_id();
  last.reset();
  ASSERT_TRUE(until([this]() {
    fire_robot_navigation::refresh_action_status(node->get_node_base_interface());
    return last && last->status_list.size() == 1 && last->status_list[0].status == 2;
  }));
  EXPECT_EQ(uuid, last->status_list[0].goal_info.goal_id.uuid);
  auto canceled = client->async_cancel_goal(accepted);
  ASSERT_TRUE(until([&canceled]() {return canceled.wait_for(0s) == std::future_status::ready;}));
  ASSERT_TRUE(handle->is_canceling());
  handle->canceled(std::make_shared<Action::Result>());
  last.reset();
  ASSERT_TRUE(until([this]() {
    fire_robot_navigation::refresh_action_status(node->get_node_base_interface());
    return last && last->status_list.size() == 1 && last->status_list[0].status == 5;
  }));
  EXPECT_EQ(uuid, last->status_list[0].goal_info.goal_id.uuid);
}
