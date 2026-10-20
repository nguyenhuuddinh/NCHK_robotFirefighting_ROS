// Copyright 2026 Fire Robot contributors.
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include "../src/isolated_component_manager.hpp"

namespace
{
std::atomic<unsigned> entered{0};
std::atomic<unsigned> returned{0};
class ObservedExecutor : public rclcpp::executors::SingleThreadedExecutor
{
public:
  void spin() override
  {
    ++entered;
    rclcpp::executors::SingleThreadedExecutor::spin();
    ++returned;
  }
};

class Manager : public fire_robot_navigation::ComponentManagerIsolated<ObservedExecutor>
{
public:
  void attach(uint64_t id, std::shared_ptr<rclcpp::Node> node)
  {
    node_wrappers_.emplace(id, rclcpp_components::NodeInstanceWrapper(node,
      [](const std::shared_ptr<void> & ptr) {
        return std::static_pointer_cast<rclcpp::Node>(ptr)->get_node_base_interface();
      }));
    add_node_to_executor(id);
  }
  void unload(uint64_t id)
  {
    remove_node_from_executor(id);
    node_wrappers_.erase(id);
  }
};

bool await(const std::function<bool()> & ready)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!ready() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return ready();
}

class IsolatedShutdown : public ::testing::Test
{
protected:
  void SetUp() override
  {
    entered = 0;
    returned = 0;
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(IsolatedShutdown, EmptyManager)
{
  auto manager = std::make_shared<Manager>();
  rclcpp::shutdown();
  manager.reset();
}

TEST_F(IsolatedShutdown, ShutdownAfterAllExecutorsAlreadyReturned)
{
  auto manager = std::make_shared<Manager>();
  for (unsigned i = 1; i <= 3; ++i) {
    manager->attach(i, std::make_shared<rclcpp::Node>("shutdown_regression_" + std::to_string(i)));
  }
  ASSERT_TRUE(await([] {return entered.load() == 3;}));
  rclcpp::shutdown();
  ASSERT_TRUE(await([] {return returned.load() == 3;}));
  // The old unconditional is_spinning wait hangs here. CTest's process timeout
  // bounds the regression; no detached destructor thread hides a failure.
  manager.reset();
}

TEST_F(IsolatedShutdown, ShutdownImmediatelyAfterExecutorCreation)
{
  auto manager = std::make_shared<Manager>();
  manager->attach(1, std::make_shared<rclcpp::Node>("shutdown_startup_race"));
  rclcpp::shutdown();
  manager.reset();
  EXPECT_EQ(returned.load(), 1u);
}

TEST_F(IsolatedShutdown, LiveExecutorUnloadsAndRemainingExecutorStillRuns)
{
  auto manager = std::make_shared<Manager>();
  auto first = std::make_shared<rclcpp::Node>("shutdown_unload_first");
  auto second = std::make_shared<rclcpp::Node>("shutdown_unload_second");
  std::atomic<unsigned> callbacks{0};
  auto timer = second->create_wall_timer(std::chrono::milliseconds(5), [&] {++callbacks;});
  manager->attach(1, first);
  manager->attach(2, second);
  ASSERT_TRUE(await([&] {return callbacks.load() > 0;}));
  manager->unload(1);
  EXPECT_EQ(returned.load(), 1u);
  const auto before = callbacks.load();
  EXPECT_TRUE(await([&] {return callbacks.load() > before;}));
  manager->unload(2);
  EXPECT_EQ(returned.load(), 2u);
}
}  // namespace
