// Copyright 2026 Fire Robot contributors. SPDX-License-Identifier: Apache-2.0
#include <memory>
#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "../src/retained_behavior_component_manager.hpp"

namespace
{
using Retainer = fire_robot_navigation::BehaviorLibraryRetainer;
using Manager = fire_robot_navigation::RetainedBehaviorComponentManager;

class ExposedManager : public Manager
{
public:
  using Manager::create_node_options;
};

TEST(BehaviorLibraryParameters, ConfiguredAliasesAndLastOverrideWin)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("behavior_plugins", std::vector<std::string>{"rotate", "pause", "again"}),
    rclcpp::Parameter("rotate.plugin", "old"),
    rclcpp::Parameter("rotate.plugin", "nav2_behaviors/Spin"),
    rclcpp::Parameter("pause.plugin", "nav2_behaviors/Wait"),
    rclcpp::Parameter("again.plugin", "nav2_behaviors/Spin")});
  EXPECT_EQ(Retainer::configured_types(options), (std::vector<std::string>{
      "nav2_behaviors/Spin", "nav2_behaviors/Wait", "nav2_behaviors/Spin"}));
}

TEST(BehaviorLibraryParameters, UpstreamDefaultsAndDefaultOverride)
{
  auto options = rclcpp::NodeOptions();
  EXPECT_EQ(Retainer::configured_types(options), (std::vector<std::string>{
      "nav2_behaviors/Spin", "nav2_behaviors/BackUp", "nav2_behaviors/DriveOnHeading",
      "nav2_behaviors/Wait"}));
  options.parameter_overrides({rclcpp::Parameter("spin.plugin", "custom/Spin")});
  EXPECT_EQ(Retainer::configured_types(options).at(0), "custom/Spin");
}

TEST(BehaviorLibraryParameters, EmptyListRequiresNoLibraries)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("behavior_plugins", std::vector<std::string>{})});
  EXPECT_TRUE(Retainer::configured_types(options).empty());
}

TEST(BehaviorLibraryParameters, MissingWrongTypeAndEmptyTypeAreRejected)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("behavior_plugins", std::vector<std::string>{"custom"})});
  EXPECT_ANY_THROW(Retainer::configured_types(options));
  options.parameter_overrides({rclcpp::Parameter("behavior_plugins", "spin")});
  EXPECT_ANY_THROW(Retainer::configured_types(options));
  options.parameter_overrides({
    rclcpp::Parameter("behavior_plugins", std::vector<std::string>{"spin"}),
    rclcpp::Parameter("spin.plugin", "")});
  EXPECT_ANY_THROW(Retainer::configured_types(options));
}

class BehaviorLoadOptions : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
  static auto request() -> std::shared_ptr<Manager::LoadNode::Request>
  {
    auto result = std::make_shared<Manager::LoadNode::Request>();
    result->package_name = "nav2_behaviors";
    result->plugin_name = "behavior_server::BehaviorServer";
    result->node_name = "renamed_behavior";
    result->node_namespace = "/qa_retention";
    result->remap_rules = {"cmd_vel:=/qa_retention/never_armed"};
    for (const auto & parameter : std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("behavior_plugins", std::vector<std::string>{"spin", "wait"}),
        rclcpp::Parameter("spin.plugin", "nav2_behaviors/Spin"),
        rclcpp::Parameter("wait.plugin", "nav2_behaviors/Wait")})
    {
      result->parameters.push_back(parameter.to_parameter_msg());
    }
    return result;
  }
};

TEST_F(BehaviorLoadOptions, DeduplicatesLibrariesAndPreservesRequest)
{
  auto manager = std::make_shared<ExposedManager>();
  const auto input = request();
  const auto original = *input;
  const auto options = manager->create_node_options(input);
  EXPECT_EQ(*input, original);
  EXPECT_EQ(options.parameter_overrides().size(), 3u);
  EXPECT_FALSE(options.use_global_arguments());
  EXPECT_EQ(options.arguments(), (std::vector<std::string>{
      "--ros-args", "-r", "cmd_vel:=/qa_retention/never_armed", "-r",
      "__node:=renamed_behavior", "-r", "__ns:=/qa_retention"}));
  EXPECT_EQ(manager->retained_behavior_library_count(), 2u);
  manager->create_node_options(input);
  EXPECT_EQ(manager->retained_behavior_library_count(), 2u);
}

TEST_F(BehaviorLoadOptions, OtherComponentsDoNotRetainBehaviorLibraries)
{
  auto manager = std::make_shared<ExposedManager>();
  auto input = request();
  input->package_name = "nav2_map_server";
  input->plugin_name = "nav2_map_server::MapServer";
  manager->create_node_options(input);
  EXPECT_EQ(manager->retained_behavior_library_count(), 0u);
}

TEST_F(BehaviorLoadOptions, UnknownPluginFailsBeforeNodeConstruction)
{
  auto manager = std::make_shared<ExposedManager>();
  auto input = request();
  input->parameters.back() = rclcpp::Parameter("wait.plugin", "missing/NoSuchBehavior").to_parameter_msg();
  EXPECT_THROW(manager->create_node_options(input), rclcpp_components::ComponentManagerException);
}
}  // namespace
