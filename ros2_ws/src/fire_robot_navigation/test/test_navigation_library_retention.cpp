// Copyright 2026 Fire Robot contributors. SPDX-License-Identifier: Apache-2.0
#include <cstdlib>
#include <fstream>
#include "gtest/gtest.h"
#include "../src/retained_navigation_component_manager.hpp"
using Retainer = fire_robot_navigation::NavigationLibraryRetainer;

TEST(NavigationRetention, DefaultsAndExplicitEmptyLists)
{
  EXPECT_EQ(Retainer::configured_types({}, "plugins", {"a"}, {"type/A"}),
    std::vector<std::string>{"type/A"});
  Retainer::Parameters p;
  p["plugins"] = rclcpp::Parameter("plugins", std::vector<std::string>{});
  EXPECT_TRUE(Retainer::configured_types(p, "plugins", {"a"}, {"type/A"}).empty());
  EXPECT_TRUE(Retainer::configured_types({}, "filters", {}, {}).empty());
}

TEST(NavigationRetention, CustomAliasesResolveTheirOwnTypes)
{
  Retainer::Parameters p;
  p["plugins"] = rclcpp::Parameter("plugins", std::vector<std::string>{"custom"});
  p["custom.plugin"] = rclcpp::Parameter("custom.plugin", "external/Custom");
  EXPECT_EQ(Retainer::configured_types(p, "plugins", {"a"}, {"type/A"}),
    std::vector<std::string>{"external/Custom"});
}

TEST(NavigationRetention, MissingWrongOrEmptyTypesFailClosed)
{
  Retainer::Parameters p;
  p["plugins"] = rclcpp::Parameter("plugins", std::vector<std::string>{"custom"});
  EXPECT_ANY_THROW(Retainer::configured_types(p, "plugins", {"a"}, {"type/A"}));
  p["custom.plugin"] = rclcpp::Parameter("custom.plugin", "");
  EXPECT_ANY_THROW(Retainer::configured_types(p, "plugins", {}, {}));
  p["plugins"] = rclcpp::Parameter("plugins", "custom");
  EXPECT_ANY_THROW(Retainer::configured_types(p, "plugins", {}, {}));
}

TEST(NavigationRetention, GlobalYamlMatchesNamespaceAndLastFileOverride)
{
  const auto directory = std::getenv("CPP_TEST_TMP");
  ASSERT_NE(directory, nullptr);
  const std::string a = std::string(directory) + "/parameter_a.yaml";
  const std::string b = std::string(directory) + "/parameter_b.yaml";
  std::ofstream(a) << "/**:\n  ros__parameters:\n    marker: common\n"
    "/qa/local_costmap/local_costmap:\n  ros__parameters:\n    plugins: [custom]\n"
    "    custom.plugin: external/Old\n";
  std::ofstream(b) << "/qa/local_costmap/local_costmap:\n  ros__parameters:\n"
    "    custom.plugin: external/New\n";
  const char * args[]{"test", "--ros-args", "--params-file", a.c_str(), "--params-file", b.c_str()};
  auto context = std::make_shared<rclcpp::Context>();
  context->init(6, args);
  const auto selected = Retainer::global_parameters(context, "/qa/local_costmap/local_costmap");
  EXPECT_EQ(selected.at("custom.plugin").as_string(), "external/New");
  EXPECT_EQ(selected.at("marker").as_string(), "common");
  const auto other = Retainer::global_parameters(context, "/elsewhere/local_costmap/local_costmap");
  EXPECT_EQ(other.at("marker").as_string(), "common");
  EXPECT_EQ(other.count("custom.plugin"), 0u);
  context->shutdown("test done");
}

TEST(NavigationRetention, NoGlobalArgumentsUsesNav2Defaults)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  EXPECT_TRUE(Retainer::global_parameters(context, "/local_costmap/local_costmap").empty());
  context->shutdown("test done");
}
