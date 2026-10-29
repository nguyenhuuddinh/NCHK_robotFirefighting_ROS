// Copyright 2026 Fire Robot contributors.
// SPDX-License-Identifier: Apache-2.0
#ifndef FIRE_ROBOT_NAVIGATION__RETAINED_BEHAVIOR_COMPONENT_MANAGER_HPP_
#define FIRE_ROBOT_NAVIGATION__RETAINED_BEHAVIOR_COMPONENT_MANAGER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include "nav2_core/behavior.hpp"
#include "pluginlib/class_loader.hpp"
#include "isolated_component_manager.hpp"

namespace fire_robot_navigation
{
// Humble BehaviorServer owns its plugin loader in the derived node, while
// LifecycleNode keeps weak publisher control blocks in the base node. The
// plugin's control-block vtable must remain mapped until the entire node dies.
class BehaviorLibraryRetainer
{
public:
  static std::vector<std::string> configured_types(const rclcpp::NodeOptions & options)
  {
    std::map<std::string, rclcpp::Parameter> parameters;
    for (const auto & parameter : options.parameter_overrides()) {
      parameters[parameter.get_name()] = parameter;  // Same last override wins.
    }
    // Match installed Nav2 1.1.x BehaviorServer defaults when no list is supplied.
    const std::vector<std::string> defaults{"spin", "backup", "drive_on_heading", "wait"};
    const std::map<std::string, std::string> default_types{
      {"spin", "nav2_behaviors/Spin"}, {"backup", "nav2_behaviors/BackUp"},
      {"drive_on_heading", "nav2_behaviors/DriveOnHeading"}, {"wait", "nav2_behaviors/Wait"}};
    const auto list = parameters.find("behavior_plugins");
    const auto ids = list == parameters.end() ? defaults : list->second.as_string_array();
    std::vector<std::string> result;
    for (const auto & id : ids) {
      const auto found = parameters.find(id + ".plugin");
      if (found != parameters.end()) {
        result.push_back(found->second.as_string());
      } else if (ids == defaults) {
        result.push_back(default_types.at(id));
      } else {
        throw rclcpp_components::ComponentManagerException(
                "Behavior library retention: missing parameter " + id + ".plugin");
      }
      if (result.back().empty()) {
        throw rclcpp_components::ComponentManagerException(
                "Behavior library retention: empty plugin type for " + id);
      }
    }
    return result;
  }

  void retain(const rclcpp::NodeOptions & options, const rclcpp::Logger & logger)
  {
    const auto types = configured_types(options);
    if (types.empty()) {return;}
    if (!loader_) {
      loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Behavior>>(
        "nav2_core", "nav2_core::Behavior");
    }
    for (const auto & type : types) {
      // Resolve installed registry metadata, not hardcoded paths or Spin/Wait.
      const auto path = loader_->getClassLibraryPath(type);
      if (paths_.count(path)) {continue;}
      loader_->loadLibraryForClass(type);  // Library only: do not create a behavior.
      paths_.insert(path);
      RCLCPP_INFO(logger, "Retained behavior plugin library until component teardown: %s",
        path.c_str());
    }
  }

  size_t retained_library_count() const {return paths_.size();}

private:
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Behavior>> loader_;
  std::set<std::string> paths_;
};

// Base declaration order is intentional: the C5 manager and all node wrappers
// are destroyed FIRST, then BehaviorLibraryRetainer releases nested libraries.
// A derived data member alone would die before the manager's base destructor.
class RetainedBehaviorComponentManager
  : private BehaviorLibraryRetainer, public ComponentManagerIsolated<>
{
public:
  using ComponentManagerIsolated<>::ComponentManagerIsolated;

  size_t retained_behavior_library_count() const {return retained_library_count();}

protected:
  rclcpp::NodeOptions create_node_options(const std::shared_ptr<LoadNode::Request> request) override
  {
    auto options = ComponentManagerIsolated<>::create_node_options(request);
    if (request->package_name == "nav2_behaviors" &&
      request->plugin_name == "behavior_server::BehaviorServer")
    {
      try {
        retain(options, get_logger());
      } catch (const std::exception & error) {
        // ComponentManager's normal load-service failure reports the reason;
        // never continue with partially unprotected behavior node construction.
        throw rclcpp_components::ComponentManagerException(
                std::string("Failed to retain behavior plugin library: ") + error.what());
      }
    }
    return options;  // Parameters/remaps/options are not rewritten.
  }
};
}  // namespace fire_robot_navigation
#endif  // FIRE_ROBOT_NAVIGATION__RETAINED_BEHAVIOR_COMPONENT_MANAGER_HPP_
