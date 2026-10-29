// Copyright 2026 Fire Robot contributors. SPDX-License-Identifier: Apache-2.0
#ifndef FIRE_ROBOT_NAVIGATION__RETAINED_NAVIGATION_COMPONENT_MANAGER_HPP_
#define FIRE_ROBOT_NAVIGATION__RETAINED_NAVIGATION_COMPONENT_MANAGER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <stdexcept>
#include <vector>
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rcl/arguments.h"
#include "rclcpp/parameter_map.hpp"
#include "retained_behavior_component_manager.hpp"
#include "action_status_refresh.hpp"

namespace fire_robot_navigation
{
// CallbackGroup and LifecycleNode keep weak control blocks after derived plugins
// are destroyed. Their vtables must remain mapped until the WHOLE node is gone.
class NavigationLibraryRetainer
{
public:
  using Parameters = std::map<std::string, rclcpp::Parameter>;

  static std::vector<std::string> configured_types(
    const Parameters & parameters, const std::string & list_name,
    const std::vector<std::string> & defaults,
    const std::vector<std::string> & default_types)
  {
    const auto list = parameters.find(list_name);
    const auto ids = list == parameters.end() ? defaults : list->second.as_string_array();
    std::vector<std::string> types;
    for (size_t i = 0; i < ids.size(); ++i) {
      const auto type = parameters.find(ids[i] + ".plugin");
      if (type != parameters.end()) {
        types.push_back(type->second.as_string());
      } else if (ids == defaults && default_types.size() == defaults.size()) {
        types.push_back(default_types[i]);
      } else {
        throw std::runtime_error("Missing plugin type: " + ids[i] + ".plugin");
      }
      if (types.back().empty()) {throw std::runtime_error("Empty plugin type: " + ids[i]);}
    }
    return types;
  }

  static Parameters global_parameters(
    const std::shared_ptr<rclcpp::Context> & context, const std::string & node_fqn)
  {
    auto rcl_context = context->get_rcl_context();
    if (!rcl_context) {throw std::runtime_error("Uninitialized child costmap context");}
    rcl_params_t * raw = nullptr;
    const auto rc = rcl_arguments_get_param_overrides(&rcl_context->global_arguments, &raw);
    std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> owned(
      raw, &rcl_yaml_node_struct_fini);
    if (rc != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(rc, "Read child costmap parameters");
    }
    Parameters result;
    if (raw) {
      // Use ROS's wildcard/node-name matching and order rather than parse YAML
      // independently. Last override wins, just as the actual child node does.
      const auto map = rclcpp::parameter_map_from(raw, node_fqn.c_str());
      const auto found = map.find(node_fqn);
      if (found != map.end()) {
        for (const auto & p : found->second) {result[p.get_name()] = p;}
      }
    }
    return result;
  }

  void retain(
    const std::string & package, const std::string & plugin, const std::string & node_namespace,
    const rclcpp::NodeOptions & options, const rclcpp::Logger & logger)
  {
    const bool controller = package == "nav2_controller" &&
      plugin == "nav2_controller::ControllerServer";
    const bool planner = package == "nav2_planner" && plugin == "nav2_planner::PlannerServer";
    if (!controller && !planner) {return;}
    std::string prefix = node_namespace;
    if (prefix.empty()) {prefix = "/";}
    if (prefix.front() != '/') {throw std::runtime_error("Costmap namespace must be absolute");}
    if (prefix.back() != '/') {prefix += '/';}
    const std::string child = controller ? "local_costmap" : "global_costmap";
    // Humble constructs these children with NodeOptions(), including the global
    // default context and process parameters, not the parent's LoadNode overrides.
    const auto parameters = global_parameters(
      rclcpp::NodeOptions().context(), prefix + child + "/" + child);
    const auto layer_types = configured_types(parameters, "plugins",
      {"static_layer", "obstacle_layer", "inflation_layer"},
      {"nav2_costmap_2d::StaticLayer", "nav2_costmap_2d::ObstacleLayer",
        "nav2_costmap_2d::InflationLayer"});
    const auto filter_types = configured_types(parameters, "filters", {}, {});
    for (const auto & type : layer_types) {retain_layer(type, logger);}
    for (const auto & type : filter_types) {retain_layer(type, logger);}
    if (controller) {
      Parameters parent;
      for (const auto & p : options.parameter_overrides()) {parent[p.get_name()] = p;}
      const auto types = configured_types(parent, "controller_plugins", {"FollowPath"},
        {"dwb_core::DWBLocalPlanner"});
      if (!types.empty() && !controllers_) {
        controllers_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
          "nav2_core", "nav2_core::Controller");
      }
      for (const auto & type : types) {retain_type(*controllers_, type, logger);}
    }
  }

  size_t retained_library_count() const {return paths_.size();}

private:
  template<typename T>
  void retain_type(pluginlib::ClassLoader<T> & loader, const std::string & type,
    const rclcpp::Logger & logger)
  {
    const auto path = loader.getClassLibraryPath(type);
    if (paths_.count(path)) {return;}
    loader.loadLibraryForClass(type);  // No plugin instance or periodic work.
    paths_.insert(path);
    RCLCPP_INFO(logger, "Retained navigation plugin library until component teardown: %s",
      path.c_str());
  }

  void retain_layer(const std::string & type, const rclcpp::Logger & logger)
  {
    if (!layers_) {
      layers_ = std::make_unique<pluginlib::ClassLoader<nav2_costmap_2d::Layer>>(
        "nav2_costmap_2d", "nav2_costmap_2d::Layer");
    }
    retain_type(*layers_, type, logger);
  }
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> controllers_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_costmap_2d::Layer>> layers_;
  std::set<std::string> paths_;
};

// First base is destroyed LAST, after C6 behavior retention and the complete
// ComponentManager, including node wrappers, executors and factory loaders.
class RetainedNavigationComponentManager
  : private NavigationLibraryRetainer, public RetainedBehaviorComponentManager
{
public:
  using RetainedBehaviorComponentManager::RetainedBehaviorComponentManager;
  size_t retained_navigation_library_count() const
  {return NavigationLibraryRetainer::retained_library_count();}

  void refresh_bt_action_status()
  {
    for (auto & row : node_wrappers_) {
      auto node = row.second.get_node_base_interface();
      if (std::string(node->get_fully_qualified_name()) == "/bt_navigator") {
        try {
          fire_robot_navigation::refresh_action_status(node);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Nav2 status refresh failed: %s", error.what());
        }
      }
    }
  }

protected:
  rclcpp::NodeOptions create_node_options(const std::shared_ptr<LoadNode::Request> request) override
  {
    auto options = RetainedBehaviorComponentManager::create_node_options(request);
    try {
      NavigationLibraryRetainer::retain(request->package_name, request->plugin_name, request->node_namespace,
        options, get_logger());
    } catch (const std::exception & error) {
      throw rclcpp_components::ComponentManagerException(
              std::string("Failed to retain navigation plugin library: ") + error.what());
    }
    return options;
  }
};
}  // namespace fire_robot_navigation
#endif
