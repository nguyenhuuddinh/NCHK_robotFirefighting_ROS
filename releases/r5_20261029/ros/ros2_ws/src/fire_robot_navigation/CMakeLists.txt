cmake_minimum_required(VERSION 3.8)
project(fire_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Threads REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcl REQUIRED)
find_package(smclib REQUIRED)

# Local Humble shutdown backport; never replaces /opt/ros or the baseline profile.
add_executable(fire_robot_component_container_isolated src/component_container_isolated.cpp)
target_compile_features(fire_robot_component_container_isolated PRIVATE cxx_std_14)
ament_target_dependencies(fire_robot_component_container_isolated rclcpp rclcpp_action rclcpp_components pluginlib nav2_core smclib nav2_costmap_2d rcl_yaml_param_parser rcl)
# Consume the complete Behavior interface through normal ament dependencies.
# This propagates imported target requirements, including lifecycle headers.
# Do not manually copy only INCLUDE_DIRS or directly link behavior plugins.
target_link_libraries(fire_robot_component_container_isolated Threads::Threads)
install(TARGETS fire_robot_component_container_isolated DESTINATION lib/${PROJECT_NAME})
find_package(rclpy REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_msgs REQUIRED)
  ament_add_gtest(test_action_status_refresh test/test_action_status_refresh.cpp TIMEOUT 15)
  target_compile_features(test_action_status_refresh PRIVATE cxx_std_14)
  ament_target_dependencies(test_action_status_refresh rclcpp rclcpp_action nav2_msgs)
  ament_add_gtest(test_isolated_shutdown test/test_isolated_shutdown.cpp TIMEOUT 30)
  target_compile_features(test_isolated_shutdown PRIVATE cxx_std_14)
  ament_target_dependencies(test_isolated_shutdown rclcpp rclcpp_components)
  target_link_libraries(test_isolated_shutdown Threads::Threads)
  ament_add_gtest(test_behavior_library_retention test/test_behavior_library_retention.cpp TIMEOUT 30)
  target_compile_features(test_behavior_library_retention PRIVATE cxx_std_14)
  ament_target_dependencies(test_behavior_library_retention rclcpp rclcpp_components pluginlib nav2_core smclib nav2_costmap_2d rcl_yaml_param_parser rcl)
  target_link_libraries(test_behavior_library_retention Threads::Threads)
  ament_add_gtest(test_navigation_library_retention test/test_navigation_library_retention.cpp TIMEOUT 30
    ENV CPP_TEST_TMP=${CMAKE_CURRENT_BINARY_DIR})
  target_compile_features(test_navigation_library_retention PRIVATE cxx_std_14)
  ament_target_dependencies(test_navigation_library_retention rclcpp rclcpp_action rclcpp_components pluginlib nav2_core smclib nav2_costmap_2d rcl_yaml_param_parser rcl)
  target_link_libraries(test_navigation_library_retention Threads::Threads)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Register Python unit tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav2_launch test/test_nav2_launch.py)
  ament_add_pytest_test(test_nav2_motion_shadow test/test_nav2_motion_shadow.py)
  ament_add_pytest_test(test_nav2_composed_shadow test/test_nav2_composed_shadow.py)
  ament_add_pytest_test(test_nav2_params test/test_nav2_params.py)
  ament_add_pytest_test(test_nav2_invariants test/test_nav2_invariants.py)
endif()

install(DIRECTORY
  launch
  config
  maps
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
  PATTERN ".gitkeep" EXCLUDE
  REGEX "(__pycache__)" EXCLUDE
)

install(FILES
  NAV2_TUNING_GUIDE.md
  NAV2_FUTURE_WORK.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
