cmake_minimum_required(VERSION 3.8)
project(fire_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(bt_navigator_snapshot src/bt_navigator_snapshot.cpp)
target_compile_features(bt_navigator_snapshot PUBLIC cxx_std_17)
target_include_directories(bt_navigator_snapshot PRIVATE include)
ament_target_dependencies(bt_navigator_snapshot nav2_bt_navigator rclcpp_action)
install(TARGETS bt_navigator_snapshot DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_status_refresh test/test_action_status_refresh.cpp)
  target_compile_features(test_action_status_refresh PUBLIC cxx_std_17)
  target_include_directories(test_action_status_refresh PRIVATE include)
  ament_target_dependencies(test_action_status_refresh rclcpp rclcpp_action nav2_bt_navigator)
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
  ament_add_pytest_test(test_nav2_params test/test_nav2_params.py)
  ament_add_pytest_test(test_nav2_invariants test/test_nav2_invariants.py)
  ament_add_pytest_test(test_nav2_snapshot_launch test/test_nav2_snapshot_launch.py)
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
