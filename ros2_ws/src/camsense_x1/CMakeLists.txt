cmake_minimum_required(VERSION 3.5)
project(camsense_x1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

# Parser library (pure C++, no ROS dependency)
add_library(camsense_x1_parser src/camsense_x1_parser.cpp)
target_include_directories(camsense_x1_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(camsense_x1_node src/camsense_x1_node.cpp src/camsense_x1.cpp)
target_include_directories(camsense_x1_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camsense_x1_node camsense_x1_parser)

ament_target_dependencies(camsense_x1_node
  rclcpp
  sensor_msgs
  serial)

install(TARGETS camsense_x1_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No lint suppressions (L3-8). All linters run for real.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camsense_x1 test/test_camsense_x1.cpp)
  target_include_directories(test_camsense_x1 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(test_camsense_x1 camsense_x1_parser)

  ament_add_gtest(test_camsense_x1_node test/test_camsense_x1_node.cpp src/camsense_x1.cpp src/camsense_x1_parser.cpp)
  target_include_directories(test_camsense_x1_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_camsense_x1_node rclcpp sensor_msgs serial)

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    ament_add_gtest(test_real_serial_close test/test_real_serial_close.cpp)
    target_link_libraries(test_real_serial_close util)
    ament_target_dependencies(test_real_serial_close serial)
  endif()
endif()

ament_package()
