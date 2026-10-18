cmake_minimum_required(VERSION 3.8)
project(referee_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sentry_interfaces REQUIRED)

add_executable(referee_serial_bridge_node
  src/referee_serial_bridge_node.cpp
)
target_include_directories(referee_serial_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(referee_serial_bridge_node
  rclcpp
  sentry_interfaces
)

install(TARGETS referee_serial_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
