cmake_minimum_required(VERSION 3.8)
project(sentry_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sentry_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)   # 如果消息中使用了标准消息类型，则需要
find_package(rosidl_default_generators REQUIRED)
# Include directories
include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExtraCmd.msg"
  "msg/CmdVelUdp.msg"
  DEPENDENCIES std_msgs   # 如果有依赖，列在这里
)

# 确保消息被安装
ament_export_dependencies(rosidl_default_runtime)

# Add executable
add_executable(sentry_decision_node
  src/sentry_decision_node.cpp
)

ament_target_dependencies(sentry_decision_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  sentry_interfaces
  rclcpp_action
  rclcpp_lifecycle
)

# 链接生成的消息
rosidl_target_interfaces(sentry_decision_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install targets
install(TARGETS
  sentry_decision_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cmd_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
