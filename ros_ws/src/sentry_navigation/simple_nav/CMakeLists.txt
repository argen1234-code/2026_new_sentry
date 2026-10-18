cmake_minimum_required(VERSION 3.8)
project(simple_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(simple_nav_node src/simple_nav_node.cpp)
target_include_directories(simple_nav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_nav_node PUBLIC cxx_std_17)
ament_target_dependencies(simple_nav_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(TARGETS simple_nav_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
