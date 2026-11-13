// Copyright 2026 The 2026_new_sentry Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/behaviors/wait_for_clearance.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace pb_nav2_behaviors
{

void WaitForClearance::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const std::string prefix = behavior_name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "costmap_frame", rclcpp::ParameterValue("odom"));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "max_wait_s", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "check_radius", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "clear_ratio", rclcpp::ParameterValue(0.5));

  node->get_parameter(prefix + "service_name", service_name_);
  node->get_parameter(prefix + "costmap_frame", costmap_frame_);
  node->get_parameter(prefix + "max_wait_s", max_wait_s_);
  node->get_parameter(prefix + "check_radius", check_radius_);
  node->get_parameter(prefix + "clear_ratio", clear_ratio_);

  costmap_client_ =
    node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  RCLCPP_INFO(
    logger_, "WaitForClearance: service=%s frame=%s max_wait=%.1fs radius=%.1fm ratio=%.2f",
    service_name_.c_str(), costmap_frame_.c_str(), max_wait_s_, check_radius_, clear_ratio_);
}

void WaitForClearance::onCleanup()
{
  costmap_client_.reset();
}

size_t WaitForClearance::countInscribedCells(double rx, double ry, double radius)
{
  if (!costmap_client_ || !costmap_client_->service_is_ready()) {
    return 0;
  }
  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto future = costmap_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(200)) != std::future_status::ready) {
    return 0;
  }
  const auto & costmap = future.get()->map;
  const auto & meta = costmap.metadata;
  const double res = meta.resolution;
  const double r2 = radius * radius;
  size_t count = 0;
  for (uint32_t my = 0; my < meta.size_y; ++my) {
    const double wy = meta.origin.position.y + (static_cast<double>(my) + 0.5) * res;
    const double dy = wy - ry;
    if (dy * dy > r2) {
      continue;
    }
    for (uint32_t mx = 0; mx < meta.size_x; ++mx) {
      const double wx = meta.origin.position.x + (static_cast<double>(mx) + 0.5) * res;
      const double dx = wx - rx;
      if (dx * dx + dy * dy > r2) {
        continue;
      }
      if (costmap.data[static_cast<size_t>(my) * meta.size_x + mx] >=
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        ++count;
        if (count > 100000) {
          return count;
        }
      }
    }
  }
  return count;
}

nav2_behaviors::Status WaitForClearance::onRun(
  const std::shared_ptr<const BackUpAction::Goal> /*command*/)
{
  stopRobot();
  waiting_ = false;

  if (!costmap_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(logger_, "WaitForClearance: %s 不可用，直接交给后续恢复", service_name_.c_str());
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "WaitForClearance: 无法获取当前位姿");
    return nav2_behaviors::Status::FAILED;
  }

  initial_blocked_ = countInscribedCells(
    pose.pose.position.x, pose.pose.position.y, check_radius_);
  start_time_ = clock_->now();
  waiting_ = true;

  RCLCPP_WARN(
    logger_, "路径被堵：开始等待（周围 %.1f m 内内切障碍格 %zu 个，最多 %.1f s）",
    check_radius_, initial_blocked_, max_wait_s_);

  if (initial_blocked_ == 0) {
    RCLCPP_INFO(logger_, "WaitForClearance: 周围无障碍，直接继续");
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status WaitForClearance::onCycleUpdate()
{
  stopRobot();

  if ((clock_->now() - start_time_).seconds() > max_wait_s_) {
    RCLCPP_WARN(logger_, "WaitForClearance: 等待超时（%.1f s），交给后续恢复", max_wait_s_);
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "WaitForClearance: 无法获取当前位姿");
    return nav2_behaviors::Status::FAILED;
  }

  const size_t now_blocked = countInscribedCells(
    pose.pose.position.x, pose.pose.position.y, check_radius_);
  const size_t threshold = static_cast<size_t>(clear_ratio_ * static_cast<double>(initial_blocked_));
  if (now_blocked <= threshold) {
    RCLCPP_INFO(
      logger_, "WaitForClearance: 障碍已减少（%zu → %zu），继续导航", initial_blocked_, now_blocked);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000, "WaitForClearance: 仍在等待（内切障碍格 %zu）", now_blocked);
  return nav2_behaviors::Status::RUNNING;
}

}  // namespace pb_nav2_behaviors

PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::WaitForClearance, nav2_core::Behavior)
