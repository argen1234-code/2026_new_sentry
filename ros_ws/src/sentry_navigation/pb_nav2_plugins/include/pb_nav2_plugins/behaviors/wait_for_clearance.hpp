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

#ifndef PB_NAV2_PLUGINS__BEHAVIORS__WAIT_FOR_CLEARANCE_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__WAIT_FOR_CLEARANCE_HPP_

#include <memory>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::WaitForClearance
 * @brief 路被堵死时先停下来等：周期检查局部代价地图里车周围的障碍是否减少，
 *        减少到阈值以下就返回 SUCCEEDED，超过 max_wait_s 返回 FAILED（交给 BackUp）。
 *
 * 与 HERO RM2026 的"道路被堵时的等待策略"同一思路：不绕行、不推，先等障碍自己离开。
 * 复用 BackUp 动作类型，BT 里用 <BackUp server_name="wait_for_clearance"/> 调用。
 */
class WaitForClearance : public nav2_behaviors::TimedBehavior<BackUpAction>
{
public:
  WaitForClearance() = default;

  void onConfigure() override;
  void onCleanup() override;

  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  /// 统计车心周围 radius 内的内切障碍格数量（cost >= 253）
  size_t countInscribedCells(double rx, double ry, double radius);

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::string service_name_;
  std::string costmap_frame_;
  double max_wait_s_{10.0};
  double check_radius_{2.0};
  double clear_ratio_{0.5};
  size_t initial_blocked_{0};
  rclcpp::Time start_time_;
  bool waiting_{false};
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__WAIT_FOR_CLEARANCE_HPP_
