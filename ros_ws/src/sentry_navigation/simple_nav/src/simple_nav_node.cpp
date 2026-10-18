// Copyright 2025
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

#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class SimpleNavNode : public rclcpp::Node
{
public:
  explicit SimpleNavNode()
  : Node("simple_nav_node"), is_navigating_(false)
  {
    // 声明参数
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("loop_patrol", true);   // 是否循环巡逻
    this->declare_parameter("wait_at_goal", 1.0);   // 到达目标点后等待时间（秒）
    // 目标点列表 [x1, y1, yaw1, x2, y2, yaw2, ...]
    this->declare_parameter("goals", std::vector<double>{});

    map_frame_ = this->get_parameter("map_frame").as_string();
    loop_patrol_ = this->get_parameter("loop_patrol").as_bool();
    wait_at_goal_ = this->get_parameter("wait_at_goal").as_double();

    // 解析目标点
    auto goals_flat = this->get_parameter("goals").as_double_array();
    if (goals_flat.size() % 3 != 0) {
      RCLCPP_ERROR(this->get_logger(), "goals参数格式错误，应为 [x, y, yaw, ...]，每组3个值");
    } else {
      for (size_t i = 0; i < goals_flat.size(); i += 3) {
        goals_.push_back({goals_flat[i], goals_flat[i + 1], goals_flat[i + 2]});
      }
    }

    if (goals_.empty()) {
      RCLCPP_WARN(this->get_logger(), "未配置目标点，节点空转");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "共配置 %zu 个目标点，循环巡逻: %s",
      goals_.size(), loop_patrol_ ? "是" : "否");

    // 创建action客户端
    nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");

    // 等待导航服务器启动后发送目标
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        timer_->cancel();
        sendGoals();
      });
  }

private:
  void sendGoals()
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "导航action服务器不可用，5秒后重试");
      timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        [this]() { timer_->cancel(); sendGoals(); });
      return;
    }

    auto goal_msg = NavigateThroughPoses::Goal();
    for (const auto & g : goals_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = this->now();
      pose.pose.position.x = g[0];
      pose.pose.position.y = g[1];
      pose.pose.position.z = 0.0;
      double yaw = g[2];
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      goal_msg.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "发送 %zu 个目标点", goal_msg.poses.size());
    is_navigating_ = true;

    auto opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    opts.goal_response_callback = [this](const GoalHandle::SharedPtr & gh) {
      if (!gh) {
        RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
        is_navigating_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "目标已接受");
      }
    };
    opts.result_callback = [this](const GoalHandle::WrappedResult & result) {
      is_navigating_ = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "导航完成");
        if (loop_patrol_) {
          // 成功后等待再重新发送
          timer_ = this->create_wall_timer(
            std::chrono::duration<double>(wait_at_goal_),
            [this]() { timer_->cancel(); sendGoals(); });
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "导航结束（code=%d），5秒后重试", static_cast<int>(result.code));
        // 失败时也重试（无论是否循环巡逻）
        timer_ = this->create_wall_timer(
          std::chrono::seconds(5),
          [this]() { timer_->cancel(); sendGoals(); });
      }
    };

    nav_client_->async_send_goal(goal_msg, opts);
  }

  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::array<double, 3>> goals_;
  std::string map_frame_;
  bool loop_patrol_;
  double wait_at_goal_;
  bool is_navigating_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNavNode>());
  rclcpp::shutdown();
  return 0;
}
