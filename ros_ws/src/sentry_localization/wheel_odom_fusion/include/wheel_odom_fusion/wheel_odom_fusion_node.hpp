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

#ifndef WHEEL_ODOM_FUSION__WHEEL_ODOM_FUSION_NODE_HPP_
#define WHEEL_ODOM_FUSION__WHEEL_ODOM_FUSION_NODE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "wheel_odom_fusion/fusion_core.hpp"

namespace wheel_odom_fusion
{

/// 轮式位姿做预测、LIO 做校正，以轮式频率发布 odom -> base_footprint 与 /odometry。
class WheelOdomFusionNode : public rclcpp::Node
{
public:
  explicit WheelOdomFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class Mode
  {
    WAIT_LIO,
    LIO_ONLY,
    FUSED,
  };

  struct WheelSample
  {
    rclcpp::Time stamp;
    Pose2 raw;
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  void wheelCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void watchdogCallback();

  bool ensureStatics();
  Pose2 interpolateWheel(const rclcpp::Time & stamp) const;
  Twist2 twistAt(const rclcpp::Time & stamp) const;
  MotionWindow motionWindow(const rclcpp::Time & stamp) const;
  /// 返回 true 表示已重锚定（当前样本应继续入缓冲），false 表示应丢弃该样本。
  bool handleWheelStepJump(const Pose2 & raw, const Pose2 & step, const rclcpp::Time & stamp);
  void reanchor(const Pose2 & raw);
  void publishPose(
    const Pose2 & planar, bool has_twist, const WheelSample * sample, const rclcpp::Time & stamp);

  // 帧名与话题
  std::string odom_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;

  // 融合参数
  bool publish_tf_{true};
  bool publish_odometry_{true};
  double alpha_{0.2};
  double snap_translation_m_{0.5};
  double snap_yaw_rad_{0.5};
  int snap_consecutive_{3};
  double wheel_timeout_s_{0.2};
  double lio_timeout_s_{0.5};
  double wheel_buffer_s_{1.0};
  double max_wheel_linear_mps_{5.0};
  double max_wheel_angular_rps_{8.0};
  double max_wheel_dt_s_{0.5};
  bool publish_on_lio_{false};
  double tf_lookup_timeout_s_{0.2};

  // 置信度门控与跳变判据
  ConfidenceParams confidence_;
  double gate_window_s_{0.2};
  JumpGateParams jump_gate_;
  int jump_reanchor_consecutive_{3};
  int jump_reject_count_{0};
  Pose2 last_reject_raw_;
  rclcpp::Time last_reject_stamp_;
  bool have_last_reject_{false};

  // ROS 接口
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  mutable std::mutex mutex_;

  // 静态外参（查不到就不发布，绝不退化成单位阵）
  tf2::Transform t_lidar_base_;
  tf2::Transform t_base_gimbal_;
  double t_base_gimbal_yaw_{0.0};
  bool have_static_{false};

  // LIO 侧状态
  tf2::Transform latest_p_lio_;
  bool have_p_lio_{false};
  bool lio_stale_{false};
  rclcpp::Time last_lio_rx_;

  // 轮式侧状态
  std::deque<WheelSample> wheel_buf_;
  rclcpp::Time last_wheel_rx_;

  // 融合状态
  Mode mode_{Mode::WAIT_LIO};
  Pose2 anchor_;
  bool have_anchor_{false};
  Pose2 offset_;
  int outlier_count_{0};
  Pose2 last_output_;
  bool have_output_{false};
};

}  // namespace wheel_odom_fusion

#endif  // WHEEL_ODOM_FUSION__WHEEL_ODOM_FUSION_NODE_HPP_
