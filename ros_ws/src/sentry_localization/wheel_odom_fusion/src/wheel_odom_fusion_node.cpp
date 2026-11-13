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

#include "wheel_odom_fusion/wheel_odom_fusion_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wheel_odom_fusion
{

WheelOdomFusionNode::WheelOdomFusionNode(const rclcpp::NodeOptions & options)
: Node("wheel_odom_fusion", options), tf_buffer_(this->get_clock())
{
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "gimbal_yaw");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "front_mid360");

  const std::string wheel_odom_topic =
    declare_parameter<std::string>("wheel_odom_topic", "wheel_odom");
  const std::string lidar_odometry_topic =
    declare_parameter<std::string>("lidar_odometry_topic", "lidar_odometry");
  const std::string odometry_topic = declare_parameter<std::string>("odometry_topic", "odometry");

  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_odometry_ = declare_parameter<bool>("publish_odometry", true);
  alpha_ = declare_parameter<double>("alpha", 0.2);
  snap_translation_m_ = declare_parameter<double>("snap_translation_m", 0.5);
  snap_yaw_rad_ = declare_parameter<double>("snap_yaw_rad", 0.5);
  snap_consecutive_ = declare_parameter<int>("snap_consecutive", 3);
  wheel_timeout_s_ = declare_parameter<double>("wheel_timeout_s", 0.2);
  lio_timeout_s_ = declare_parameter<double>("lio_timeout_s", 0.5);
  lio_hold_s_ = declare_parameter<double>("lio_hold_s", 5.0);
  wheel_buffer_s_ = declare_parameter<double>("wheel_buffer_s", 1.0);
  max_wheel_linear_mps_ = declare_parameter<double>("max_wheel_linear_mps", 5.0);
  max_wheel_angular_rps_ = declare_parameter<double>("max_wheel_angular_rps", 8.0);
  max_wheel_dt_s_ = declare_parameter<double>("max_wheel_dt_s", 0.5);
  publish_on_lio_ = declare_parameter<bool>("publish_on_lio", false);
  tf_lookup_timeout_s_ = declare_parameter<double>("tf_lookup_timeout_s", 0.2);

  // 置信度门控：alpha 只会被抬高，不会低于基值。
  gate_window_s_ = declare_parameter<double>("gate_window_s", 0.2);
  confidence_.alpha = alpha_;
  confidence_.alpha_max = declare_parameter<double>("alpha_max", 1.0);
  confidence_.alpha_mismatch = declare_parameter<double>("alpha_mismatch", 0.5);
  confidence_.spin_gain = declare_parameter<double>("alpha_spin_gain", 5.0);
  confidence_.spin_deadband_rps = declare_parameter<double>("alpha_spin_deadband_rps", 0.0);
  confidence_.still_speed_mps = declare_parameter<double>("gate_still_speed_mps", 0.05);
  confidence_.still_yaw_rps = declare_parameter<double>("gate_still_yaw_rps", 0.1);
  confidence_.min_motion_m = declare_parameter<double>("gate_min_motion_m", 0.02);
  confidence_.min_motion_rad = declare_parameter<double>("gate_min_motion_rad", 0.05);
  confidence_.ratio_max = declare_parameter<double>("gate_ratio_max", 3.0);

  jump_reanchor_consecutive_ = declare_parameter<int>("jump_reanchor_consecutive", 3);
  jump_gate_.max_linear_mps = max_wheel_linear_mps_;
  jump_gate_.max_angular_rps = max_wheel_angular_rps_;
  jump_gate_.linear_margin_m = declare_parameter<double>("jump_linear_margin_m", 0.05);
  jump_gate_.yaw_margin_rad = declare_parameter<double>("jump_yaw_margin_rad", 0.1);
  jump_gate_.yaw_gyro_tolerance_rad = declare_parameter<double>("yaw_jump_tolerance_rad", 0.05);

  // 参数兜底，避免手改 yaml 时出现 alpha 被门控压低或阈值自相矛盾。
  confidence_.alpha = std::max(confidence_.alpha, 1e-4);
  confidence_.alpha_max = std::max(confidence_.alpha_max, confidence_.alpha);
  confidence_.alpha_mismatch =
    std::min(std::max(confidence_.alpha_mismatch, confidence_.alpha), confidence_.alpha_max);
  confidence_.spin_gain = std::max(0.0, confidence_.spin_gain);
  confidence_.ratio_max = std::max(1.0, confidence_.ratio_max);
  gate_window_s_ = std::max(0.0, gate_window_s_);
  jump_reanchor_consecutive_ = std::max(1, jump_reanchor_consecutive_);

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    wheel_odom_topic, sensor_qos,
    std::bind(&WheelOdomFusionNode::wheelCallback, this, std::placeholders::_1));
  lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    lidar_odometry_topic, sensor_qos,
    std::bind(&WheelOdomFusionNode::lioCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic, rclcpp::QoS(10));

  last_lio_rx_ = this->now();
  last_wheel_rx_ = this->now();
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&WheelOdomFusionNode::watchdogCallback, this));

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = create_wall_timer(
    std::chrono::seconds(1), std::bind(&WheelOdomFusionNode::publishDiagnostics, this));

  RCLCPP_INFO(
    get_logger(), "wheel_odom_fusion: %s + %s -> %s (%s -> %s at wheel rate)",
    wheel_odom_topic.c_str(), lidar_odometry_topic.c_str(), odometry_topic.c_str(),
    odom_frame_.c_str(), base_frame_.c_str());
}

bool WheelOdomFusionNode::ensureStatics()
{
  if (have_static_) {
    return true;
  }
  try {
    const auto lidar_to_base = tf_buffer_.lookupTransform(
      lidar_frame_, base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_s_));
    const auto base_to_gimbal = tf_buffer_.lookupTransform(
      base_frame_, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_s_));

    tf2::fromMsg(lidar_to_base.transform, t_lidar_base_);
    tf2::fromMsg(base_to_gimbal.transform, t_base_gimbal_);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(t_base_gimbal_.getRotation()).getRPY(roll, pitch, yaw);
    t_base_gimbal_yaw_ = yaw;

    have_static_ = true;
    RCLCPP_INFO(
      get_logger(), "静态外参已缓存: %s->%s (%.3f, %.3f, %.3f m), %s->%s yaw %.4f rad",
      lidar_frame_.c_str(), base_frame_.c_str(), t_lidar_base_.getOrigin().x(),
      t_lidar_base_.getOrigin().y(), t_lidar_base_.getOrigin().z(), base_frame_.c_str(),
      robot_base_frame_.c_str(), t_base_gimbal_yaw_);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "静态 TF 查询失败，跳过本次更新: %s", ex.what());
    return false;
  }
}

Pose2 WheelOdomFusionNode::interpolateWheel(const rclcpp::Time & stamp) const
{
  if (wheel_buf_.empty()) {
    // 空缓冲时假设「自锚点起未动」（W = I）；正常流程不会走到这里（重锚定后同一回调内
    // 就补入了新样本），但绝不能让 front() 作用在空 deque 上。
    return anchor_;
  }
  const auto & front = wheel_buf_.front();
  if (stamp <= front.stamp) {
    return front.raw;
  }
  for (size_t i = 1; i < wheel_buf_.size(); ++i) {
    if (stamp <= wheel_buf_[i].stamp) {
      const auto & a = wheel_buf_[i - 1];
      const auto & b = wheel_buf_[i];
      const double span = (b.stamp - a.stamp).seconds();
      if (span <= 0.0) {
        return b.raw;
      }
      const double u = (stamp - a.stamp).seconds() / span;
      return interpolate(a.raw, b.raw, u);
    }
  }
  return wheel_buf_.back().raw;
}

void WheelOdomFusionNode::reanchor(const Pose2 & raw)
{
  // 重锚定后 W = I，把当前输出位姿吃进偏移，保证位姿连续。
  if (have_output_) {
    offset_ = last_output_;
  }
  anchor_ = raw;
  have_anchor_ = true;
  outlier_count_ = 0;
  wheel_buf_.clear();
  jump_reject_count_ = 0;
  have_last_reject_ = false;
}

Twist2 WheelOdomFusionNode::twistAt(const rclcpp::Time & stamp) const
{
  if (wheel_buf_.empty()) {
    return Twist2{};
  }
  if (stamp <= wheel_buf_.front().stamp) {
    const auto & s = wheel_buf_.front();
    return Twist2{s.vx, s.vy, s.wz};
  }
  for (size_t i = 1; i < wheel_buf_.size(); ++i) {
    if (stamp <= wheel_buf_[i].stamp) {
      const auto & a = wheel_buf_[i - 1];
      const auto & b = wheel_buf_[i];
      const double span = (b.stamp - a.stamp).seconds();
      if (span <= 0.0) {
        return Twist2{b.vx, b.vy, b.wz};
      }
      const double u = (stamp - a.stamp).seconds() / span;
      return Twist2{a.vx + u * (b.vx - a.vx), a.vy + u * (b.vy - a.vy),
        a.wz + u * (b.wz - a.wz)};
    }
  }
  const auto & s = wheel_buf_.back();
  return Twist2{s.vx, s.vy, s.wz};
}

MotionWindow WheelOdomFusionNode::motionWindow(const rclcpp::Time & stamp) const
{
  MotionWindow w;
  if (wheel_buf_.size() < 2) {
    return w;
  }

  const rclcpp::Time begin = stamp - rclcpp::Duration::from_seconds(gate_window_s_);
  size_t first = wheel_buf_.size();
  for (size_t i = 0; i < wheel_buf_.size(); ++i) {
    if (wheel_buf_[i].stamp >= begin) {
      first = i;
      break;
    }
  }
  if (first == wheel_buf_.size()) {
    return w;  // 窗口内一个样本都没有
  }
  if (first > 0) {
    --first;  // 含窗口起点前一帧，保证第一段也有 dt
  }

  for (size_t i = first + 1; i < wheel_buf_.size(); ++i) {
    const auto & a = wheel_buf_[i - 1];
    const auto & b = wheel_buf_[i];
    if (b.stamp > stamp) {
      break;
    }
    const double dt = (b.stamp - a.stamp).seconds();
    if (dt <= 0.0) {
      continue;
    }
    const Pose2 step = compose(inverse(a.raw), b.raw);
    w.dt += dt;
    w.pose_path_m += std::hypot(step.x, step.y);
    w.pose_path_rad += std::fabs(step.yaw);
    w.twist_path_m += 0.5 * (std::hypot(a.vx, a.vy) + std::hypot(b.vx, b.vy)) * dt;
    w.twist_path_rad += 0.5 * (std::fabs(a.wz) + std::fabs(b.wz)) * dt;
  }
  return w;
}

bool WheelOdomFusionNode::handleWheelStepJump(
  const Pose2 & raw, const Pose2 & step, const rclcpp::Time & stamp)
{
  ++jump_reject_count_;

  // 只在「被拒样本彼此也是连续运动」时才认定是整条链换了；否则可能只是一串噪声。
  bool chain_coherent = false;
  if (have_last_reject_) {
    const double rdt = (stamp - last_reject_stamp_).seconds();
    const Pose2 rstep = compose(inverse(last_reject_raw_), raw);
    chain_coherent =
      rdt > 0.0 && rdt <= max_wheel_dt_s_ &&
      std::hypot(rstep.x, rstep.y) <=
      max_wheel_linear_mps_ * rdt + jump_gate_.linear_margin_m &&
      std::fabs(rstep.yaw) <= max_wheel_angular_rps_ * rdt + jump_gate_.yaw_margin_rad;
  }
  last_reject_raw_ = raw;
  last_reject_stamp_ = stamp;
  have_last_reject_ = true;

  if (jump_reject_count_ >= jump_reanchor_consecutive_ && chain_coherent) {
    RCLCPP_WARN(
      get_logger(), "轮式链路切换/原点重置（连续 %d 帧被拒），立即重锚定", jump_reject_count_);
    reanchor(raw);
    return true;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000, "丢弃轮式数据：位姿跳变 %.3f m / %.3f rad（%d/%d）",
    std::hypot(step.x, step.y), std::fabs(step.yaw), jump_reject_count_,
    jump_reanchor_consecutive_);
  return false;
}

void WheelOdomFusionNode::publishPose(
  const Pose2 & planar, bool has_twist, const WheelSample * sample, const rclcpp::Time & stamp)
{
  if (!have_p_lio_ || lio_stale_) {
    return;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double lio_yaw = 0.0;
  tf2::Matrix3x3(latest_p_lio_.getRotation()).getRPY(roll, pitch, lio_yaw);

  tf2::Quaternion q;
  q.setRPY(roll, pitch, planar.yaw);
  tf2::Transform p_3d;
  p_3d.setOrigin(
    tf2::Vector3(planar.x, planar.y, latest_p_lio_.getOrigin().z()));
  p_3d.setRotation(q);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(p_3d);
    tf_broadcaster_->sendTransform(tf_msg);
  }

  if (publish_odometry_) {
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = robot_base_frame_;

    const tf2::Transform odom_to_robot_base = p_3d * t_base_gimbal_;
    const auto & origin = odom_to_robot_base.getOrigin();
    out.pose.pose.position.x = origin.x();
    out.pose.pose.position.y = origin.y();
    out.pose.pose.position.z = origin.z();
    out.pose.pose.orientation = tf2::toMsg(odom_to_robot_base.getRotation());

    if (has_twist && sample != nullptr) {
      // body twist 从 base_footprint 转到 gimbal_yaw（本车 yaw=0）。
      const double c = std::cos(t_base_gimbal_yaw_);
      const double s = std::sin(t_base_gimbal_yaw_);
      out.twist.twist.linear.x = c * sample->vx + s * sample->vy;
      out.twist.twist.linear.y = -s * sample->vx + c * sample->vy;
      out.twist.twist.angular.z = sample->wz;
    }

    odom_pub_->publish(out);
  }

  last_output_ = planar;
  have_output_ = true;
}

void WheelOdomFusionNode::wheelCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  const double wz = msg->twist.twist.angular.z;

  if (std::hypot(vx, vy) > max_wheel_linear_mps_ || std::fabs(wz) > max_wheel_angular_rps_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "丢弃轮式数据：twist (%.3f, %.3f, %.3f) 超限", vx, vy, wz);
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);
  Pose2 raw;
  raw.x = msg->pose.pose.position.x;
  raw.y = msg->pose.pose.position.y;
  raw.yaw = wrapPi(tf2::getYaw(msg->pose.pose.orientation));

  double dt = 0.0;
  if (!wheel_buf_.empty()) {
    dt = (stamp - wheel_buf_.back().stamp).seconds();
    if (dt <= 0.0) {
      return;  // 重复或乱序
    }
    if (dt > max_wheel_dt_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "轮式数据间隔 %.2f s 过大，重新锚定", dt);
      reanchor(raw);
    } else {
      const auto & prev = wheel_buf_.back();
      const Pose2 step = compose(inverse(prev.raw), raw);
      const Twist2 prev_twist{prev.vx, prev.vy, prev.wz};
      const Twist2 cur_twist{vx, vy, wz};
      if (isWheelStepJump(step, dt, prev_twist, cur_twist, jump_gate_)) {
        if (!handleWheelStepJump(raw, step, stamp)) {
          return;  // 孤立尖峰：丢弃，缓冲不动
        }
      } else {
        jump_reject_count_ = 0;
        have_last_reject_ = false;
      }
    }
  }

  WheelSample sample;
  sample.stamp = stamp;
  sample.raw = raw;
  sample.vx = vx;
  sample.vy = vy;
  sample.wz = wz;
  wheel_buf_.push_back(sample);
  while (
    wheel_buf_.size() > 2 &&
    (wheel_buf_.back().stamp - wheel_buf_.front().stamp).seconds() > wheel_buffer_s_)
  {
    wheel_buf_.pop_front();
  }

  last_wheel_rx_ = this->now();

  if (!have_anchor_) {
    anchor_ = raw;
    have_anchor_ = true;
  }

  if (mode_ != Mode::FUSED) {
    return;
  }

  const Pose2 wheel = compose(inverse(anchor_), raw);
  publishPose(predict(offset_, wheel), true, &wheel_buf_.back(), stamp);
}

void WheelOdomFusionNode::lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!ensureStatics()) {
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);
  tf2::Transform odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, odom_to_lidar);
  latest_p_lio_ = odom_to_lidar * t_lidar_base_;
  have_p_lio_ = true;
  lio_stale_ = false;
  lio_hold_ = false;
  last_lio_rx_ = this->now();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(latest_p_lio_.getRotation()).getRPY(roll, pitch, yaw);
  const Pose2 p_lio{
    latest_p_lio_.getOrigin().x(), latest_p_lio_.getOrigin().y(), wrapPi(yaw)};

  const bool wheel_fresh = !wheel_buf_.empty() &&
    (this->now() - last_wheel_rx_).seconds() <= wheel_timeout_s_;

  switch (mode_) {
    case Mode::WAIT_LIO: {
        offset_ = p_lio;
        if (wheel_fresh && have_anchor_) {
          anchor_ = interpolateWheel(stamp);
          mode_ = Mode::FUSED;
          RCLCPP_INFO(get_logger(), "进入融合模式：轮式位姿预测 + LIO 校正");
        } else {
          mode_ = Mode::LIO_ONLY;
          RCLCPP_WARN(get_logger(), "暂无轮式数据，退化为纯 LIO");
        }
        outlier_count_ = 0;
        publishPose(p_lio, false, nullptr, stamp);
        break;
      }
    case Mode::FUSED: {
        const Pose2 wheel = compose(inverse(anchor_), interpolateWheel(stamp));
        const Pose2 measured = residual(p_lio, wheel);
        const double err_translation =
          std::hypot(measured.x - offset_.x, measured.y - offset_.y);
        const double err_yaw = std::fabs(wrapPi(measured.yaw - offset_.yaw));

        if (err_translation > snap_translation_m_ || err_yaw > snap_yaw_rad_) {
          ++outlier_count_;
          if (outlier_count_ >= snap_consecutive_) {
            RCLCPP_WARN(
              get_logger(), "LIO 残差异常 (%.3f m, %.3f rad) 连续 %d 帧，强制对齐",
              err_translation, err_yaw, outlier_count_);
            offset_ = measured;
            outlier_count_ = 0;
          }
        } else {
          outlier_count_ = 0;
          // 轮式越不可信（自转 / 零速却在动 / 位姿与 twist 不自洽），越快采纳 LIO。
          const MotionWindow win = motionWindow(stamp);
          const double alpha_eff = gatedAlpha(win, twistAt(stamp), confidence_);
          offset_ = blend(offset_, measured, alpha_eff);
          last_alpha_eff_ = alpha_eff;
          last_err_translation_ = err_translation;
          last_err_yaw_ = err_yaw;
          last_mismatch_ = zeroSpeedMismatch(win, confidence_) ||
            consistencyMismatch(win, confidence_);
        }

        if (publish_on_lio_) {
          publishPose(predict(offset_, wheel), false, nullptr, stamp);
        }
        break;
      }
    case Mode::LIO_ONLY: {
        offset_ = p_lio;
        if (wheel_fresh && have_anchor_) {
          anchor_ = interpolateWheel(stamp);
          mode_ = Mode::FUSED;
          outlier_count_ = 0;
          RCLCPP_INFO(get_logger(), "轮式数据恢复，重新进入融合模式");
        }
        publishPose(p_lio, false, nullptr, stamp);
        break;
      }
  }
}

void WheelOdomFusionNode::watchdogCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!have_p_lio_) {
    return;
  }

  const double lio_age = (this->now() - last_lio_rx_).seconds();
  if (lio_age > lio_hold_s_) {
    if (!lio_stale_) {
      RCLCPP_ERROR(get_logger(), "LIO 已 %.2f s 无更新，超过保持上限，停止发布位姿", lio_age);
    }
    lio_stale_ = true;
    lio_hold_ = false;
    return;
  }
  if (lio_age > lio_timeout_s_) {
    if (!lio_hold_) {
      RCLCPP_WARN(
        get_logger(), "LIO 已 %.2f s 无更新，改用轮速外推保持发布（上限 %.1f s）", lio_age,
        lio_hold_s_);
    }
    lio_hold_ = true;
    lio_stale_ = false;
    return;
  }
  lio_hold_ = false;

  if (mode_ != Mode::FUSED) {
    return;
  }

  const double wheel_age = (this->now() - last_wheel_rx_).seconds();
  if (wheel_age > wheel_timeout_s_) {
    mode_ = Mode::LIO_ONLY;
    outlier_count_ = 0;
    RCLCPP_WARN(get_logger(), "轮式里程计已 %.2f s 无有效数据，退化为纯 LIO", wheel_age);
  }
}

void WheelOdomFusionNode::publishDiagnostics()
{
  std::lock_guard<std::mutex> lock(mutex_);

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "localization/fusion";
  st.hardware_id = "wheel_odom_fusion";

  const char * state = "NORMAL";
  if (lio_stale_) {
    state = "LIO_STALE";
  } else if (lio_hold_) {
    state = "LIO_STALE_HOLD";
  } else if (mode_ == Mode::LIO_ONLY) {
    state = "WHEEL_LOST";
  } else if (last_mismatch_) {
    state = "MISMATCH";
  }
  const bool bad = std::string(state) != "NORMAL";
  st.level = bad ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  st.message = state;

  auto add = [&st](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      st.values.push_back(kv);
    };
  add("state", state);
  add("alpha_eff", std::to_string(last_alpha_eff_));
  add("last_err_translation_m", std::to_string(last_err_translation_));
  add("last_err_yaw_rad", std::to_string(last_err_yaw_));
  add("mismatch", last_mismatch_ ? "true" : "false");
  add("lio_age_s", std::to_string((this->now() - last_lio_rx_).seconds()));
  add("wheel_age_s", std::to_string((this->now() - last_wheel_rx_).seconds()));

  arr.status.push_back(st);
  diag_pub_->publish(arr);
}

}  // namespace wheel_odom_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_odom_fusion::WheelOdomFusionNode>());
  rclcpp::shutdown();
  return 0;
}
