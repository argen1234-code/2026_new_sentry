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

#ifndef WHEEL_ODOM_FUSION__FUSION_CORE_HPP_
#define WHEEL_ODOM_FUSION__FUSION_CORE_HPP_

#include <algorithm>
#include <cmath>

namespace wheel_odom_fusion
{

constexpr double kPi = 3.14159265358979323846;

/// 平面位姿（SE(2)），语义为「子帧在父帧中的位姿」。
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// 折到 [-pi, pi]。
inline double wrapPi(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

/// a * b：先应用 b，再应用 a。
inline Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrapPi(a.yaw + b.yaw)};
}

inline Pose2 inverse(const Pose2 & a)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {-c * a.x - s * a.y, s * a.x - c * a.y, wrapPi(-a.yaw)};
}

/// 轮式轨迹相对 LIO 的残差偏移：R = P_L * W^-1（表达在 odom 帧，左乘）。
inline Pose2 residual(const Pose2 & p_lio, const Pose2 & wheel)
{
  return compose(p_lio, inverse(wheel));
}

/// 融合输出：P = O * W。
inline Pose2 predict(const Pose2 & offset, const Pose2 & wheel)
{
  return compose(offset, wheel);
}

/// 分量插值 + 最短弧 yaw。
inline Pose2 blend(const Pose2 & a, const Pose2 & b, double alpha)
{
  return {(1.0 - alpha) * a.x + alpha * b.x,
    (1.0 - alpha) * a.y + alpha * b.y,
    wrapPi(a.yaw + alpha * wrapPi(b.yaw - a.yaw))};
}

inline Pose2 interpolate(const Pose2 & a, const Pose2 & b, double u)
{
  return {a.x + u * (b.x - a.x), a.y + u * (b.y - a.y),
    wrapPi(a.yaw + u * wrapPi(b.yaw - a.yaw))};
}

// ---------------------------------------------------------------------------
// 置信度门控：alpha 的语义是「多快采纳 LIO」，轮式越不可信，alpha 越应该被抬高。
// 门控只会把 alpha 从基值往上抬，永不低于基值。
// ---------------------------------------------------------------------------

/// 车体瞬时速度（来自 /wheel_odom 的 twist）。
struct Twist2
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/// 一个时间窗口内「轮式位姿路径」与「twist 积分路径」的统计量。
struct MotionWindow
{
  double dt{0.0};
  double pose_path_m{0.0};      // sum |Δxy|
  double pose_path_rad{0.0};    // sum |Δyaw|
  double twist_path_m{0.0};     // sum 0.5*(|v_i|+|v_i+1|)*dt
  double twist_path_rad{0.0};   // sum 0.5*(|wz_i|+|wz_i+1|)*dt
};

struct ConfidenceParams
{
  double alpha{0.2};
  double alpha_max{1.0};
  double alpha_mismatch{0.5};
  double spin_gain{5.0};
  double spin_deadband_rps{0.0};
  double still_speed_mps{0.05};
  double still_yaw_rps{0.1};
  double min_motion_m{0.02};
  double min_motion_rad{0.05};
  double ratio_max{3.0};
};

/// 自转门控：alpha_eff = clamp(alpha*(1 + gain*max(0,|wz|-deadband)), alpha, alpha_max)。
inline double spinGateAlpha(const Twist2 & twist, const ConfidenceParams & p)
{
  const double w =
    std::max(0.0, std::fabs(twist.wz) - std::max(0.0, p.spin_deadband_rps));
  const double boosted = p.alpha * (1.0 + std::max(0.0, p.spin_gain) * w);
  return std::min(std::max(boosted, p.alpha), p.alpha_max);
}

/// 自报静止（线速度与角速度都接近 0）但位姿在动：物理上不可能。
inline bool zeroSpeedMismatch(const MotionWindow & w, const ConfidenceParams & p)
{
  if (w.dt <= 0.0) {
    return false;
  }
  const bool twist_still =
    w.twist_path_m / w.dt <= p.still_speed_mps &&
    w.twist_path_rad / w.dt <= p.still_yaw_rps;
  const bool pose_moved =
    w.pose_path_m >= p.min_motion_m || w.pose_path_rad >= p.min_motion_rad;
  return twist_still && pose_moved;
}

inline bool ratioMismatch(double pose_path, double twist_path, double ratio_max, double min_path)
{
  if (std::max(pose_path, twist_path) < min_path) {
    return false;  // 真静止：两条路径都小，不评估
  }
  const double lo = std::max(std::min(pose_path, twist_path), min_path);
  return std::max(pose_path, twist_path) > ratio_max * lo;
}

/// 一致性失配：位姿路径与 twist 积分路径比值越界（任一方向），平移与 yaw 都查。
inline bool consistencyMismatch(const MotionWindow & w, const ConfidenceParams & p)
{
  return ratioMismatch(w.pose_path_m, w.twist_path_m, p.ratio_max, p.min_motion_m) ||
         ratioMismatch(w.pose_path_rad, w.twist_path_rad, p.ratio_max, p.min_motion_rad);
}

/// 最终门控 alpha：保证在 [alpha, alpha_max] 区间内。
inline double gatedAlpha(
  const MotionWindow & w, const Twist2 & twist, const ConfidenceParams & p)
{
  double a = spinGateAlpha(twist, p);
  if (zeroSpeedMismatch(w, p) || consistencyMismatch(w, p)) {
    a = std::max(a, std::min(p.alpha_mismatch, p.alpha_max));
  }
  return std::min(std::max(a, p.alpha), p.alpha_max);
}

// ---------------------------------------------------------------------------
// 轮式样本跳变判据
// ---------------------------------------------------------------------------

struct JumpGateParams
{
  double max_linear_mps{5.0};
  double max_angular_rps{8.0};
  double linear_margin_m{0.05};
  double yaw_margin_rad{0.1};
  double yaw_gyro_tolerance_rad{0.05};
};

/// 单步位姿跳变判定。yaw 除绝对值门限外，还与陀螺积分比对（实测差 p99 = 0.0059 rad）。
inline bool isWheelStepJump(
  const Pose2 & step, double dt, const Twist2 & prev, const Twist2 & cur,
  const JumpGateParams & p)
{
  if (dt <= 0.0) {
    return false;
  }
  if (std::hypot(step.x, step.y) > p.max_linear_mps * dt + p.linear_margin_m) {
    return true;
  }
  if (std::fabs(step.yaw) > p.max_angular_rps * dt + p.yaw_margin_rad) {
    return true;
  }
  const double wz = 0.5 * (prev.wz + cur.wz);
  return std::fabs(wrapPi(step.yaw - wz * dt)) > p.yaw_gyro_tolerance_rad;
}

}  // namespace wheel_odom_fusion

#endif  // WHEEL_ODOM_FUSION__FUSION_CORE_HPP_
