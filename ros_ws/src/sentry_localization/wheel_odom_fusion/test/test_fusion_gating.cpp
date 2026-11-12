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

#include <gtest/gtest.h>

#include "wheel_odom_fusion/fusion_core.hpp"

using wheel_odom_fusion::ConfidenceParams;
using wheel_odom_fusion::consistencyMismatch;
using wheel_odom_fusion::gatedAlpha;
using wheel_odom_fusion::MotionWindow;
using wheel_odom_fusion::spinGateAlpha;
using wheel_odom_fusion::Twist2;
using wheel_odom_fusion::zeroSpeedMismatch;

namespace
{
constexpr double kTol = 1e-9;

/// 窗口内 twist 与位姿都只有微小的正常噪声（50 Hz 下 10 帧的典型量级）。
MotionWindow quietWindow()
{
  MotionWindow w;
  w.dt = 0.2;
  w.pose_path_m = 0.02;
  w.pose_path_rad = 0.01;
  w.twist_path_m = 0.02;
  w.twist_path_rad = 0.01;
  return w;
}
}  // namespace

TEST(FusionGating, SpinGateAtRestEqualsAlpha)
{
  const ConfidenceParams p;
  const Twist2 twist;
  EXPECT_NEAR(spinGateAlpha(twist, p), p.alpha, kTol);
}

TEST(FusionGating, SpinGateMonotonicInYawRate)
{
  const ConfidenceParams p;
  const double slow = spinGateAlpha(Twist2{0.0, 0.0, 0.05}, p);
  const double mid = spinGateAlpha(Twist2{0.0, 0.0, 0.1}, p);
  const double fast = spinGateAlpha(Twist2{0.0, 0.0, 0.2}, p);
  EXPECT_LT(slow, mid);
  EXPECT_LT(mid, fast);
  EXPECT_NEAR(mid, p.alpha * 1.5, kTol);
}

TEST(FusionGating, SpinGateCapsAtAlphaMax)
{
  const ConfidenceParams p;
  EXPECT_NEAR(spinGateAlpha(Twist2{0.0, 0.0, 5.0}, p), p.alpha_max, kTol);
}

TEST(FusionGating, SpinGateRespectsDeadband)
{
  ConfidenceParams p;
  p.spin_deadband_rps = 0.2;
  EXPECT_NEAR(spinGateAlpha(Twist2{0.0, 0.0, 0.15}, p), p.alpha, kTol);
  // 只有超出死区的部分参与增益：alpha * (1 + 5 * (0.3 - 0.2)) = 0.3
  EXPECT_NEAR(spinGateAlpha(Twist2{0.0, 0.0, 0.3}, p), 0.3, kTol);
}

TEST(FusionGating, SpinGateNeverBelowAlpha)
{
  ConfidenceParams p;
  p.spin_gain = 0.0;
  EXPECT_NEAR(spinGateAlpha(Twist2{0.0, 0.0, 1.0}, p), p.alpha, kTol);
  // 负增益不允许把 alpha 压到基值以下。
  p.spin_gain = -3.0;
  EXPECT_NEAR(spinGateAlpha(Twist2{0.0, 0.0, 0.5}, p), p.alpha, kTol);
}

TEST(FusionGating, ZeroSpeedMismatchWhenPoseMoves)
{
  const ConfidenceParams p;
  MotionWindow w;
  w.dt = 0.2;
  w.pose_path_m = 0.05;  // 自报静止却挪了 5 cm
  EXPECT_TRUE(zeroSpeedMismatch(w, p));
  EXPECT_NEAR(gatedAlpha(w, Twist2{}, p), p.alpha_mismatch, kTol);
}

TEST(FusionGating, ZeroSpeedMismatchIgnoredWhenPoseStuck)
{
  const ConfidenceParams p;
  MotionWindow w;
  w.dt = 0.2;
  w.pose_path_m = 0.001;
  w.pose_path_rad = 0.001;
  EXPECT_FALSE(zeroSpeedMismatch(w, p));
}

TEST(FusionGating, ZeroSpeedMismatchIgnoredWhenTwistMoves)
{
  const ConfidenceParams p;
  MotionWindow w;
  w.dt = 0.2;
  w.twist_path_m = 0.2;   // 平均 1 m/s
  w.twist_path_rad = 0.04;
  w.pose_path_m = 0.2;
  w.pose_path_rad = 0.04;
  EXPECT_FALSE(zeroSpeedMismatch(w, p));
}

TEST(FusionGating, ConsistencyMismatchLinear)
{
  const ConfidenceParams p;
  MotionWindow w = quietWindow();
  w.pose_path_m = 0.3;    // 位姿走了 30 cm，twist 只积出 3 cm
  w.twist_path_m = 0.03;
  EXPECT_TRUE(consistencyMismatch(w, p));
  EXPECT_NEAR(gatedAlpha(w, Twist2{}, p), p.alpha_mismatch, kTol);
}

TEST(FusionGating, ConsistencyMismatchYaw)
{
  const ConfidenceParams p;
  MotionWindow w = quietWindow();
  w.pose_path_rad = 0.3;
  w.twist_path_rad = 0.03;
  EXPECT_TRUE(consistencyMismatch(w, p));
}

TEST(FusionGating, ConsistencyMismatchIgnoredWhenTrulyStill)
{
  const ConfidenceParams p;
  MotionWindow w;
  w.dt = 0.2;
  w.pose_path_m = 0.005;
  w.pose_path_rad = 0.005;
  w.twist_path_m = 0.001;
  w.twist_path_rad = 0.001;
  // 两条路径都低于 min_motion，比值再离谱也不评估，避免真静止时误触发。
  EXPECT_FALSE(consistencyMismatch(w, p));
}

TEST(FusionGating, GatedAlphaKeepsSpinBoostWhenWindowConsistent)
{
  const ConfidenceParams p;
  const MotionWindow w = quietWindow();
  const Twist2 twist{0.0, 0.0, 0.3};
  EXPECT_NEAR(gatedAlpha(w, twist, p), spinGateAlpha(twist, p), kTol);
  EXPECT_GE(gatedAlpha(w, twist, p), p.alpha);
}
