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

#include <cmath>

#include "wheel_odom_fusion/fusion_core.hpp"

using wheel_odom_fusion::blend;
using wheel_odom_fusion::compose;
using wheel_odom_fusion::interpolate;
using wheel_odom_fusion::inverse;
using wheel_odom_fusion::kPi;
using wheel_odom_fusion::Pose2;
using wheel_odom_fusion::predict;
using wheel_odom_fusion::residual;
using wheel_odom_fusion::wrapPi;

namespace
{
constexpr double kTol = 1e-9;

Pose2 deg(double x, double y, double yaw_deg)
{
  return {x, y, wrapPi(yaw_deg * kPi / 180.0)};
}

double angleDiff(double a, double b)
{
  return std::fabs(wrapPi(a - b));
}
}  // namespace

TEST(FusionCore, ComposeWithIdentity)
{
  const Pose2 a = deg(1.0, -2.0, 30.0);
  const Pose2 identity;
  const Pose2 r1 = compose(a, identity);
  const Pose2 r2 = compose(identity, a);
  EXPECT_NEAR(r1.x, a.x, kTol);
  EXPECT_NEAR(r1.y, a.y, kTol);
  EXPECT_NEAR(angleDiff(r1.yaw, a.yaw), 0.0, kTol);
  EXPECT_NEAR(r2.x, a.x, kTol);
  EXPECT_NEAR(r2.y, a.y, kTol);
  EXPECT_NEAR(angleDiff(r2.yaw, a.yaw), 0.0, kTol);
}

TEST(FusionCore, InverseRoundTrip)
{
  const Pose2 a = deg(0.7, 1.3, 117.0);
  const Pose2 r = compose(a, inverse(a));
  EXPECT_NEAR(r.x, 0.0, kTol);
  EXPECT_NEAR(r.y, 0.0, kTol);
  EXPECT_NEAR(angleDiff(r.yaw, 0.0), 0.0, kTol);
}

TEST(FusionCore, ResidualRecoversOffset)
{
  const Pose2 offset = deg(3.0, -1.5, 20.0);
  const Pose2 wheel = deg(0.4, 0.2, -15.0);
  const Pose2 p_lio = predict(offset, wheel);

  const Pose2 measured = residual(p_lio, wheel);
  EXPECT_NEAR(measured.x, offset.x, 1e-9);
  EXPECT_NEAR(measured.y, offset.y, 1e-9);
  EXPECT_NEAR(angleDiff(measured.yaw, offset.yaw), 0.0, 1e-9);
}

TEST(FusionCore, BlendConvergesToTarget)
{
  Pose2 offset = deg(0.0, 0.0, 0.0);
  const Pose2 target = deg(1.0, -0.5, 0.8);
  for (int i = 0; i < 200; ++i) {
    offset = blend(offset, target, 0.05);
  }
  EXPECT_NEAR(offset.x, target.x, 1e-3);
  EXPECT_NEAR(offset.y, target.y, 1e-3);
  EXPECT_NEAR(angleDiff(offset.yaw, target.yaw), 0.0, 1e-3);
}

TEST(FusionCore, BlendTakesShortestArcAcrossPi)
{
  const Pose2 a = deg(0.0, 0.0, 179.0);
  const Pose2 b = deg(0.0, 0.0, -179.0);
  const Pose2 r = blend(a, b, 0.5);
  // 必须从 179 经 180 走到 -179 一侧，而不是绕过 0。
  EXPECT_NEAR(std::fabs(r.yaw), kPi, 1e-6);
}

TEST(FusionCore, InterpolateEndpointsAndMidpoint)
{
  const Pose2 a = deg(0.0, 0.0, 170.0);
  const Pose2 b = deg(1.0, 2.0, -170.0);

  const Pose2 start = interpolate(a, b, 0.0);
  EXPECT_NEAR(start.x, a.x, kTol);
  EXPECT_NEAR(angleDiff(start.yaw, a.yaw), 0.0, kTol);

  const Pose2 end = interpolate(a, b, 1.0);
  EXPECT_NEAR(end.x, b.x, kTol);
  EXPECT_NEAR(angleDiff(end.yaw, b.yaw), 0.0, kTol);

  const Pose2 mid = interpolate(a, b, 0.5);
  EXPECT_NEAR(mid.x, 0.5, kTol);
  EXPECT_NEAR(mid.y, 1.0, kTol);
  EXPECT_NEAR(std::fabs(mid.yaw), kPi, 1e-6);
}

TEST(FusionCore, WrapPiRange)
{
  EXPECT_NEAR(wrapPi(0.0), 0.0, kTol);
  EXPECT_NEAR(wrapPi(3.0 * kPi), -kPi, 1e-9);
  EXPECT_NEAR(wrapPi(-3.0 * kPi), -kPi, 1e-9);
  EXPECT_NEAR(wrapPi(2.0 * kPi + 0.1), 0.1, 1e-9);
}

TEST(FusionCore, PredictWithoutOffsetFollowsWheel)
{
  const Pose2 wheel = deg(0.25, -0.1, 12.0);
  const Pose2 r = predict(Pose2{}, wheel);
  EXPECT_NEAR(r.x, wheel.x, kTol);
  EXPECT_NEAR(r.y, wheel.y, kTol);
  EXPECT_NEAR(angleDiff(r.yaw, wheel.yaw), 0.0, kTol);
}
