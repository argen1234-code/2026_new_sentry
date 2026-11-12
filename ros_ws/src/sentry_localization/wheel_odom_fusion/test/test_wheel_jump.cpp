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

using wheel_odom_fusion::compose;
using wheel_odom_fusion::inverse;
using wheel_odom_fusion::isWheelStepJump;
using wheel_odom_fusion::JumpGateParams;
using wheel_odom_fusion::kPi;
using wheel_odom_fusion::Pose2;
using wheel_odom_fusion::Twist2;
using wheel_odom_fusion::wrapPi;

namespace
{
// 标定包实测：50 Hz 下单步 |Δxy| max 8.6 mm，|Δyaw| max 0.030 rad。
constexpr double kDt = 0.02;
}  // namespace

TEST(WheelJump, NormalStepNotJump)
{
  const JumpGateParams p;
  const Twist2 spin{0.4, 0.0, 0.5};
  EXPECT_FALSE(isWheelStepJump(Pose2{0.006, 0.0, 0.01}, kDt, spin, spin, p));
}

TEST(WheelJump, LargeTranslationIsJump)
{
  const JumpGateParams p;
  const Twist2 still;
  EXPECT_TRUE(isWheelStepJump(Pose2{0.5, 0.0, 0.0}, kDt, still, still, p));
}

TEST(WheelJump, RotationConsistentWithGyroNotJump)
{
  const JumpGateParams p;
  const Twist2 spin{0.0, 0.0, 1.5};
  // 陀螺积分 1.5 * 0.02 = 0.03 rad，与位姿一致。
  EXPECT_FALSE(isWheelStepJump(Pose2{0.0, 0.0, 0.03}, kDt, spin, spin, p));
}

TEST(WheelJump, RotationWithoutGyroIsJump)
{
  const JumpGateParams p;
  const Twist2 still;
  // 0.08 rad 仍在绝对门限 8*0.02+0.1 = 0.26 之内，但陀螺说没转，所以判跳。
  EXPECT_TRUE(isWheelStepJump(Pose2{0.0, 0.0, 0.08}, kDt, still, still, p));
}

TEST(WheelJump, FrozenGyroStillCatchesLargeYawJump)
{
  const JumpGateParams p;
  const Twist2 still;
  EXPECT_TRUE(isWheelStepJump(Pose2{0.0, 0.0, 0.2}, kDt, still, still, p));
}

TEST(WheelJump, YawWrapAcrossPiNotJump)
{
  const JumpGateParams p;
  const Pose2 a{0.0, 0.0, wrapPi(179.0 * kPi / 180.0)};
  const Pose2 b{0.0, 0.0, wrapPi(-179.0 * kPi / 180.0)};
  const Pose2 step = compose(inverse(a), b);
  // 跨 ±pi 的 2° 转动，不是 358° 跳变。
  EXPECT_NEAR(std::fabs(step.yaw), 2.0 * kPi / 180.0, 1e-9);
  const Twist2 spin{0.0, 0.0, std::fabs(step.yaw) / kDt};
  EXPECT_FALSE(isWheelStepJump(step, kDt, spin, spin, p));
}

TEST(WheelJump, NonPositiveDtNotJump)
{
  const JumpGateParams p;
  const Twist2 still;
  EXPECT_FALSE(isWheelStepJump(Pose2{5.0, 5.0, 1.0}, 0.0, still, still, p));
  EXPECT_FALSE(isWheelStepJump(Pose2{5.0, 5.0, 1.0}, -0.02, still, still, p));
}
