/**
 ******************************************************************************
 * @file    observer.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   观测器组件 C++ 零成本包装。
 *
 * @note    本文件只做类型别名和 inline 转发，运行时实现全部位于 C 文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_OBSERVER_HPP
#define PRINTK_COMPONENT_OBSERVER_HPP

#include "observer.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Observer {

/** @brief 观测器执行状态。 */
using Status = ::ObserverStatus;

/** @brief IMU 原始输入。 */
using ImuInput = ::ObserverImuInput;

/** @brief 普通电机反馈。 */
using MotorFb = ::ObserverMotorFb;

/** @brief MIT 执行器反馈。 */
using MitFb = ::ObserverMitFb;

/** @brief 观测器周期输入。 */
using Input = ::ObserverInput;

/** @brief INS 姿态状态。 */
using InsState = ::ObserverInsState;

/** @brief 底盘速度状态。 */
using ChassisState = ::ObserverChassisState;

/** @brief 单条平衡腿状态。 */
using LegState = ::ObserverLegState;

/** @brief 平衡底盘状态。 */
using BalanceState = ::ObserverBalanceState;

/** @brief 观测器整体状态。 */
using State = ::ObserverState;

/** @brief 单腿几何配置。 */
using LegConfig = ::ObserverLegCfg;

/** @brief 观测器配置。 */
using Config = ::ObserverCfg;

/** @brief 观测器上下文。 */
using Context = ::Observer;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief INS 编译期开关。 */
inline constexpr bool UseIns = (OBSERVER_USE_INS != 0U);

/** @brief 底盘速度观测编译期开关。 */
inline constexpr bool UseChassis = (OBSERVER_USE_CHASSIS != 0U);

/** @brief 平衡腿观测编译期开关。 */
inline constexpr bool UseBalance = (OBSERVER_USE_BALANCE != 0U);

/** @brief 初始化观测器组件。 */
inline Status init(Context &ctx, const Config &cfg)
{
    return observer_init(&ctx, &cfg);
}

/** @brief 复位观测器动态状态。 */
inline Status reset(Context &ctx)
{
    return observer_reset(&ctx);
}

/** @brief 更新观测器状态。 */
inline Status update(Context &ctx, const Input &input)
{
    return observer_update(&ctx, &input);
}

/** @brief 执行一次完整观测周期。 */
inline Status step(Context &ctx, const Input &input, State &state)
{
    return observer_step(&ctx, &input, &state);
}

/** @brief 获取最近一次观测状态。 */
inline Status get(const Context &ctx, State &state)
{
    return observer_get(&ctx, &state);
}

} // namespace Component::Observer

#endif // PRINTK_COMPONENT_OBSERVER_HPP
