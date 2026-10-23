/**
 ******************************************************************************
 * @file    thermal.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   温控控制器 C++ 零成本包装。
 *
 * @note    本文件只做类型别名和 inline 转发，不承载运行时控制逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_CONTROLLER_THERMAL_HPP
#define PRINTK_COMPONENT_CONTROLLER_THERMAL_HPP

#include "thermal.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Controller::Thermal {

/** @brief 温控控制器执行状态。 */
using Status = ::ThermalStatus;

/** @brief 温控 PID 参数。 */
using PidConfig = ::ThermalPidCfg;

/** @brief 温控控制器配置。 */
using Config = ::ThermalCfg;

/** @brief 温控控制器周期输入。 */
using Input = ::ThermalInput;

/** @brief 温控控制器周期输出。 */
using Output = ::ThermalOutput;

/** @brief 温控控制器上下文。 */
using Context = ::Thermal;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化温控控制器。 */
inline Status init(Context &ctx, const Config &cfg)
{
    return thermal_init(&ctx, &cfg);
}

/** @brief 复位温控控制器动态状态。 */
inline Status reset(Context &ctx)
{
    return thermal_reset(&ctx);
}

/** @brief 更新温控控制器。 */
inline Status update(Context &ctx, const Input &input)
{
    return thermal_update(&ctx, &input);
}

/** @brief 获取最近一次温控输出。 */
inline Status output(const Context &ctx, Output &out)
{
    return thermal_output(&ctx, &out);
}

/** @brief 执行一次完整温控周期。 */
inline Status step(Context &ctx, const Input &input, Output &out)
{
    return thermal_step(&ctx, &input, &out);
}

} // namespace Component::Controller::Thermal

#endif // PRINTK_COMPONENT_CONTROLLER_THERMAL_HPP
