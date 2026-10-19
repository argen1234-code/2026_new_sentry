/**
 ******************************************************************************
 * @file    power_ctrl.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   底盘功率预测控制器 C++ 零成本包装。
 *
 * @note    本文件只提供类型别名和 inline 转发，不承载运行时控制逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_POWER_CTRL_HPP
#define PRINTK_COMPONENT_POWER_CTRL_HPP

#include "power_ctrl.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Controller::Power {

/** @brief 功率控制器执行状态。 */
using Status = ::PowerCtrlStatus;

/** @brief 单个参与功率预测的电机输入。 */
using Motor = ::PowerCtrlMotor;

/** @brief 功率预测模型配置。 */
using Config = ::PowerCtrlCfg;

/** @brief 功率控制器周期输入。 */
using Input = ::PowerCtrlInput;

/** @brief 功率控制器周期输出。 */
using Output = ::PowerCtrlOutput;

/** @brief 功率控制器上下文。 */
using Context = ::PowerCtrl;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化功率控制器。 */
inline Status init(Context &ctx, const Config &cfg)
{
    return power_ctrl_init(&ctx, &cfg);
}

/** @brief 使用默认配置初始化功率控制器。 */
inline Status init(Context &ctx)
{
    return power_ctrl_init(&ctx, nullptr);
}

/** @brief 执行一次功率预测和电流缩放计算。 */
inline Status step(Context &ctx, const Input &input, Output &output)
{
    return power_ctrl_step(&ctx, &input, &output);
}

/** @brief 计算并返回本周期缩放系数。 */
inline float32_t scale(Context &ctx, const Input &input)
{
    return power_ctrl_scale(&ctx, &input);
}

} // namespace Component::Controller::Power

#endif /* PRINTK_COMPONENT_POWER_CTRL_HPP */
