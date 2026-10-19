/**
 ******************************************************************************
 * @file    aim.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   自瞄参考整形组件 C++ 薄包装。
 *
 * @note    本文件只做类型别名和 inline 转发，运行时事实来源位于 C 接口。
 * @note    不引入虚函数、RTTI 或动态分派。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_AIM_HPP
#define PRINTK_COMPONENT_AIM_HPP

#include "aim.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Aim {

/** @brief Aim 执行状态。 */
using Status = ::AimStatus;

/** @brief Aim 配置。 */
using Config = ::AimCfg;

/** @brief Aim 周期输入。 */
using Input = ::AimInput;

/** @brief Aim 周期输出。 */
using Output = ::AimOutput;

/** @brief Aim 上下文。 */
using Context = ::Aim;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 填充默认配置。 */
inline Status default_cfg(Config &cfg)
{
    return aim_default_cfg(&cfg);
}

/** @brief 初始化上下文。 */
inline Status init(Context &ctx, const Config *cfg = nullptr)
{
    return aim_init(&ctx, cfg);
}

/** @brief 重置上下文。 */
inline Status reset(Context &ctx)
{
    return aim_reset(&ctx);
}

/** @brief 执行一次自瞄整形。 */
inline Status step(Context &ctx, const Input &input, Output &output)
{
    return aim_step(&ctx, &input, &output);
}

/** @brief 读取最近输出。 */
inline Status get(const Context &ctx, Output &output)
{
    return aim_get(&ctx, &output);
}

} // namespace Component::Aim

#endif // PRINTK_COMPONENT_AIM_HPP
