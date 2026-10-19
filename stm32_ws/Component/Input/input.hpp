/**
 ******************************************************************************
 * @file    input.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   统一输入组件 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 类型别名和 inline 包装，运行时实现全部位于 input.c。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_HPP
#define PRINTK_COMPONENT_INPUT_HPP

#include "input.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Input {

/** @brief Input 执行状态。 */
using Status = ::InputStatus;

/** @brief 输入源标识。 */
using Source = ::InputSource;

/** @brief 单轴参考。 */
using AxisRef = ::InputAxisRef;

/** @brief 云台参考。 */
using GimbalRef = ::InputGimbalRef;

/** @brief 底盘参考。 */
using ChassisRef = ::InputChassisRef;

/** @brief 发射参考。 */
using ShootRef = ::InputShootRef;

/** @brief 弹舱动作。 */
using MagAction = ::InputMagAction;

/** @brief 弹舱参考。 */
using MagRef = ::InputMagRef;

/** @brief 弹舱命令。 */
using MagCmd = ::InputMagCmd;

/** @brief 整车输入参考。 */
using Ref = ::InputRef;

/** @brief 组件配置。 */
using Config = ::InputCfg;

/** @brief 单周期输入源集合。 */
using Frame = ::InputFrame;

/** @brief 组件输出。 */
using Output = ::InputOutput;

/** @brief 组件上下文。 */
using Context = ::Input;

/** @brief RC 云台映射配置。 */
using GimbalRcMap = ::InputGimbalRcMap;

/** @brief RC 底盘映射配置。 */
using ChassisRcMap = ::InputChassisRcMap;

/** @brief RC 发射映射配置。 */
using ShootRcMap = ::InputShootRcMap;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 Input 组件。 */
inline Status init(Context *input, const Config *cfg)
{
    return input_init(input, cfg);
}

/** @brief 更新输入聚合和命令输出。 */
inline Status update(Context *input, const Frame *frame)
{
    return input_update(input, frame);
}

/** @brief 获取最近一次输出。 */
inline Status get(const Context *input, Output *out)
{
    return input_get(input, out);
}

/** @brief 执行一次完整输入周期。 */
inline Status step(Context *input, const Frame *frame, Output *out)
{
    return input_step(input, frame, out);
}

} // namespace Component::Input

#endif // PRINTK_COMPONENT_INPUT_HPP
