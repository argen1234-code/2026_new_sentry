/**
 ******************************************************************************
 * @file    ballistic.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   弹道补偿 C++ 零成本包装。
 *
 * @note    本文件只做类型别名和 inline 转发，运行时实现位于 Src/ballistic.c。
 * @note    不使用虚函数、RTTI 或动态内存。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_BALLISTIC_HPP
#define PRINTK_ALGORITHM_BALLISTIC_HPP

#include "Src/ballistic.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只聚合类型别名、constexpr 和 inline 转发，不引入运行时控制逻辑。
 * ========================================================================== */
namespace Algo::Ballistic {

/** @brief 弹道算法执行状态。 */
using Status = ::BallisticStatus;

/** @brief 弹道补偿配置。 */
using Config = ::BallisticCfg;

/** @brief 弹道补偿计算结果。 */
using Result = ::BallisticResult;

/** @brief 执行成功。 */
inline constexpr Status Ok = BallisticOk;

/**
 * @brief 填充默认弹道配置。
 *
 * @param cfg 配置输出。
 * @return 执行状态。
 */
inline Status defaultConfig(Config &cfg)
{
    return ballistic_default_cfg(&cfg);
}

/**
 * @brief 按目标水平距离和高度计算带空气阻力的 pitch 补偿。
 *
 * @param horizontal 目标水平距离，单位 m。
 * @param vertical 目标相对枪口高度，单位 m。
 * @param cfg 弹道配置。
 * @param out 计算结果输出。
 * @return 执行状态。
 */
inline Status dragPitch(float32_t horizontal, float32_t vertical, const Config &cfg, Result &out)
{
    return ballistic_drag_pitch(horizontal, vertical, &cfg, &out);
}

/**
 * @brief 使用默认配置按目标水平距离和高度计算 pitch 补偿。
 *
 * @param horizontal 目标水平距离，单位 m。
 * @param vertical 目标相对枪口高度，单位 m。
 * @param out 计算结果输出。
 * @return 执行状态。
 */
inline Status dragPitch(float32_t horizontal, float32_t vertical, Result &out)
{
    return ballistic_drag_pitch(horizontal, vertical, nullptr, &out);
}

/**
 * @brief 按视觉距离和目标仰角计算带枪口偏置的 pitch 补偿。
 *
 * @param shootAngle 目标视线仰角，单位 rad。
 * @param distance 目标视线距离，单位 m。
 * @param cfg 弹道配置。
 * @param out 计算结果输出。
 * @return 执行状态。
 */
inline Status pitch(float32_t shootAngle, float32_t distance, const Config &cfg, Result &out)
{
    return ballistic_pitch(shootAngle, distance, &cfg, &out);
}

/**
 * @brief 使用默认配置按视觉距离和目标仰角计算 pitch 补偿。
 *
 * @param shootAngle 目标视线仰角，单位 rad。
 * @param distance 目标视线距离，单位 m。
 * @param out 计算结果输出。
 * @return 执行状态。
 */
inline Status pitch(float32_t shootAngle, float32_t distance, Result &out)
{
    return ballistic_pitch(shootAngle, distance, nullptr, &out);
}

} // namespace Algo::Ballistic

#endif /* PRINTK_ALGORITHM_BALLISTIC_HPP */
