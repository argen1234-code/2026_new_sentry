/**
 ******************************************************************************
 * @file    mahony.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   Mahony 姿态解算 C++ 零成本包装。
 *
 * @note    本文件只做类型别名和 inline 转发，运行时实现位于 Src/mahony.c。
 * @note    不使用虚函数、RTTI 或动态内存。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_MAHONY_HPP
#define PRINTK_ALGORITHM_MAHONY_HPP

#include "Src/mahony.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只聚合类型别名、constexpr 和 inline 转发，不引入运行时控制逻辑。
 * ========================================================================== */
namespace Algo::Mahony {

/** @brief Mahony 执行状态。 */
using Status = ::MahonyStatus;

/** @brief Mahony 配置。 */
using Config = ::MahonyCfg;

/** @brief Mahony 输入。 */
using Input = ::MahonyInput;

/** @brief Mahony 输出。 */
using Output = ::MahonyOutput;

/** @brief 执行成功。 */
inline constexpr Status Ok = MahonyOk;

/**
 * @brief Mahony 姿态解算器。
 */
struct Filter {
    /** @brief C 实现上下文。 */
    ::Mahony ctx{};

    /**
     * @brief 使用指定配置初始化。
     *
     * @param cfg Mahony 配置。
     * @return 初始化状态。
     */
    Status init(const Config &cfg)
    {
        return mahony_init(&ctx, &cfg);
    }

    /**
     * @brief 使用默认配置初始化。
     *
     * @return 初始化状态。
     */
    Status init()
    {
        return mahony_init(&ctx, nullptr);
    }

    /**
     * @brief 复位动态状态。
     *
     * @return 复位状态。
     */
    Status reset()
    {
        return mahony_reset(&ctx);
    }

    /**
     * @brief 执行一次姿态更新。
     *
     * @param input 本周期输入。
     * @return 更新状态。
     */
    Status update(const Input &input)
    {
        return mahony_update(&ctx, &input);
    }

    /**
     * @brief 执行 Z 轴陀螺仪零偏均值校准。
     *
     * @param gyro_z 当前 Z 轴角速度。
     * @param samples 目标采样数。
     * @return 校准状态。
     */
    Status caliGyroZ(float32_t gyro_z, uint32_t samples)
    {
        return mahony_cali_gyro_z(&ctx, gyro_z, samples);
    }

    /**
     * @brief 执行三轴陀螺仪零偏均值校准。
     *
     * @param gyro 当前三轴角速度。
     * @param samples 目标采样数。
     * @return 校准状态。
     */
    Status caliGyro(const Vec3 &gyro, uint32_t samples)
    {
        return mahony_cali_gyro(&ctx, &gyro, samples);
    }

    /**
     * @brief 设置 Z 轴陀螺仪零偏。
     *
     * @param gyro_z_bias Z 轴陀螺仪零偏。
     * @return 设置状态。
     */
    Status setGyroZ(float32_t gyro_z_bias)
    {
        return mahony_set_gyro_z(&ctx, gyro_z_bias);
    }

    /**
     * @brief 设置三轴陀螺仪零偏。
     *
     * @param gyro_bias 三轴陀螺仪零偏。
     * @return 设置状态。
     */
    Status setGyro(const Vec3 &gyro_bias)
    {
        return mahony_set_gyro(&ctx, &gyro_bias);
    }

    /**
     * @brief 获取当前输出。
     *
     * @return 当前输出引用。
     */
    const Output &out() const
    {
        return ctx.out;
    }

    /**
     * @brief 获取当前四元数。
     *
     * @return 当前四元数引用。
     */
    const Quat &quat() const
    {
        return ctx.out.quat;
    }

    /**
     * @brief 获取当前 Euler 角。
     *
     * @return 当前 Euler 角引用。
     */
    const Euler &euler() const
    {
        return ctx.out.euler;
    }
};

} // namespace Algo::Mahony

#endif // PRINTK_ALGORITHM_MAHONY_HPP
