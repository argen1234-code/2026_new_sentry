/**
 ******************************************************************************
 * @file    pid.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   PID 控制器 C++ 零成本策略包装。
 *
 * @note    本文件只做固定策略的 C++ 包装，运行时实现位于 Src/pid_core.c 和 Src/pid.c。
 *
 * @note    策略由模板参数固定，不使用虚函数、RTTI 或动态内存。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_PID_HPP
#define PRINTK_ALGORITHM_PID_HPP

#include "Src/pid.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只提供模板策略和 inline 转发，运行时事实仍在 Algorithm/Src 的 C 实现中。
 * ========================================================================== */
namespace Algo::Pid {

/** @brief PID 执行状态。 */
using Status = pid_core_status_t;

/** @brief PID 输出项。 */
using Out = pid_out_t;

/** @brief 执行成功。 */
inline constexpr Status Ok = PID_CORE_OK;

/** @brief 线性位置式 PID 策略标签。 */
struct Linear {
    /** @brief 获取 C 策略模式。 */
    static constexpr pid_mode_t mode()
    {
        return PID_LINEAR;
    }
};

/** @brief 角度 PID 策略标签。 */
struct Angle {
    /** @brief 默认积分泄漏误差阈值。 */
    float32_t freeze_err;

    /** @brief 默认积分泄漏外部微分阈值。 */
    float32_t freeze_d;

    /** @brief 默认积分活动误差上限。 */
    float32_t active_err;

    /** @brief 默认积分泄漏系数。 */
    float32_t leak;

    /** @brief 获取 C 策略模式。 */
    static constexpr pid_mode_t mode()
    {
        return PID_ANGLE;
    }
};

/**
 * @brief PID 参数。
 */
struct Param {
    float32_t kp;       /**< 比例增益。 */
    float32_t ki;       /**< 积分增益。 */
    float32_t kd;       /**< 微分增益。 */
    float32_t max_out;  /**< 输出绝对值限幅。 */
    float32_t max_iout; /**< 积分项绝对值限幅。 */
};

/* ==========================================================================
 * 模板策略接口
 * --------------------------------------------------------------------------
 * Policy 在编译期决定 PID 模式，不引入虚函数、RTTI 或动态分派。
 * ========================================================================== */
/**
 * @brief 固定策略 PID 控制器。
 *
 * @tparam Policy PID 策略标签。
 */
template<typename Policy>
struct Control {
    /** @brief C 控制器上下文。 */
    pid_ctrl_t ctx{};

    /**
     * @brief 初始化线性 PID。
     *
     * @param param PID 参数。
     * @return 初始化状态。
     */
    Status init(const Param &param)
    {
        pid_cfg_t cfg{};

        cfg.kp = param.kp;
        cfg.ki = param.ki;
        cfg.kd = param.kd;
        cfg.max_out = param.max_out;
        cfg.max_iout = param.max_iout;
        cfg.mode = Policy::mode();

        return pid_init(&ctx, &cfg);
    }

    /**
     * @brief 初始化带策略参数的 PID。
     *
     * @param param PID 参数。
     * @param policy PID 策略参数。
     * @return 初始化状态。
     */
    Status init(const Param &param, const Policy &policy)
    {
        pid_cfg_t cfg{};

        cfg.kp = param.kp;
        cfg.ki = param.ki;
        cfg.kd = param.kd;
        cfg.max_out = param.max_out;
        cfg.max_iout = param.max_iout;
        cfg.mode = Policy::mode();

        if constexpr (Policy::mode() == PID_ANGLE)
        {
            cfg.angle.freeze_err = policy.freeze_err;
            cfg.angle.freeze_d = policy.freeze_d;
            cfg.angle.active_err = policy.active_err;
            cfg.angle.leak = policy.leak;
        }

        return pid_init(&ctx, &cfg);
    }

    /** @brief 清除动态状态。 */
    Status reset()
    {
        return pid_reset(&ctx);
    }

    /** @brief 执行一次 set/get 更新。 */
    float32_t step(float32_t get, float32_t set)
    {
        return pid_calc(&ctx, get, set);
    }

    /** @brief 执行一次 set/get/外部微分更新。 */
    float32_t step(float32_t get, float32_t set, float32_t d_in)
    {
        return pid_calc_d(&ctx, get, set, d_in);
    }

    /** @brief 获取最近一次输出。 */
    float32_t out() const
    {
        return ctx.core.out;
    }

    /** @brief 获取最近一次误差。 */
    float32_t err() const
    {
        return ctx.core.err[0];
    }
};

} // namespace Algo::Pid

#endif // PRINTK_ALGORITHM_PID_HPP
