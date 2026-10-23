/**
 ******************************************************************************
 * @file    pid_angle.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   角度 PID 策略实现。
 *
 * @note    角度策略提供 wrap 误差、外部微分和云台当前使用的积分泄漏/活动区逻辑。
 *
 * @note    PID 主干仍由 pid_core_step() 执行。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "pid_angle.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 圆周常量 pi。
 */
#define PID_ANGLE_PI (3.14159265358979323846F)

/**
 * @brief 圆周常量 2*pi。
 */
#define PID_ANGLE_2PI (6.28318530717958647692F)

/* ==========================================================================
 * 角度策略 hook
 * --------------------------------------------------------------------------
 * 角度 PID 只重定义误差环绕、积分泄漏和外部微分语义，主干仍由 pid_core 执行。
 * ========================================================================== */
/**
 * @brief 将角度归一化到 [-pi, pi)。
 *
 * @param angle 输入角度，单位 rad。
 * @return 归一化后的角度，单位 rad。
 */
static float32_t angle_wrap(float32_t angle)
{
    while (angle >= PID_ANGLE_PI)
    {
        angle -= PID_ANGLE_2PI;
    }

    while (angle < -PID_ANGLE_PI)
    {
        angle += PID_ANGLE_2PI;
    }

    return angle;
}

/**
 * @brief 角度误差 hook。
 *
 * @param set 设定角度。
 * @param get 反馈角度。
 * @param arg 用户上下文，当前未使用。
 * @return wrap 后角度误差。
 */
static float32_t angle_err(float32_t set, float32_t get, void *arg)
{
    (void)arg;
    return angle_wrap(set - get);
}

/**
 * @brief 角度积分 hook。
 *
 * @param pid PID 核心上下文。
 * @param err 本次角度误差。
 * @param in 本次输入。
 * @param arg pid_angle_t 参数。
 */
static void angle_i(pid_core_t *pid,
                    float32_t err,
                    const pid_core_in_t *in,
                    void *arg)
{
    const pid_angle_t *cfg;

    if ((pid == NULL) || (in == NULL) || (arg == NULL))
    {
        return;
    }

    cfg = (const pid_angle_t *)arg;
    if ((fabsf(err) < cfg->freeze_err) && (fabsf(in->d_in) < cfg->freeze_d))
    {
        pid->iout *= cfg->leak;
    }
    else if ((err > 0.05F) && (fabsf(err) < cfg->active_err))
    {
        pid->iout += pid->ki * err;
    }
}

/**
 * @brief 外部微分 hook。
 *
 * @param pid PID 核心上下文。
 * @param err 本次误差，当前未使用。
 * @param in 本次输入。
 * @param arg 用户上下文，当前未使用。
 * @return 微分项输出。
 */
static float32_t angle_d(const pid_core_t *pid,
                         float32_t err,
                         const pid_core_in_t *in,
                         void *arg)
{
    (void)err;
    (void)arg;

    if ((pid == NULL) || (in == NULL))
    {
        return 0.0F;
    }

    return pid->kd * in->d_in;
}

/**
 * @brief 角度 PID hook 表。
 */
static const pid_core_ops_t angle_ops = {
    angle_err,
    angle_i,
    angle_d,
};

const pid_core_ops_t *pid_angle_ops(void)
{
    return &angle_ops;
}
