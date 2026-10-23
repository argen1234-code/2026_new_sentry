/**
 ******************************************************************************
 * @file    pid.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   PID 控制器 C 语言组合接口实现。
 *
 * @note    本文件只组合 pid_core 的 hook 策略，不重复实现 PID 主干方程。
 *
 * @note    业务层若需要新的 trick，应新增 hook 组合，而不是复制一套 PID。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "pid.h"

#include <stddef.h>

/* ==========================================================================
 * 组合接口
 * --------------------------------------------------------------------------
 * 组合层只把线性/角度策略装配进 pid_core，不复制 PID 主干公式。
 * ========================================================================== */
pid_core_status_t pid_init(pid_ctrl_t *pid, const pid_cfg_t *cfg)
{
    pid_core_cfg_t core;

    if ((pid == NULL) || (cfg == NULL))
    {
        return PID_CORE_ERR_ARG;
    }

    if ((cfg->mode != PID_LINEAR) && (cfg->mode != PID_ANGLE))
    {
        return PID_CORE_ERR_VALUE;
    }

    if (cfg->mode == PID_ANGLE)
    {
        if (!(cfg->angle.freeze_err >= 0.0F) ||
            !(cfg->angle.freeze_d >= 0.0F) ||
            !(cfg->angle.active_err >= 0.0F) ||
            !(cfg->angle.leak >= 0.0F) ||
            !(cfg->angle.leak <= 1.0F))
        {
            return PID_CORE_ERR_VALUE;
        }
    }

    pid->mode = cfg->mode;
    pid->angle = cfg->angle;

    core.kp = cfg->kp;
    core.ki = cfg->ki;
    core.kd = cfg->kd;
    core.max_out = cfg->max_out;
    core.max_iout = cfg->max_iout;
    core.ops = (cfg->mode == PID_ANGLE) ? pid_angle_ops() : pid_linear_ops();
    core.arg = (cfg->mode == PID_ANGLE) ? (void *)&pid->angle : NULL;

    return pid_core_init(&pid->core, &core);
}

pid_core_status_t pid_reset(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return PID_CORE_ERR_ARG;
    }

    return pid_core_reset(&pid->core);
}

pid_core_status_t pid_step(pid_ctrl_t *pid,
                           const pid_in_t *in,
                           pid_out_t *out)
{
    pid_core_in_t core_in;

    if ((pid == NULL) || (in == NULL))
    {
        return PID_CORE_ERR_ARG;
    }

    core_in.set = in->set;
    core_in.get = in->get;
    core_in.d_in = in->d_in;
    core_in.ff = in->ff;

    return pid_core_step(&pid->core, &core_in, out);
}

float32_t pid_calc(pid_ctrl_t *pid, float32_t get, float32_t set)
{
    pid_in_t in;

    in.set = set;
    in.get = get;
    in.d_in = 0.0F;
    in.ff = 0.0F;

    if (pid_step(pid, &in, NULL) != PID_CORE_OK)
    {
        return 0.0F;
    }

    return pid->core.out;
}

float32_t pid_calc_d(pid_ctrl_t *pid,
                     float32_t get,
                     float32_t set,
                     float32_t d_in)
{
    pid_in_t in;

    in.set = set;
    in.get = get;
    in.d_in = d_in;
    in.ff = 0.0F;

    if (pid_step(pid, &in, NULL) != PID_CORE_OK)
    {
        return 0.0F;
    }

    return pid->core.out;
}
