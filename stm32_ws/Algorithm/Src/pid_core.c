/**
 ******************************************************************************
 * @file    pid_core.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   标准离散 PID 核心实现。
 *
 * @note    本文件只实现 PID 主干差分方程，具体策略通过 hook 注入。
 *
 * @note    所有状态由调用者提供的 pid_core_t 保存，内部不申请动态内存。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "pid_core.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ==========================================================================
 * PID core 内部校验与默认 hook
 * --------------------------------------------------------------------------
 * core 只维护离散 PID 主干，默认 err/I/D 策略作为 hook 缺省实现存在。
 * ========================================================================== */
/**
 * @brief 判断 float32_t 是否为有限值。
 *
 * @param value 待检查数值。
 * @return 非 0 表示有限，0 表示 NaN 或 Inf。
 */
static uint8_t pid_finite(float32_t value)
{
    return isfinite((double)value) ? 1U : 0U;
}

/**
 * @brief 默认线性误差 err = set - get。
 *
 * @param set 设定值。
 * @param get 反馈值。
 * @return 线性误差。
 */
static float32_t pid_default_err(float32_t set, float32_t get)
{
    return set - get;
}

/**
 * @brief 默认积分更新 I += ki * err。
 *
 * @param pid PID 核心上下文。
 * @param err 本次误差。
 */
static void pid_default_i(pid_core_t *pid, float32_t err)
{
    pid->iout += pid->ki * err;
}

/**
 * @brief 默认微分项 D = kd * (err - last_err)。
 *
 * @param pid PID 核心上下文。
 * @param err 本次误差。
 * @return 微分项输出。
 */
static float32_t pid_default_d(const pid_core_t *pid, float32_t err)
{
    return pid->kd * (err - pid->err[1]);
}

/* ==========================================================================
 * PID 主干接口
 * --------------------------------------------------------------------------
 * 这一组是 PID 的事实来源；策略只通过 hook 改变误差、积分和微分语义。
 * ========================================================================== */
float32_t pid_core_limit(float32_t value, float32_t limit)
{
    if (limit < 0.0F)
    {
        return value;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

pid_core_status_t pid_core_init(pid_core_t *pid, const pid_core_cfg_t *cfg)
{
    if ((pid == NULL) || (cfg == NULL))
    {
        return PID_CORE_ERR_ARG;
    }

    if ((pid_finite(cfg->kp) == 0U) ||
        (pid_finite(cfg->ki) == 0U) ||
        (pid_finite(cfg->kd) == 0U) ||
        (pid_finite(cfg->max_out) == 0U) ||
        (pid_finite(cfg->max_iout) == 0U) ||
        (cfg->max_out < 0.0F) ||
        (cfg->max_iout < 0.0F))
    {
        return PID_CORE_ERR_VALUE;
    }

    memset(pid, 0, sizeof(*pid));
    pid->kp = cfg->kp;
    pid->ki = cfg->ki;
    pid->kd = cfg->kd;
    pid->max_out = cfg->max_out;
    pid->max_iout = cfg->max_iout;
    pid->ops = cfg->ops;
    pid->arg = cfg->arg;

    return PID_CORE_OK;
}

pid_core_status_t pid_core_reset(pid_core_t *pid)
{
    if (pid == NULL)
    {
        return PID_CORE_ERR_ARG;
    }

    pid->set = 0.0F;
    pid->get = 0.0F;
    pid->err[0] = 0.0F;
    pid->err[1] = 0.0F;
    pid->err[2] = 0.0F;
    pid->pout = 0.0F;
    pid->iout = 0.0F;
    pid->dout = 0.0F;
    pid->ff = 0.0F;
    pid->out = 0.0F;

    return PID_CORE_OK;
}

pid_core_status_t pid_core_step(pid_core_t *pid,
                                const pid_core_in_t *in,
                                pid_core_out_t *out)
{
    float32_t err;
    const pid_core_ops_t *ops;

    if ((pid == NULL) || (in == NULL))
    {
        return PID_CORE_ERR_ARG;
    }

    if ((pid_finite(in->set) == 0U) ||
        (pid_finite(in->get) == 0U) ||
        (pid_finite(in->d_in) == 0U) ||
        (pid_finite(in->ff) == 0U))
    {
        pid->out = 0.0F;
        return PID_CORE_ERR_VALUE;
    }

    ops = pid->ops;
    err = (ops != NULL && ops->err != NULL) ?
              ops->err(in->set, in->get, pid->arg) :
              pid_default_err(in->set, in->get);

    if (pid_finite(err) == 0U)
    {
        pid->out = 0.0F;
        return PID_CORE_ERR_VALUE;
    }

    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->err[0] = err;
    pid->set = in->set;
    pid->get = in->get;
    pid->ff = in->ff;
    pid->pout = pid->kp * err;

    if (ops != NULL && ops->i != NULL)
    {
        ops->i(pid, err, in, pid->arg);
    }
    else
    {
        pid_default_i(pid, err);
    }
    pid->iout = pid_core_limit(pid->iout, pid->max_iout);

    pid->dout = (ops != NULL && ops->d != NULL) ?
                    ops->d(pid, err, in, pid->arg) :
                    pid_default_d(pid, err);

    pid->out = pid->pout + pid->iout + pid->dout + pid->ff;
    pid->out = pid_core_limit(pid->out, pid->max_out);

    if (out != NULL)
    {
        out->err = pid->err[0];
        out->p = pid->pout;
        out->i = pid->iout;
        out->d = pid->dout;
        out->ff = pid->ff;
        out->raw = pid->pout + pid->iout + pid->dout + pid->ff;
        out->out = pid->out;
    }

    return PID_CORE_OK;
}
