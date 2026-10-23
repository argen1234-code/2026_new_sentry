/**
 ******************************************************************************
 * @file    angle_pid.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-20
 * @brief   角度环 PID 组合实现。
 *
 * @note    短路径：本次计算把 set 临时调整到 actual 附近，持久 target 不变。
 * @note    死区：输出 0、冻结积分、清 P/D 和误差历史，离开死区不产生冲击。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "angle_pid.h"

#include <math.h>
#include <stddef.h>

/** @brief 圆周常量 pi。 */
#define ANGLE_PID_PI (3.14159265358979323846F)
/** @brief 圆周常量 2*pi。 */
#define ANGLE_PID_2PI (6.28318530717958647692F)

static uint8_t angle_pid_finite(float32_t value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

pid_core_status_t angle_pid_init(angle_pid_t *pid, const angle_pid_cfg_t *cfg)
{
    pid_cfg_t core_cfg;

    if ((pid == NULL) || (cfg == NULL)) {
        return PID_CORE_ERR_ARG;
    }

    if ((angle_pid_finite(cfg->kp) == 0U) ||
        (angle_pid_finite(cfg->ki) == 0U) ||
        (angle_pid_finite(cfg->kd) == 0U) ||
        (angle_pid_finite(cfg->max_out) == 0U) ||
        (angle_pid_finite(cfg->max_iout) == 0U) ||
        (angle_pid_finite(cfg->deadband) == 0U) ||
        (cfg->max_out < 0.0F) || (cfg->max_iout < 0.0F) ||
        (cfg->deadband < 0.0F)) {
        return PID_CORE_ERR_VALUE;
    }

    core_cfg.kp = cfg->kp;
    core_cfg.ki = cfg->ki;
    core_cfg.kd = cfg->kd;
    core_cfg.max_out = cfg->max_out;
    core_cfg.max_iout = cfg->max_iout;
    core_cfg.mode = PID_ANGLE;
    core_cfg.angle.freeze_err = cfg->freeze_err;
    core_cfg.angle.freeze_d = cfg->freeze_d;
    core_cfg.angle.active_err = cfg->active_err;
    core_cfg.angle.leak = cfg->leak;

    if (pid_init(&pid->pid, &core_cfg) != PID_CORE_OK) {
        return PID_CORE_ERR_VALUE;
    }

    pid->cfg = *cfg;
    pid->target = 0.0F;
    pid->in_deadband = 0U;
    return PID_CORE_OK;
}

pid_core_status_t angle_pid_reset(angle_pid_t *pid)
{
    if (pid == NULL) {
        return PID_CORE_ERR_ARG;
    }

    pid->target = 0.0F;
    pid->in_deadband = 0U;
    return pid_reset(&pid->pid);
}

void angle_pid_set_target(angle_pid_t *pid, float32_t target)
{
    if ((pid != NULL) && (angle_pid_finite(target) != 0U)) {
        pid->target = target;
    }
}

float32_t angle_pid_get_target(const angle_pid_t *pid)
{
    return (pid != NULL) ? pid->target : 0.0F;
}

float32_t angle_pid_wrap(float32_t angle)
{
    float32_t wrapped;

    if (angle_pid_finite(angle) == 0U) {
        return 0.0F;
    }

    wrapped = fmodf(angle + ANGLE_PID_PI, ANGLE_PID_2PI);
    if (wrapped < 0.0F) {
        wrapped += ANGLE_PID_2PI;
    }
    return wrapped - ANGLE_PID_PI;
}

float32_t angle_pid_calc(angle_pid_t *pid, float32_t actual)
{
    return angle_pid_calc_d(pid, actual, 0.0F);
}

float32_t angle_pid_calc_d(angle_pid_t *pid, float32_t actual, float32_t d_in)
{
    float32_t err;
    pid_in_t in;
    pid_out_t out;

    if ((pid == NULL) || (angle_pid_finite(actual) == 0U) ||
        (angle_pid_finite(d_in) == 0U)) {
        return 0.0F;
    }

    err = angle_pid_wrap(pid->target - actual);

    if (fabsf(err) <= pid->cfg.deadband) {
        pid->in_deadband = 1U;
        /* 冻结积分：不调用 pid_step；清 P/D 和误差历史，离开死区不产生冲击。 */
        pid->pid.core.err[0] = err;
        pid->pid.core.err[1] = err;
        pid->pid.core.err[2] = err;
        pid->pid.core.pout = 0.0F;
        pid->pid.core.dout = 0.0F;
        pid->pid.core.out = 0.0F;
        return 0.0F;
    }

    pid->in_deadband = 0U;

    /*
     * 短路径：把本次 set 临时调整到 actual 附近。持久 target 不改。
     * pid_angle 的误差 hook 会再次 wrap，结果仍是 err。
     */
    in.set = actual + err;
    in.get = actual;
    in.d_in = d_in;
    in.ff = 0.0F;

    if (pid_step(&pid->pid, &in, &out) != PID_CORE_OK) {
        return 0.0F;
    }
    return out.out;
}
