/**
 ******************************************************************************
 * @file    ballistic.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   弹道补偿算法实现。
 *
 * @note    本文件迁移旧弹道模块的可用算法事实，并移除无上限循环和全局参数。
 * @note    所有计算均为调用方传入配置的纯函数，不进行运行时分配。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "ballistic.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* ==========================================================================
 * 内部数值参数
 * --------------------------------------------------------------------------
 * 这些常量只限制搜索范围和数值微分步长，不表达具体机器人配置。
 * ========================================================================== */

/** @brief pitch 搜索范围，避免迭代发散到不合理角度。 */
#define BALLISTIC_PITCH_LIMIT (1.45F)

/** @brief 数值导数使用的角度扰动，单位 rad。 */
#define BALLISTIC_DERIV_DELTA (0.001F)

/** @brief 弹丸落得过低时提前停止积分，单位 m。 */
#define BALLISTIC_DROP_LIMIT (-20.0F)

/* ==========================================================================
 * 参数校验
 * --------------------------------------------------------------------------
 * 弹道计算对距离、速度、重力和迭代参数先做有限值检查，非法输入不继续迭代。
 * ========================================================================== */
/**
 * @brief 判断浮点数是否为有限值。
 *
 * @param x 待检查数值。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t ballistic_finite(float32_t x)
{
    return ((x == x) && (x <= FLT_MAX) && (x >= -FLT_MAX)) ? 1U : 0U;
}

/**
 * @brief 浮点绝对值。
 *
 * @param x 输入值。
 * @return 绝对值。
 */
static float32_t ballistic_abs(float32_t x)
{
    return (x >= 0.0F) ? x : -x;
}

/**
 * @brief 限制数值范围。
 *
 * @param x 输入值。
 * @param lo 下限。
 * @param hi 上限。
 * @return 限幅后数值。
 */
static float32_t ballistic_clamp(float32_t x, float32_t lo, float32_t hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/**
 * @brief 准备配置，传入空指针时使用默认配置。
 *
 * @param cfg 用户配置。
 * @param out 有效配置输出。
 * @return 执行状态。
 */
static BallisticStatus ballistic_prepare_cfg(const BallisticCfg *cfg, BallisticCfg *out)
{
    if (out == NULL) {
        return BallisticNull;
    }

    if (cfg == NULL) {
        return ballistic_default_cfg(out);
    }

    *out = *cfg;
    return BallisticOk;
}

/**
 * @brief 检查配置是否可用于数值积分。
 *
 * @param cfg 配置。
 * @return 合法返回 1，否则返回 0。
 */
static uint8_t ballistic_cfg_ok(const BallisticCfg *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }

    if ((ballistic_finite(cfg->gravity) == 0U) ||
        (ballistic_finite(cfg->drag_k) == 0U) ||
        (ballistic_finite(cfg->muzzle_speed) == 0U) ||
        (ballistic_finite(cfg->muzzle_forward) == 0U) ||
        (ballistic_finite(cfg->muzzle_up) == 0U) ||
        (ballistic_finite(cfg->dt) == 0U) ||
        (ballistic_finite(cfg->eps) == 0U) ||
        (ballistic_finite(cfg->min_distance) == 0U)) {
        return 0U;
    }

    if ((cfg->gravity <= 0.0F) ||
        (cfg->drag_k < 0.0F) ||
        (cfg->muzzle_speed <= 0.0F) ||
        (cfg->dt <= 0.0F) ||
        (cfg->eps <= 0.0F) ||
        (cfg->min_distance <= 0.0F) ||
        (cfg->max_steps == 0U) ||
        (cfg->max_iter == 0U)) {
        return 0U;
    }

    return 1U;
}

/* ==========================================================================
 * 空气阻力迭代
 * --------------------------------------------------------------------------
 * 迭代过程有最大步数和高度下限，避免旧实现中无界循环影响控制周期。
 * ========================================================================== */
/**
 * @brief 在指定 pitch 下积分弹道，并插值到目标水平距离。
 *
 * 积分时显式限制最大步数；当 x 跨过目标距离时，使用上一点与当前点做线性插值，
 * 避免积分步长对高度误差造成明显台阶。
 *
 * @param horizontal 目标水平距离。
 * @param pitch 发射 pitch。
 * @param cfg 配置。
 * @param out_z 目标距离处高度输出。
 * @param out_time 飞行时间输出。
 * @return 执行状态。
 */
static BallisticStatus ballistic_simulate(float32_t horizontal,
                                          float32_t pitch,
                                          const BallisticCfg *cfg,
                                          float32_t *out_z,
                                          float32_t *out_time)
{
    float32_t x = 0.0F;
    float32_t z = 0.0F;
    float32_t t = 0.0F;
    float32_t vx;
    float32_t vz;
    uint16_t i;

    if ((cfg == NULL) || (out_z == NULL) || (out_time == NULL)) {
        return BallisticNull;
    }
    if ((ballistic_finite(horizontal) == 0U) ||
        (ballistic_finite(pitch) == 0U) ||
        (horizontal <= cfg->min_distance)) {
        return BallisticArg;
    }

    vx = cfg->muzzle_speed * cosf(pitch);
    vz = cfg->muzzle_speed * sinf(pitch);
    if ((ballistic_finite(vx) == 0U) ||
        (ballistic_finite(vz) == 0U) ||
        (vx <= cfg->eps)) {
        return BallisticUnreach;
    }

    for (i = 0U; i < cfg->max_steps; i++) {
        float32_t prev_x = x;
        float32_t prev_z = z;
        float32_t prev_t = t;
        float32_t speed = sqrtf((vx * vx) + (vz * vz));
        float32_t drag;

        if ((ballistic_finite(speed) == 0U) || (speed <= cfg->eps)) {
            return BallisticValue;
        }

        drag = cfg->drag_k * speed;
        vx -= drag * vx * cfg->dt;
        vz -= (cfg->gravity + (drag * vz)) * cfg->dt;

        if ((ballistic_finite(vx) == 0U) ||
            (ballistic_finite(vz) == 0U) ||
            (vx <= cfg->eps)) {
            return BallisticUnreach;
        }

        x += vx * cfg->dt;
        z += vz * cfg->dt;
        t += cfg->dt;

        if ((ballistic_finite(x) == 0U) ||
            (ballistic_finite(z) == 0U) ||
            (ballistic_finite(t) == 0U)) {
            return BallisticValue;
        }

        if (x >= horizontal) {
            float32_t ratio = (horizontal - prev_x) / (x - prev_x);

            ratio = ballistic_clamp(ratio, 0.0F, 1.0F);
            *out_z = prev_z + ((z - prev_z) * ratio);
            *out_time = prev_t + ((t - prev_t) * ratio);
            return BallisticOk;
        }

        if (z < BALLISTIC_DROP_LIMIT) {
            return BallisticUnreach;
        }
    }

    return BallisticUnreach;
}

/* ==========================================================================
 * 默认配置与无阻力解
 * --------------------------------------------------------------------------
 * 默认配置只保证函数可用；真实弹速、枪口偏置和阻力参数应由 Robot target 注入。
 * ========================================================================== */
BallisticStatus ballistic_default_cfg(BallisticCfg *cfg)
{
    if (cfg == NULL) {
        return BallisticNull;
    }

    cfg->gravity = BALLISTIC_GRAVITY_DEFAULT;
    cfg->drag_k = BALLISTIC_DRAG_K_DEFAULT;
    cfg->muzzle_speed = BALLISTIC_MUZZLE_SPEED_DEFAULT;
    cfg->muzzle_forward = BALLISTIC_MUZZLE_FORWARD_DEFAULT;
    cfg->muzzle_up = BALLISTIC_MUZZLE_UP_DEFAULT;
    cfg->dt = BALLISTIC_DT_DEFAULT;
    cfg->eps = BALLISTIC_EPS_DEFAULT;
    cfg->min_distance = BALLISTIC_MIN_DISTANCE_DEFAULT;
    cfg->max_steps = BALLISTIC_MAX_STEPS_DEFAULT;
    cfg->max_iter = BALLISTIC_MAX_ITER_DEFAULT;
    return BallisticOk;
}

BallisticStatus ballistic_drag_pitch(float32_t horizontal,
                                     float32_t vertical,
                                     const BallisticCfg *cfg,
                                     BallisticResult *out)
{
    BallisticCfg local_cfg;
    BallisticStatus status;
    float32_t angle;
    float32_t best_angle;
    float32_t best_z = 0.0F;
    float32_t best_time = 0.0F;
    float32_t best_err = FLT_MAX;
    uint8_t have_best = 0U;
    uint8_t i;

    if (out == NULL) {
        return BallisticNull;
    }
    (void)memset(out, 0, sizeof(*out));

    status = ballistic_prepare_cfg(cfg, &local_cfg);
    if (status != BallisticOk) {
        return status;
    }
    if (ballistic_cfg_ok(&local_cfg) == 0U) {
        return BallisticArg;
    }
    if ((ballistic_finite(horizontal) == 0U) ||
        (ballistic_finite(vertical) == 0U) ||
        (horizontal <= local_cfg.min_distance)) {
        return BallisticArg;
    }

    angle = ballistic_clamp(atan2f(vertical, horizontal),
                            -BALLISTIC_PITCH_LIMIT,
                            BALLISTIC_PITCH_LIMIT);
    best_angle = angle;

    for (i = 0U; i < local_cfg.max_iter; i++) {
        float32_t z;
        float32_t t;
        float32_t error;
        float32_t z_deriv;
        float32_t t_deriv;
        float32_t derivative;
        float32_t step;

        status = ballistic_simulate(horizontal, angle, &local_cfg, &z, &t);
        if (status != BallisticOk) {
            break;
        }

        error = vertical - z;
        if (ballistic_abs(error) < best_err) {
            best_err = ballistic_abs(error);
            best_angle = angle;
            best_z = z;
            best_time = t;
            have_best = 1U;
        }

        if (best_err <= local_cfg.eps) {
            break;
        }

        status = ballistic_simulate(horizontal,
                                    angle + BALLISTIC_DERIV_DELTA,
                                    &local_cfg,
                                    &z_deriv,
                                    &t_deriv);
        (void)t_deriv;
        if (status != BallisticOk) {
            break;
        }

        derivative = (z_deriv - z) / BALLISTIC_DERIV_DELTA;
        if ((ballistic_finite(derivative) == 0U) ||
            (ballistic_abs(derivative) <= 1.0e-6F)) {
            break;
        }

        step = error / derivative;
        if (ballistic_finite(step) == 0U) {
            return BallisticValue;
        }

        angle = ballistic_clamp(angle + step,
                                -BALLISTIC_PITCH_LIMIT,
                                BALLISTIC_PITCH_LIMIT);
    }

    if (have_best == 0U) {
        return BallisticUnreach;
    }

    out->pitch = best_angle;
    out->flight_time = best_time;
    out->impact_z = best_z;
    out->error_z = best_z - vertical;
    out->reachable = 1U;
    return BallisticOk;
}

/* ==========================================================================
 * 视觉距离与枪口偏置封装
 * --------------------------------------------------------------------------
 * 公开求解接口把视觉坐标距离、枪口偏置和补偿角组合成有限状态返回。
 * ========================================================================== */
BallisticStatus ballistic_pitch(float32_t shoot_angle,
                                float32_t distance,
                                const BallisticCfg *cfg,
                                BallisticResult *out)
{
    BallisticCfg local_cfg;
    BallisticStatus status;
    float32_t horizontal;
    float32_t vertical;

    if (out == NULL) {
        return BallisticNull;
    }
    (void)memset(out, 0, sizeof(*out));

    status = ballistic_prepare_cfg(cfg, &local_cfg);
    if (status != BallisticOk) {
        return status;
    }
    if (ballistic_cfg_ok(&local_cfg) == 0U) {
        return BallisticArg;
    }
    if ((ballistic_finite(shoot_angle) == 0U) ||
        (ballistic_finite(distance) == 0U) ||
        (distance <= local_cfg.min_distance)) {
        return BallisticArg;
    }

    horizontal = (distance * cosf(shoot_angle)) + local_cfg.muzzle_forward;
    vertical = (distance * sinf(shoot_angle)) + local_cfg.muzzle_up;
    if ((ballistic_finite(horizontal) == 0U) ||
        (ballistic_finite(vertical) == 0U) ||
        (horizontal <= local_cfg.min_distance)) {
        return BallisticArg;
    }

    return ballistic_drag_pitch(horizontal, vertical, &local_cfg, out);
}
