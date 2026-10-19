/**
 ******************************************************************************
 * @file    observer_balance.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   平衡腿状态观测实现。
 *
 * @note    本文件只迁移 VMC_calc_1 和 ground_detection 的观测部分，不输出关节力矩。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "observer_balance.h"

#include "observer_common.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 计算几何奇异判断使用的最小阈值。 */
#define OBSERVER_GEOM_EPS 1.0e-6F

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 获取单腿有效位。
 *
 * @param side 腿索引。
 * @return 单腿有效位。
 */
static uint8_t leg_bit(uint32_t side)
{
    return (uint8_t)(1U << side);
}

/**
 * @brief 获取关节角符号。
 *
 * @param sign 配置符号。
 * @return 归一化后的符号。
 */
static float32_t sign_value(int8_t sign)
{
    return (sign < 0) ? -1.0F : 1.0F;
}

/**
 * @brief 检查单腿几何配置是否可用。
 *
 * @param cfg 单腿几何配置。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t leg_cfg_ok(const ObserverLegCfg *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }

    if ((observer_finite(cfg->l1) == 0U) ||
        (observer_finite(cfg->l2) == 0U) ||
        (observer_finite(cfg->l3) == 0U) ||
        (observer_finite(cfg->l4) == 0U) ||
        (observer_finite(cfg->l5) == 0U) ||
        (observer_finite(cfg->phi1_offset) == 0U) ||
        (observer_finite(cfg->phi4_offset) == 0U) ||
        (observer_finite(cfg->pitch_sign) == 0U) ||
        (observer_finite(cfg->gyro_sign) == 0U)) {
        return 0U;
    }

    return ((cfg->l1 > 0.0F) &&
            (cfg->l2 > 0.0F) &&
            (cfg->l3 > 0.0F) &&
            (cfg->l4 > 0.0F)) ? 1U : 0U;
}

/**
 * @brief 从 MIT 反馈计算关节角。
 *
 * @param fb 关节反馈。
 * @param sign 角度符号。
 * @param offset 角度零位偏置。
 * @return 关节角，单位 rad。
 */
static float32_t joint_angle(const ObserverMitFb *fb, int8_t sign, float32_t offset)
{
    return (sign_value(sign) * fb->pos) + offset;
}

/**
 * @brief 检查 MIT 反馈是否可用。
 *
 * @param fb 关节反馈。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t mit_ok(const ObserverMitFb *fb)
{
    if (fb == NULL) {
        return 0U;
    }

    return ((fb->online != 0U) &&
            (observer_finite(fb->pos) != 0U) &&
            (observer_finite(fb->vel) != 0U) &&
            (observer_finite(fb->tor) != 0U)) ? 1U : 0U;
}

/**
 * @brief 根据当前几何和历史值计算差分速度。
 *
 * @param ctx 单腿历史上下文。
 * @param state 单腿输出状态。
 * @param dt 更新间隔，单位 s。
 */
static void update_derivative(ObserverLegCtx *ctx, ObserverLegState *state, float32_t dt)
{
    float32_t d_phi0;

    if (ctx->history_valid == 0U) {
        ctx->last_phi0 = state->phi0;
        ctx->last_length = state->length;
        ctx->last_length_speed = 0.0F;
        ctx->last_theta_speed = state->theta_speed;
        ctx->history_valid = 1U;
        state->length_speed = 0.0F;
        state->length_accel = 0.0F;
        state->alpha_speed = 0.0F;
        state->theta_accel = 0.0F;
        return;
    }

    d_phi0 = (state->phi0 - ctx->last_phi0) / dt;
    state->alpha_speed = -d_phi0;
    state->length_speed = (state->length - ctx->last_length) / dt;
    state->length_accel = (state->length_speed - ctx->last_length_speed) / dt;
    state->theta_accel = (state->theta_speed - ctx->last_theta_speed) / dt;

    ctx->last_phi0 = state->phi0;
    ctx->last_length = state->length;
    ctx->last_length_speed = state->length_speed;
    ctx->last_theta_speed = state->theta_speed;
}

/**
 * @brief 计算单腿五连杆几何观测。
 *
 * @param observer 观测器上下文。
 * @param input 本周期输入。
 * @param side 腿索引。
 * @param out 单腿输出状态。
 * @return 执行状态。
 */
static ObserverStatus calc_leg(Observer *observer,
                               const ObserverInput *input,
                               uint32_t side,
                               ObserverLegState *out)
{
    const ObserverLegCfg *cfg;
    const ObserverMitFb *front;
    const ObserverMitFb *back;
    ObserverLegCtx *ctx;
    float32_t phi1;
    float32_t phi4;
    float32_t xb;
    float32_t yb;
    float32_t xd;
    float32_t yd;
    float32_t xc;
    float32_t yc;
    float32_t lbd;
    float32_t a0;
    float32_t b0;
    float32_t c0;
    float32_t disc;
    float32_t phi2;
    float32_t phi3;
    float32_t d_phi0;
    float32_t pitch;
    float32_t pitch_gyro;

    cfg = &observer->cfg.leg[side];
    front = &input->joint_fb[side * 2U];
    back = &input->joint_fb[(side * 2U) + 1U];
    ctx = &observer->leg_ctx[side];

    observer_clear_leg(out);
    if ((leg_cfg_ok(cfg) == 0U) || (mit_ok(front) == 0U) || (mit_ok(back) == 0U)) {
        ctx->history_valid = 0U;
        return ObserverOk;
    }

    phi1 = joint_angle(front, cfg->phi1_sign, cfg->phi1_offset);
    phi4 = joint_angle(back, cfg->phi4_sign, cfg->phi4_offset);

    yd = cfg->l4 * arm_sin_f32(phi4);
    yb = cfg->l1 * arm_sin_f32(phi1);
    xd = cfg->l5 + (cfg->l4 * arm_cos_f32(phi4));
    xb = cfg->l1 * arm_cos_f32(phi1);

    lbd = sqrtf(((xd - xb) * (xd - xb)) + ((yd - yb) * (yd - yb)));
    a0 = 2.0F * cfg->l2 * (xd - xb);
    b0 = 2.0F * cfg->l2 * (yd - yb);
    c0 = (cfg->l2 * cfg->l2) + (lbd * lbd) - (cfg->l3 * cfg->l3);
    disc = (a0 * a0) + (b0 * b0) - (c0 * c0);
    if ((observer_finite(disc) == 0U) || (disc < 0.0F)) {
        ctx->history_valid = 0U;
        return ObserverGeometry;
    }

    phi2 = 2.0F * atan2f(b0 + sqrtf(disc), a0 + c0);
    phi3 = atan2f(yb - yd + (cfg->l2 * arm_sin_f32(phi2)),
                  xb - xd + (cfg->l2 * arm_cos_f32(phi2)));
    xc = (cfg->l1 * arm_cos_f32(phi1)) + (cfg->l2 * arm_cos_f32(phi2));
    yc = (cfg->l1 * arm_sin_f32(phi1)) + (cfg->l2 * arm_sin_f32(phi2));

    out->length = sqrtf(((xc - (cfg->l5 * 0.5F)) * (xc - (cfg->l5 * 0.5F))) + (yc * yc));
    if ((observer_finite(out->length) == 0U) || (out->length <= OBSERVER_GEOM_EPS)) {
        ctx->history_valid = 0U;
        observer_clear_leg(out);
        return ObserverGeometry;
    }

    out->phi0 = atan2f(yc, xc - (cfg->l5 * 0.5F));
    out->alpha = (COMMON_PI * 0.5F) - out->phi0;

    d_phi0 = (ctx->history_valid != 0U) ? ((out->phi0 - ctx->last_phi0) / input->imu.dt) : 0.0F;
    pitch = cfg->pitch_sign * observer->state.ins.euler.pitch;
    pitch_gyro = cfg->gyro_sign * observer->state.ins.gyro.x;
    out->theta = (COMMON_PI * 0.5F) - pitch - out->phi0;
    out->theta_speed = -pitch_gyro - d_phi0;

    update_derivative(ctx, out, input->imu.dt);

    if ((input->leg_force_valid_mask & leg_bit(side)) != 0U) {
        if ((observer_finite(input->leg_force[side]) == 0U) ||
            (observer_finite(input->leg_torque[side]) == 0U)) {
            ctx->history_valid = 0U;
            observer_clear_leg(out);
            return ObserverValue;
        }

        out->support_force =
            (input->leg_force[side] * arm_cos_f32(out->theta)) +
            ((input->leg_torque[side] * arm_sin_f32(out->theta)) / out->length) +
            observer->cfg.wheel_weight;
        out->grounded = (out->support_force >= observer->cfg.ground_threshold) ? 1U : 0U;
    }

    if ((observer_finite(out->theta) == 0U) ||
        (observer_finite(out->theta_speed) == 0U) ||
        (observer_finite(out->length_speed) == 0U) ||
        (observer_finite(out->length_accel) == 0U)) {
        ctx->history_valid = 0U;
        observer_clear_leg(out);
        return ObserverGeometry;
    }

    out->valid = 1U;
    (void)phi3;
    return ObserverOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ObserverStatus observer_balance_init(Observer *observer)
{
    if (observer == NULL) {
        return ObserverNull;
    }

    return observer_balance_reset(observer);
}

ObserverStatus observer_balance_reset(Observer *observer)
{
    if (observer == NULL) {
        return ObserverNull;
    }

    (void)memset(observer->leg_ctx, 0, sizeof(observer->leg_ctx));
    observer_clear_balance(&observer->state.balance);
    observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_BALANCE);

    return ObserverOk;
}

ObserverStatus observer_balance_update(Observer *observer, const ObserverInput *input)
{
    uint32_t side;
    uint8_t bit;
    uint8_t valid_mask = 0U;
    uint8_t grounded_mask = 0U;
    ObserverStatus status;

    if ((observer == NULL) || (input == NULL)) {
        return ObserverNull;
    }

    if (observer->initialized == 0U) {
        return ObserverUninit;
    }

    if ((input->imu.dt <= 0.0F) || (observer_finite(input->imu.dt) == 0U)) {
        observer_balance_reset(observer);
        return ObserverValue;
    }

    if ((observer->state.ins.valid == 0U) || (observer->state.ins.ready == 0U)) {
        observer_balance_reset(observer);
        return ObserverOk;
    }

    observer_clear_balance(&observer->state.balance);
    for (side = 0U; side < OBSERVER_LEG_NUM; ++side) {
        bit = leg_bit(side);
        status = calc_leg(observer, input, side, &observer->state.balance.leg[side]);
        if (status != ObserverOk) {
            observer_balance_reset(observer);
            return status;
        }

        if (observer->state.balance.leg[side].valid != 0U) {
            valid_mask |= bit;
            if (observer->state.balance.leg[side].grounded != 0U) {
                grounded_mask |= bit;
            }
        }
    }

    observer->state.balance.valid_mask = valid_mask;
    observer->state.balance.grounded_mask = grounded_mask;
    observer->state.balance.valid = (valid_mask != 0U) ? 1U : 0U;
    observer->state.balance.timestamp_us =
        (input->timestamp_us != 0U) ? input->timestamp_us : input->imu.timestamp_us;

    if (valid_mask != 0U) {
        observer->state.valid_mask |= OBSERVER_VALID_BALANCE;
    } else {
        observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_BALANCE);
    }

    return ObserverOk;
}
