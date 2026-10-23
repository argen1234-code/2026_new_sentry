/**
 ******************************************************************************
 * @file    mahony.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   Mahony 姿态解算实现。
 *
 * @note    本文件按原始 DM-balanceV1 的 mahony_filter.c 迁移，保留旋转矩阵、
 *          误差叉乘、四元数积分和 Euler 输出公式。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "mahony.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/** @brief 默认比例增益。 */
#ifndef MAHONY_DEFAULT_KP
#define MAHONY_DEFAULT_KP 1.0F
#endif

/** @brief 默认积分增益。 */
#ifndef MAHONY_DEFAULT_KI
#define MAHONY_DEFAULT_KI 0.0F
#endif

/** @brief 默认更新周期，单位 s。 */
#ifndef MAHONY_DEFAULT_DT
#define MAHONY_DEFAULT_DT 0.001F
#endif

/** @brief 数值零阈值。 */
#ifndef MAHONY_EPS
#define MAHONY_EPS 1.0e-8F
#endif

/* ==========================================================================
 * 输入校验
 * --------------------------------------------------------------------------
 * Mahony 入口先检查有限值和向量有效性，避免 NaN/Inf 进入姿态积分。
 * ========================================================================== */
/**
 * @brief 判断 float32_t 是否有限。
 *
 * @param value 输入值。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t mahony_finite(float32_t value)
{
    return (isfinite((double)value) != 0) ? 1U : 0U;
}

/**
 * @brief 判断三维向量是否全部为有限值。
 *
 * @param value 输入向量。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t mahony_vec_ok(const Vec3 *value)
{
    if (value == NULL) {
        return 0U;
    }

    return ((mahony_finite(value->x) != 0U) &&
            (mahony_finite(value->y) != 0U) &&
            (mahony_finite(value->z) != 0U)) ? 1U : 0U;
}

/**
 * @brief 将浮点数限制到反三角函数安全输入范围。
 *
 * 四元数归一化后旋转矩阵元素理论上应处于 [-1, 1]，但浮点舍入可能
 * 产生极小越界；这里夹紧以避免 asinf 返回 NaN。
 *
 * @param value 输入值。
 * @return 限幅后的值。
 */
static float32_t mahony_clamp_unit(float32_t value)
{
    if (value > 1.0F) {
        return 1.0F;
    }

    if (value < -1.0F) {
        return -1.0F;
    }

    return value;
}

/* ==========================================================================
 * 旋转矩阵与 Euler 更新
 * --------------------------------------------------------------------------
 * 四元数是主状态，旋转矩阵和 Euler 只作为下游读取与调试输出缓存。
 * ========================================================================== */
/**
 * @brief 使用当前四元数更新旋转矩阵。
 *
 * @param ctx Mahony 上下文。
 */
static void mahony_update_rmat(Mahony *ctx)
{
    const float32_t q0 = ctx->out.quat.w;
    const float32_t q1 = ctx->out.quat.x;
    const float32_t q2 = ctx->out.quat.y;
    const float32_t q3 = ctx->out.quat.z;
    const float32_t q1q1 = q1 * q1;
    const float32_t q2q2 = q2 * q2;
    const float32_t q3q3 = q3 * q3;
    const float32_t q0q1 = q0 * q1;
    const float32_t q0q2 = q0 * q2;
    const float32_t q0q3 = q0 * q3;
    const float32_t q1q2 = q1 * q2;
    const float32_t q1q3 = q1 * q3;
    const float32_t q2q3 = q2 * q3;

    ctx->out.rmat[0] = 1.0F - (2.0F * q2q2) - (2.0F * q3q3);
    ctx->out.rmat[1] = 2.0F * (q1q2 - q0q3);
    ctx->out.rmat[2] = 2.0F * (q1q3 + q0q2);

    ctx->out.rmat[3] = 2.0F * (q1q2 + q0q3);
    ctx->out.rmat[4] = 1.0F - (2.0F * q1q1) - (2.0F * q3q3);
    ctx->out.rmat[5] = 2.0F * (q2q3 - q0q1);

    ctx->out.rmat[6] = 2.0F * (q1q3 - q0q2);
    ctx->out.rmat[7] = 2.0F * (q2q3 + q0q1);
    ctx->out.rmat[8] = 1.0F - (2.0F * q1q1) - (2.0F * q2q2);
}

/**
 * @brief 按原始实现从旋转矩阵提取 Euler。
 *
 * @param ctx Mahony 上下文。
 */
static void mahony_update_euler(Mahony *ctx)
{
    ctx->out.euler.pitch = -asinf(mahony_clamp_unit(ctx->out.rmat[6]));
    ctx->out.euler.roll = atan2f(ctx->out.rmat[7], ctx->out.rmat[8]);
    ctx->out.euler.yaw = atan2f(ctx->out.rmat[3], ctx->out.rmat[0]);
}

/* ==========================================================================
 * 配置与生命周期
 * --------------------------------------------------------------------------
 * 初始化、复位和配置检查保持纯算法语义，不读取 IMU 设备或温控状态。
 * ========================================================================== */
/**
 * @brief 校验配置。
 *
 * @param cfg Mahony 配置。
 * @return 校验状态。
 */
static MahonyStatus mahony_check_cfg(const MahonyCfg *cfg)
{
    if (cfg == NULL) {
        return MahonyNull;
    }

    if ((mahony_finite(cfg->kp) == 0U) ||
        (mahony_finite(cfg->ki) == 0U) ||
        (mahony_finite(cfg->dt) == 0U)) {
        return MahonyValue;
    }

    if ((cfg->kp < 0.0F) || (cfg->ki < 0.0F) || (cfg->dt <= 0.0F)) {
        return MahonySize;
    }

    return MahonyOk;
}

MahonyStatus mahony_init(Mahony *ctx, const MahonyCfg *cfg)
{
    MahonyCfg local_cfg;
    MahonyStatus status;

    if (ctx == NULL) {
        return MahonyNull;
    }

    if (cfg == NULL) {
        local_cfg.kp = MAHONY_DEFAULT_KP;
        local_cfg.ki = MAHONY_DEFAULT_KI;
        local_cfg.dt = MAHONY_DEFAULT_DT;
        cfg = &local_cfg;
    }

    status = mahony_check_cfg(cfg);
    if (status != MahonyOk) {
        return status;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->initialized = 1U;

    return mahony_reset(ctx);
}

MahonyStatus mahony_reset(Mahony *ctx)
{
    if (ctx == NULL) {
        return MahonyNull;
    }

    memset(&ctx->out, 0, sizeof(ctx->out));
    ctx->out.quat.w = 1.0F;
    ctx->out.quat.x = 0.0F;
    ctx->out.quat.y = 0.0F;
    ctx->out.quat.z = 0.0F;
    ctx->ex_int = 0.0F;
    ctx->ey_int = 0.0F;
    ctx->ez_int = 0.0F;
    ctx->gyro_sum.x = 0.0F;
    ctx->gyro_sum.y = 0.0F;
    ctx->gyro_sum.z = 0.0F;
    ctx->gyro_count = 0U;
    mahony_update_rmat(ctx);
    mahony_update_euler(ctx);

    return MahonyOk;
}

/* ==========================================================================
 * 积分更新
 * --------------------------------------------------------------------------
 * 姿态更新只消费 gyro/accel/dt，零偏和积分项都保存在调用方提供的上下文中。
 * ========================================================================== */
MahonyStatus mahony_update(Mahony *ctx, const MahonyInput *input)
{
    Vec3 gyro;
    Vec3 acc;
    Quat q_last;
    float32_t norm;
    float32_t ex;
    float32_t ey;
    float32_t ez;
    float32_t half_t;
    float32_t dt;

    if ((ctx == NULL) || (input == NULL)) {
        return MahonyNull;
    }

    if (ctx->initialized == 0U) {
        return MahonySize;
    }

    dt = (input->dt > 0.0F) ? input->dt : ctx->cfg.dt;
    if ((dt <= 0.0F) || (mahony_finite(dt) == 0U)) {
        return MahonySize;
    }

    if ((mahony_vec_ok(&input->gyro) == 0U) || (mahony_vec_ok(&input->accel) == 0U)) {
        return MahonyValue;
    }

    gyro = input->gyro;
    acc = input->accel;
    gyro.x -= ctx->out.gyro_bias.x;
    gyro.y -= ctx->out.gyro_bias.y;
    gyro.z -= ctx->out.gyro_bias.z;
    ctx->out.gyro_z_bias = ctx->out.gyro_bias.z;

    norm = sqrtf((acc.x * acc.x) + (acc.y * acc.y) + (acc.z * acc.z));
    if ((norm <= MAHONY_EPS) || (mahony_finite(norm) == 0U)) {
        return MahonySingular;
    }

    ctx->out.accel_norm = norm;
    acc.x /= norm;
    acc.y /= norm;
    acc.z /= norm;

    /*
     * 使用 rMat 第 3 行作为当前重力方向估计，与加速度单位向量做叉乘。
     */
    ex = (acc.y * ctx->out.rmat[8]) - (acc.z * ctx->out.rmat[7]);
    ey = (acc.z * ctx->out.rmat[6]) - (acc.x * ctx->out.rmat[8]);
    ez = (acc.x * ctx->out.rmat[7]) - (acc.y * ctx->out.rmat[6]);

    ctx->ex_int += ctx->cfg.ki * ex * dt;
    ctx->ey_int += ctx->cfg.ki * ey * dt;
    ctx->ez_int += ctx->cfg.ki * ez * dt;

    gyro.x += (ctx->cfg.kp * ex) + ctx->ex_int;
    gyro.y += (ctx->cfg.kp * ey) + ctx->ey_int;
    gyro.z += (ctx->cfg.kp * ez) + ctx->ez_int;

    q_last = ctx->out.quat;
    half_t = 0.5F * dt;
    ctx->out.quat.w += ((-q_last.x * gyro.x) - (q_last.y * gyro.y) - (q_last.z * gyro.z)) * half_t;
    ctx->out.quat.x += ((q_last.w * gyro.x) + (q_last.y * gyro.z) - (q_last.z * gyro.y)) * half_t;
    ctx->out.quat.y += ((q_last.w * gyro.y) - (q_last.x * gyro.z) + (q_last.z * gyro.x)) * half_t;
    ctx->out.quat.z += ((q_last.w * gyro.z) + (q_last.x * gyro.y) - (q_last.y * gyro.x)) * half_t;

    if (quat_normalize(&ctx->out.quat, &ctx->out.quat) != CommonOk) {
        return MahonyValue;
    }

    ctx->out.gyro = gyro;
    mahony_update_rmat(ctx);
    mahony_update_euler(ctx);

    return MahonyOk;
}

/* ==========================================================================
 * 零偏校准与输出读取
 * --------------------------------------------------------------------------
 * 零偏校准接口只改算法上下文；App 层负责决定何时采样、保存和应用。
 * ========================================================================== */
MahonyStatus mahony_cali_gyro_z(Mahony *ctx, float32_t gyro_z, uint32_t samples)
{
    Vec3 gyro;

    gyro.x = 0.0F;
    gyro.y = 0.0F;
    gyro.z = gyro_z;

    return mahony_cali_gyro(ctx, &gyro, samples);
}

MahonyStatus mahony_cali_gyro(Mahony *ctx, const Vec3 *gyro, uint32_t samples)
{
    if (ctx == NULL) {
        return MahonyNull;
    }

    if ((samples == 0U) || (ctx->initialized == 0U)) {
        return MahonySize;
    }

    if (mahony_vec_ok(gyro) == 0U) {
        return MahonyValue;
    }

    if (ctx->gyro_count < samples) {
        ctx->gyro_sum.x += gyro->x;
        ctx->gyro_sum.y += gyro->y;
        ctx->gyro_sum.z += gyro->z;
        ctx->gyro_count++;
        ctx->out.gyro_bias.x = ctx->gyro_sum.x / (float32_t)ctx->gyro_count;
        ctx->out.gyro_bias.y = ctx->gyro_sum.y / (float32_t)ctx->gyro_count;
        ctx->out.gyro_bias.z = ctx->gyro_sum.z / (float32_t)ctx->gyro_count;
        ctx->out.gyro_z_bias = ctx->out.gyro_bias.z;
    }

    return MahonyOk;
}

MahonyStatus mahony_set_gyro_z(Mahony *ctx, float32_t gyro_z_bias)
{
    Vec3 bias;

    if (ctx == NULL) {
        return MahonyNull;
    }

    bias = ctx->out.gyro_bias;
    bias.z = gyro_z_bias;

    return mahony_set_gyro(ctx, &bias);
}

MahonyStatus mahony_set_gyro(Mahony *ctx, const Vec3 *gyro_bias)
{
    if (ctx == NULL) {
        return MahonyNull;
    }

    if ((ctx->initialized == 0U) || (mahony_vec_ok(gyro_bias) == 0U)) {
        return MahonyValue;
    }

    ctx->out.gyro_bias = *gyro_bias;
    ctx->out.gyro_z_bias = gyro_bias->z;

    return MahonyOk;
}

MahonyStatus mahony_get(const Mahony *ctx, MahonyOutput *out)
{
    if ((ctx == NULL) || (out == NULL)) {
        return MahonyNull;
    }

    *out = ctx->out;
    return MahonyOk;
}

const char *mahony_status_str(MahonyStatus status)
{
    switch (status) {
    case MahonyOk:
        return "MahonyOk";

    case MahonyNull:
        return "MahonyNull";

    case MahonySize:
        return "MahonySize";

    case MahonyValue:
        return "MahonyValue";

    case MahonySingular:
        return "MahonySingular";

    default:
        return "MahonyUnknown";
    }
}
