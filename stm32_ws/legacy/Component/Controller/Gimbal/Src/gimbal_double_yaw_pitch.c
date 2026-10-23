/**
 ******************************************************************************
 * @file    gimbal_double_yaw_pitch.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   double-yaw-pitch 云台构型实现。
 *
 * @note    本文件只处理 double 构型私有状态和大 yaw MIT 命令生成。
 * @note    小 yaw 与 pitch 使用直驱核心，运行时数据全部由 GimbalInput 注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "gimbal_double_yaw_pitch.h"

#include <math.h>
#include <string.h>

/** @brief double 构型大 yaw 默认 MIT 位置刚度。 */
#ifndef GIMBAL_DOUBLE_BIG_YAW_KP
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
#define GIMBAL_DOUBLE_BIG_YAW_KP 0.0F
#endif

/** @brief double 构型大 yaw 默认 MIT 速度阻尼。 */
#ifndef GIMBAL_DOUBLE_BIG_YAW_KD
#define GIMBAL_DOUBLE_BIG_YAW_KD 1.0F
#endif

/** @brief double 构型大 yaw 速度命令绝对值上限。 */
#ifndef GIMBAL_DOUBLE_BIG_YAW_MAX_VEL
#define GIMBAL_DOUBLE_BIG_YAW_MAX_VEL 20.0F
#endif

/** @brief double 构型大 yaw 默认力矩命令。 */
#ifndef GIMBAL_DOUBLE_BIG_YAW_TOR
#define GIMBAL_DOUBLE_BIG_YAW_TOR 0.0F
#endif

/** @brief 圆周常量 2*pi。 */
#define GIMBAL_DOUBLE_2PI (6.28318530717958647692F)

/** @brief 圆周常量 pi。 */
#define GIMBAL_DOUBLE_PI (3.14159265358979323846F)

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 判断浮点数是否有限。
 *
 * @param value 输入浮点数。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t double_finite(float32_t value)
{
    return isfinite((double)value) ? 1U : 0U;
}

/**
 * @brief 将角度规整到 [-pi, pi]。
 *
 * @param angle 输入角度，单位 rad。
 * @return 规整后的角度，单位 rad。
 */
static float32_t double_wrap(float32_t angle)
{
    if (double_finite(angle) == 0U) {
        return 0.0F;
    }

    while (angle > GIMBAL_DOUBLE_PI) {
        angle -= GIMBAL_DOUBLE_2PI;
    }

    while (angle < -GIMBAL_DOUBLE_PI) {
        angle += GIMBAL_DOUBLE_2PI;
    }

    return angle;
}

/**
 * @brief 按绝对值限幅。
 *
 * @param value 输入值。
 * @param limit 绝对值上限。
 * @return 限幅后的值。
 */
static float32_t double_limit(float32_t value, float32_t limit)
{
    if (limit < 0.0F) {
        return value;
    }

    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

/**
 * @brief 将控制周期转换为秒。
 *
 * @param gimbal 公共云台上下文。
 * @return 控制周期，单位 s，非法时返回 0。
 */
static float32_t double_period_s(const Gimbal *gimbal)
{
    if ((gimbal == NULL) || (gimbal->cfg.period_ms == 0U)) {
        return 0.0F;
    }

    return (float32_t)gimbal->cfg.period_ms * 0.001F;
}

/**
 * @brief 由单周期目标增量估算参考角速度。
 *
 * @param delta 单周期目标增量，单位 rad。
 * @param period_s 控制周期，单位 s。
 * @return 参考角速度，单位 rad/s。
 */
static float32_t double_ref_rate(float32_t delta, float32_t period_s)
{
    if ((double_finite(delta) == 0U) ||
        (double_finite(period_s) == 0U) ||
        (period_s <= 0.0F)) {
        return 0.0F;
    }

    return delta / period_s;
}

/**
 * @brief 初始化大 yaw PID。
 *
 * @param pid PID 状态。
 * @param cfg PID 参数。
 */
static void double_pid_init(GimbalDirectPid *pid, const GimbalPidCfg *cfg)
{
    if ((pid == NULL) || (cfg == NULL)) {
        return;
    }

    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
}

/**
 * @brief 清零大 yaw PID 动态状态。
 *
 * @param pid PID 状态。
 */
static void double_pid_clear(GimbalDirectPid *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->set = 0.0F;
    pid->get = 0.0F;
    pid->err = 0.0F;
    pid->last_err = 0.0F;
    pid->iout = 0.0F;
    pid->out = 0.0F;
}

/**
 * @brief 计算大 yaw 绝对角 PID 输出。
 *
 * @param pid PID 状态。
 * @param get 当前绝对角，单位 rad。
 * @param set 目标绝对角，单位 rad。
 * @param gyro 当前 yaw 角速度，单位 rad/s。
 * @return 大 yaw 速度命令，单位 rad/s。
 */
static float32_t double_pid_abs(GimbalDirectPid *pid, float32_t get, float32_t set, float32_t gyro)
{
    float32_t pout;
    float32_t dout;

    if ((pid == NULL) ||
        (double_finite(get) == 0U) ||
        (double_finite(set) == 0U) ||
        (double_finite(gyro) == 0U)) {
        return 0.0F;
    }

    pid->get = double_wrap(get);
    pid->set = double_wrap(set);
    pid->err = double_wrap(pid->set - pid->get);
    pout = pid->cfg.kp * pid->err;
    pid->iout += pid->cfg.ki * pid->err;
    pid->iout = double_limit(pid->iout, pid->cfg.max_iout);
    dout = pid->cfg.kd * (-gyro);
    pid->out = double_limit(pout + pid->iout + dout, pid->cfg.max_out);
    pid->last_err = pid->err;
    return double_limit(pid->out, GIMBAL_DOUBLE_BIG_YAW_MAX_VEL);
}

/**
 * @brief 清零大 yaw MIT 命令。
 *
 * @param big 大 yaw 状态。
 */
static void double_big_zero(GimbalDoubleBigYaw *big)
{
    if (big == NULL) {
        return;
    }

    big->cmd.pos = 0.0F;
    big->cmd.vel = 0.0F;
    big->cmd.tor = GIMBAL_DOUBLE_BIG_YAW_TOR;
    big->cmd.kp = GIMBAL_DOUBLE_BIG_YAW_KP;
    big->cmd.kd = GIMBAL_DOUBLE_BIG_YAW_KD;
    big->cmd.enable = 0U;
}

/**
 * @brief 复制输入并让小 yaw 不消费外部 yaw 命令。
 *
 * double 构型中公共 yaw 命令交给大 yaw；小 yaw 只保持直驱核心目标。
 *
 * @param src 原始输入。
 * @param dst 输出给直驱核心的输入。
 */
static void double_core_input(const GimbalInput *src, GimbalInput *dst)
{
    if ((src == NULL) || (dst == NULL)) {
        return;
    }

    *dst = *src;
    dst->cmd.yaw = 0.0F;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
GimbalStatus gimbal_double_init(Gimbal *gimbal, void *impl)
{
    GimbalDoubleCtx *ctx = (GimbalDoubleCtx *)impl;
    GimbalStatus status;

    if ((gimbal == NULL) || (ctx == NULL)) {
        return GimbalNull;
    }

    memset(ctx, 0, sizeof(*ctx));
    status = gimbal_direct_init(gimbal, &ctx->core);
    if (status != GimbalOk) {
        return status;
    }

    double_pid_init(&ctx->big_yaw.abs_pid, &gimbal->cfg.yaw_abs_pid);
    double_big_zero(&ctx->big_yaw);
    return GimbalOk;
}

GimbalStatus gimbal_double_step(Gimbal *gimbal,
                                void *impl,
                                const GimbalInput *input,
                                GimbalOutput *output)
{
    GimbalStatus status;

    if ((gimbal == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    status = gimbal_double_update(gimbal, impl, input);
    if (status != GimbalOk) {
        return status;
    }

    status = gimbal_double_control(gimbal, impl, input);
    if (status != GimbalOk) {
        return status;
    }

    return gimbal_double_output(gimbal, impl, output);
}

GimbalStatus gimbal_double_update(Gimbal *gimbal, void *impl, const GimbalInput *input)
{
    GimbalDoubleCtx *ctx = (GimbalDoubleCtx *)impl;
    GimbalInput core_input;
    GimbalStatus status;

    if ((gimbal == NULL) || (ctx == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    double_core_input(input, &core_input);
    status = gimbal_direct_update(gimbal, &ctx->core, &core_input);
    if (status != GimbalOk) {
        return status;
    }

    if ((input->cmd.online == 0U) ||
        (input->ins_ready == 0U) ||
        (input->big_yaw_fb.online == 0U)) {
        ctx->big_yaw.feedback_ready = 0U;
        double_big_zero(&ctx->big_yaw);
        return GimbalOk;
    }

    ctx->big_yaw.fb = input->big_yaw_fb;
    if (ctx->big_yaw.feedback_ready == 0U) {
        ctx->big_yaw.absolute_set = double_wrap(input->ins_yaw);
        double_pid_clear(&ctx->big_yaw.abs_pid);
        ctx->big_yaw.feedback_ready = 1U;
    }

    return GimbalOk;
}

GimbalStatus gimbal_double_control(Gimbal *gimbal, void *impl, const GimbalInput *input)
{
    GimbalDoubleCtx *ctx = (GimbalDoubleCtx *)impl;
    GimbalInput core_input;
    float32_t correction;
    float32_t period_s;
    float32_t ref_rate;
    float32_t vel;

    if ((gimbal == NULL) || (ctx == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    double_core_input(input, &core_input);
    (void)gimbal_direct_control(gimbal, &ctx->core, &core_input);

    if ((input->cmd.online == 0U) ||
        (input->ins_ready == 0U) ||
        (input->big_yaw_fb.online == 0U) ||
        (input->cmd.mode == GimbalModeZero)) {
        double_big_zero(&ctx->big_yaw);
        return GimbalOk;
    }

    if (input->cmd.mode == GimbalModeRaw) {
        vel = double_limit(input->cmd.yaw, GIMBAL_DOUBLE_BIG_YAW_MAX_VEL);
    } else {
        period_s = double_period_s(gimbal);
        ref_rate = double_ref_rate(input->cmd.yaw, period_s);
        ctx->big_yaw.absolute_set = double_wrap(ctx->big_yaw.absolute_set + input->cmd.yaw);
        correction = double_pid_abs(&ctx->big_yaw.abs_pid,
                                    input->ins_yaw,
                                    ctx->big_yaw.absolute_set,
                                    input->gyro_z);
        correction = double_limit(correction, GIMBAL_YAW_CORR_RATE_MAX);
        vel = double_limit(ref_rate + correction, GIMBAL_DOUBLE_BIG_YAW_MAX_VEL);
    }

    ctx->big_yaw.cmd.pos = 0.0F;
    ctx->big_yaw.cmd.vel = vel;
    ctx->big_yaw.cmd.tor = GIMBAL_DOUBLE_BIG_YAW_TOR;
    ctx->big_yaw.cmd.kp = GIMBAL_DOUBLE_BIG_YAW_KP;
    ctx->big_yaw.cmd.kd = GIMBAL_DOUBLE_BIG_YAW_KD;
    ctx->big_yaw.cmd.enable = 1U;
    return GimbalOk;
}

GimbalStatus gimbal_double_output(const Gimbal *gimbal, void *impl, GimbalOutput *output)
{
    GimbalDoubleCtx *ctx = (GimbalDoubleCtx *)impl;
    GimbalStatus status;

    if ((gimbal == NULL) || (ctx == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    status = gimbal_direct_output(gimbal, &ctx->core, output);
    if (status != GimbalOk) {
        return status;
    }

    output->big_yaw_cmd = ctx->big_yaw.cmd;
    return GimbalOk;
}

GimbalStatus gimbal_double_sync(Gimbal *gimbal, void *impl)
{
    GimbalDoubleCtx *ctx = (GimbalDoubleCtx *)impl;

    if ((gimbal == NULL) || (ctx == NULL)) {
        return GimbalNull;
    }

    (void)gimbal_direct_sync(gimbal, &ctx->core);
    ctx->big_yaw.absolute_set = double_wrap(ctx->big_yaw.fb.pos);
    ctx->big_yaw.feedback_ready = ctx->big_yaw.fb.online;
    double_pid_clear(&ctx->big_yaw.abs_pid);
    double_big_zero(&ctx->big_yaw);
    return GimbalOk;
}

GimbalStatus gimbal_double_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali)
{
    GimbalDoubleCtx *ctx = (GimbalDoubleCtx *)impl;

    if ((gimbal == NULL) || (ctx == NULL) || (cali == NULL)) {
        return GimbalNull;
    }

    return gimbal_direct_set_cali(gimbal, &ctx->core, cali);
}
