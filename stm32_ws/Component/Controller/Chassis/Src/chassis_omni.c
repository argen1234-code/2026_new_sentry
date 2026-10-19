/**
 ******************************************************************************
 * @file    chassis_omni.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   全向轮底盘构型实现。
 *
 * @note    本文件只使用 ChassisInput 注入的数据，不读取任务、遥控器、CAN 或裁判系统。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "chassis_omni.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 1 / sqrt(2)，用于全向轮速度投影。 */
#define CHASSIS_OMNI_INV_SQRT2 0.7071067812F

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 计算浮点绝对值。
 *
 * @param value 输入值。
 * @return 绝对值。
 */
static float32_t chassis_omni_abs(float32_t value)
{
    return (value >= 0.0F) ? value : -value;
}

/**
 * @brief 选择配置值或默认值。
 *
 * @param value 配置值。
 * @param default_value 默认值。
 * @return 配置值非 0 时返回配置值，否则返回默认值。
 */
static float32_t chassis_omni_pick(float32_t value, float32_t default_value)
{
    return (value != 0.0F) ? value : default_value;
}

/**
 * @brief 将数值限制在正负边界内。
 *
 * @param value 输入值。
 * @param limit 绝对值边界。
 * @return 限幅后的值。
 */
static float32_t chassis_omni_clamp(float32_t value, float32_t limit)
{
    if (limit <= 0.0F) {
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
 * @brief 将浮点电流转换为 int16_t。
 *
 * @param value 浮点电流。
 * @return int16_t 电流。
 */
static int16_t chassis_omni_i16(float32_t value)
{
    if (value > 32767.0F) {
        return 32767;
    }

    if (value < -32768.0F) {
        return -32768;
    }

    return (int16_t)value;
}

/**
 * @brief 读取极性配置。
 *
 * @param polarity 配置极性。
 * @return 有效极性。
 */
static int8_t chassis_omni_pol(int8_t polarity)
{
    return (polarity == 0) ? 1 : polarity;
}

/**
 * @brief 为 PID 配置补默认值。
 *
 * @param cfg PID 配置。
 * @param kp 默认 KP。
 * @param ki 默认 KI。
 * @param kd 默认 KD。
 * @param max_out 默认输出限幅。
 */
static void chassis_omni_pid_default(ChassisPidCfg *cfg,
                                     float32_t kp,
                                     float32_t ki,
                                     float32_t kd,
                                     float32_t max_out)
{
    if ((cfg->kp == 0.0F) && (cfg->ki == 0.0F) && (cfg->kd == 0.0F)) {
        cfg->kp = kp;
        cfg->ki = ki;
        cfg->kd = kd;
    }
    cfg->max_out = chassis_omni_pick(cfg->max_out, max_out);
}

/**
 * @brief 初始化 PID 状态。
 *
 * @param pid PID 状态。
 * @param cfg PID 配置。
 */
static void chassis_omni_pid_init(ChassisOmniPid *pid, const ChassisPidCfg *cfg)
{
    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
}

/**
 * @brief 清空 PID 动态状态。
 *
 * @param pid PID 状态。
 */
static void chassis_omni_pid_reset(ChassisOmniPid *pid)
{
    pid->set = 0.0F;
    pid->get = 0.0F;
    pid->err = 0.0F;
    pid->last_err = 0.0F;
    pid->iout = 0.0F;
    pid->out = 0.0F;
}

/**
 * @brief 执行位置式 PID 计算。
 *
 * @param pid PID 状态。
 * @param get 反馈值。
 * @param set 目标值。
 * @return PID 输出。
 */
static float32_t chassis_omni_pid_calc(ChassisOmniPid *pid, float32_t get, float32_t set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    pid->iout += pid->cfg.ki * pid->err;
    pid->iout = chassis_omni_clamp(pid->iout, pid->cfg.max_iout);
    pid->out = (pid->cfg.kp * pid->err) + pid->iout + (pid->cfg.kd * (pid->err - pid->last_err));
    pid->out = chassis_omni_clamp(pid->out, pid->cfg.max_out);
    pid->last_err = pid->err;

    return pid->out;
}

/**
 * @brief 归一化全向轮配置。
 *
 * @param cfg 底盘配置。
 */
static void chassis_omni_cfg(ChassisCfg *cfg)
{
    uint32_t i;

    cfg->drive_rpm_to_speed = chassis_omni_pick(cfg->drive_rpm_to_speed, CHASSIS_RPM_TO_SPEED_DEFAULT);
    cfg->max_drive_speed = chassis_omni_pick(cfg->max_drive_speed, CHASSIS_WHEEL_SPEED_MAX_DEFAULT);
    cfg->max_drive_current = chassis_omni_pick(cfg->max_drive_current, CHASSIS_CURRENT_MAX_DEFAULT);
    cfg->power_limit = chassis_omni_pick(cfg->power_limit, CHASSIS_POWER_LIMIT_DEFAULT);
    cfg->omni_center = chassis_omni_pick(cfg->omni_center, CHASSIS_CENTER_DIST_DEFAULT);
    cfg->wz_scale = chassis_omni_pick(cfg->wz_scale, CHASSIS_WZ_SCALE_DEFAULT);
    chassis_omni_pid_default(&cfg->drive_pid,
                             CHASSIS_DRIVE_PID_KP_DEFAULT,
                             CHASSIS_DRIVE_PID_KI_DEFAULT,
                             CHASSIS_DRIVE_PID_KD_DEFAULT,
                             CHASSIS_DRIVE_PID_MAX_DEFAULT);
    chassis_omni_pid_default(&cfg->yaw_pid,
                             CHASSIS_YAW_PID_KP_DEFAULT,
                             0.0F,
                             CHASSIS_YAW_PID_KD_DEFAULT,
                             CHASSIS_YAW_PID_MAX_DEFAULT);
    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        cfg->drive_polarity[i] = chassis_omni_pol(cfg->drive_polarity[i]);
    }
}

/**
 * @brief 清空全向轮输出状态。
 *
 * @param ctx 全向轮上下文。
 */
static void chassis_omni_zero(ChassisOmniCtx *ctx)
{
    uint32_t i;

    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        ctx->wheel[i].speed_set = 0.0F;
        ctx->wheel[i].current = 0;
        ctx->current[i] = 0;
    }
    ctx->mode = ChassisModeStop;
}

/**
 * @brief 判断四个驱动反馈是否在线。
 *
 * @param ctx 全向轮上下文。
 * @return 全部在线返回 1，否则返回 0。
 */
static uint8_t chassis_omni_online(const ChassisOmniCtx *ctx)
{
    uint32_t i;

    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        if (ctx->wheel[i].online == 0U) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 计算功率缩放系数。
 *
 * @param cfg 底盘配置。
 * @param input 周期输入。
 * @return 输出缩放系数。
 */
static float32_t chassis_omni_power_scale(ChassisOmniCtx *ctx,
                                          const ChassisCfg *cfg,
                                          const ChassisInput *input,
                                          const float32_t current[CHASSIS_OMNI_WHEEL_NUM])
{
    PowerCtrlInput power;
    uint32_t i;

    if ((ctx == NULL) || (cfg == NULL) || (input == NULL) || (current == NULL)) {
        return 1.0F;
    }

    (void)memset(&power, 0, sizeof(power));
    power.motor_count = CHASSIS_OMNI_WHEEL_NUM;
    power.power_limit = (input->power_limit > 0.0F) ? input->power_limit : cfg->power_limit;
    power.measured_power = input->power;
    power.buffer_energy = input->buffer_energy;
    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        power.motor[i].speed_rpm = (float32_t)input->drive_fb[i].speed_rpm;
        power.motor[i].current = current[i];
        power.motor[i].online = input->drive_fb[i].online;
    }

    return power_ctrl_scale(&ctx->power, &power);
}

/**
 * @brief 计算 yaw 跟随角速度。
 *
 * @param ctx 全向轮上下文。
 * @param input 周期输入。
 * @return yaw 闭环补偿角速度。
 */
static float32_t chassis_omni_yaw(ChassisOmniCtx *ctx, const ChassisInput *input)
{
    if ((input->cmd.mode == ChassisModeFollowGimbalYaw) || (input->cmd.mode == ChassisModeFollowChassisYaw)) {
        return chassis_omni_pid_calc(&ctx->yaw_pid, input->yaw, input->cmd.yaw_ref);
    }

    return 0.0F;
}

/**
 * @brief 全向轮运动学逆解。
 *
 * @param cfg 底盘配置。
 * @param input 周期输入。
 * @param wz 角速度。
 * @param wheel_speed 四轮目标速度。
 */
static void chassis_omni_kin(const ChassisCfg *cfg, const ChassisInput *input, float32_t wz, float32_t wheel_speed[4])
{
    float32_t sin_ang = sinf(input->cmd.frame_angle);
    float32_t cos_ang = cosf(input->cmd.frame_angle);
    float32_t speed_x = input->cmd.vx;
    float32_t speed_y = input->cmd.vy;
    float32_t rot = (cfg->wz_scale + 1.0F) * cfg->omni_center * wz;

    wheel_speed[0] = ((-cos_ang - sin_ang) * speed_x + (-sin_ang + cos_ang) * speed_y + rot) * CHASSIS_OMNI_INV_SQRT2;
    wheel_speed[1] = ((-cos_ang + sin_ang) * speed_x + (-sin_ang - cos_ang) * speed_y + rot) * CHASSIS_OMNI_INV_SQRT2;
    wheel_speed[2] = ((cos_ang + sin_ang) * speed_x + (sin_ang - cos_ang) * speed_y + rot) * CHASSIS_OMNI_INV_SQRT2;
    wheel_speed[3] = ((cos_ang - sin_ang) * speed_x + (sin_ang + cos_ang) * speed_y + rot) * CHASSIS_OMNI_INV_SQRT2;
}

/**
 * @brief 限制四轮最大速度。
 *
 * @param cfg 底盘配置。
 * @param wheel_speed 四轮目标速度。
 */
static void chassis_omni_limit_speed(const ChassisCfg *cfg, float32_t wheel_speed[4])
{
    float32_t max_speed = 0.0F;
    uint32_t i;

    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        float32_t abs_speed = chassis_omni_abs(wheel_speed[i]);
        if (abs_speed > max_speed) {
            max_speed = abs_speed;
        }
    }

    if (max_speed > cfg->max_drive_speed) {
        float32_t rate = cfg->max_drive_speed / max_speed;
        for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
            wheel_speed[i] *= rate;
        }
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ChassisStatus chassis_omni_init(Chassis *chassis, void *impl)
{
    ChassisOmniCtx *ctx = (ChassisOmniCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL)) {
        return ChassisNull;
    }

    chassis_omni_cfg(&chassis->cfg);
    memset(ctx, 0, sizeof(*ctx));
    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        chassis_omni_pid_init(&ctx->drive_pid[i], &chassis->cfg.drive_pid);
    }
    chassis_omni_pid_init(&ctx->yaw_pid, &chassis->cfg.yaw_pid);
    if (power_ctrl_init(&ctx->power, &chassis->cfg.power_cfg) != PowerCtrlOk) {
        return ChassisArg;
    }
    ctx->mode = ChassisModeStop;
    ctx->ready = 1U;

    return ChassisOk;
}

ChassisStatus chassis_omni_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output)
{
    ChassisStatus status;

    if ((chassis == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    status = chassis_omni_update(chassis, impl, input);
    if (status != ChassisOk) {
        return status;
    }

    status = chassis_omni_control(chassis, impl, input);
    if (status != ChassisOk) {
        return status;
    }

    return chassis_omni_output(chassis, impl, output);
}

ChassisStatus chassis_omni_update(Chassis *chassis, void *impl, const ChassisInput *input)
{
    ChassisOmniCtx *ctx = (ChassisOmniCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        float32_t speed = input->drive_fb[i].speed;
        if (speed == 0.0F) {
            speed = (float32_t)input->drive_fb[i].speed_rpm * chassis->cfg.drive_rpm_to_speed;
        }
        ctx->wheel[i].speed = speed;
        ctx->wheel[i].online = input->drive_fb[i].online;
    }

    return ChassisOk;
}

ChassisStatus chassis_omni_control(Chassis *chassis, void *impl, const ChassisInput *input)
{
    ChassisOmniCtx *ctx = (ChassisOmniCtx *)impl;
    ChassisCfg *cfg;
    float32_t wheel_speed[4];
    float32_t current_raw[CHASSIS_OMNI_WHEEL_NUM];
    float32_t scale;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    cfg = &chassis->cfg;
    chassis_omni_zero(ctx);

    if ((input->cmd.online == 0U) || (input->cmd.force_stop != 0U) || (input->cmd.mode == ChassisModeStop)) {
        return ChassisOk;
    }

    ctx->mode = input->cmd.mode;

    if (input->cmd.mode == ChassisModeRaw) {
        for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
            float32_t current = (float32_t)input->cmd.raw_drive_current[i] * (float32_t)cfg->drive_polarity[i];
            current_raw[i] = chassis_omni_clamp(current, cfg->max_drive_current);
        }
        scale = chassis_omni_power_scale(ctx, cfg, input, current_raw);
        for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
            ctx->current[i] = chassis_omni_i16(current_raw[i] * scale);
            ctx->wheel[i].current = ctx->current[i];
        }
        return ChassisOk;
    }

    if (chassis_omni_online(ctx) == 0U) {
        chassis_omni_zero(ctx);
        return ChassisState;
    }

    chassis_omni_kin(cfg, input, input->cmd.wz + chassis_omni_yaw(ctx, input), wheel_speed);
    chassis_omni_limit_speed(cfg, wheel_speed);

    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        float32_t current;

        ctx->wheel[i].speed_set = wheel_speed[i];
        current = chassis_omni_pid_calc(&ctx->drive_pid[i], ctx->wheel[i].speed, ctx->wheel[i].speed_set);
        current = chassis_omni_clamp(current, cfg->max_drive_current);
        current_raw[i] = current * (float32_t)cfg->drive_polarity[i];
    }

    scale = chassis_omni_power_scale(ctx, cfg, input, current_raw);
    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        ctx->current[i] = chassis_omni_i16(current_raw[i] * scale);
        ctx->wheel[i].current = ctx->current[i];
    }

    return ChassisOk;
}

ChassisStatus chassis_omni_output(const Chassis *chassis, void *impl, ChassisOutput *output)
{
    const ChassisOmniCtx *ctx = (const ChassisOmniCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    (void)chassis;
    memset(output, 0, sizeof(*output));
    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        output->drive_current[i] = ctx->current[i];
        output->drive_speed_set[i] = ctx->wheel[i].speed_set;
    }
    output->mode = ctx->mode;
    output->send_enable = 1U;

    return ChassisOk;
}

ChassisStatus chassis_omni_sync(Chassis *chassis, void *impl)
{
    ChassisOmniCtx *ctx = (ChassisOmniCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    for (i = 0U; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        chassis_omni_pid_reset(&ctx->drive_pid[i]);
        ctx->wheel[i].speed_set = ctx->wheel[i].speed;
    }
    chassis_omni_pid_reset(&ctx->yaw_pid);

    return ChassisOk;
}
