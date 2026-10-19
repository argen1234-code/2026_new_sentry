/**
 ******************************************************************************
 * @file    chassis_steering.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   舵轮底盘构型实现。
 *
 * @note    本文件只使用 ChassisInput 注入的数据，不读取任务、遥控器、CAN 或裁判系统。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "chassis_steering.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 圆周率。 */
#define CHASSIS_PI 3.1415926536F

/** @brief 2 倍圆周率。 */
#define CHASSIS_2PI 6.2831853072F

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
static float32_t chassis_steering_abs(float32_t value)
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
static float32_t chassis_steering_pick(float32_t value, float32_t default_value)
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
static float32_t chassis_steering_clamp(float32_t value, float32_t limit)
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
static int16_t chassis_steering_i16(float32_t value)
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
static int8_t chassis_steering_pol(int8_t polarity)
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
static void chassis_steering_pid_default(ChassisPidCfg *cfg,
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
    cfg->max_out = chassis_steering_pick(cfg->max_out, max_out);
}

/**
 * @brief 将角度约束到 [-pi, pi]。
 *
 * @param angle 输入角度。
 * @return 约束后的角度。
 */
static float32_t chassis_steering_wrap(float32_t angle)
{
    while (angle > CHASSIS_PI) {
        angle -= CHASSIS_2PI;
    }

    while (angle < -CHASSIS_PI) {
        angle += CHASSIS_2PI;
    }

    return angle;
}

/**
 * @brief 初始化 PID 状态。
 *
 * @param pid PID 状态。
 * @param cfg PID 配置。
 */
static void chassis_steering_pid_init(ChassisSteeringPid *pid, const ChassisPidCfg *cfg)
{
    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
}

/**
 * @brief 清空 PID 动态状态。
 *
 * @param pid PID 状态。
 */
static void chassis_steering_pid_reset(ChassisSteeringPid *pid)
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
static float32_t chassis_steering_pid_calc(ChassisSteeringPid *pid, float32_t get, float32_t set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    pid->iout += pid->cfg.ki * pid->err;
    pid->iout = chassis_steering_clamp(pid->iout, pid->cfg.max_iout);
    pid->out = (pid->cfg.kp * pid->err) + pid->iout + (pid->cfg.kd * (pid->err - pid->last_err));
    pid->out = chassis_steering_clamp(pid->out, pid->cfg.max_out);
    pid->last_err = pid->err;

    return pid->out;
}

/**
 * @brief 归一化舵轮配置。
 *
 * @param cfg 底盘配置。
 */
static void chassis_steering_cfg(ChassisCfg *cfg)
{
    uint32_t i;

    cfg->drive_rpm_to_speed = chassis_steering_pick(cfg->drive_rpm_to_speed, CHASSIS_RPM_TO_SPEED_DEFAULT);
    cfg->steer_rpm_to_speed = chassis_steering_pick(cfg->steer_rpm_to_speed, CHASSIS_STEER_RPM_TO_SPEED_DEFAULT);
    cfg->steer_ecd_to_rad = chassis_steering_pick(cfg->steer_ecd_to_rad, CHASSIS_STEER_ECD_TO_RAD_DEFAULT);
    cfg->max_drive_speed = chassis_steering_pick(cfg->max_drive_speed, CHASSIS_WHEEL_SPEED_MAX_DEFAULT);
    cfg->max_drive_current = chassis_steering_pick(cfg->max_drive_current, CHASSIS_CURRENT_MAX_DEFAULT);
    cfg->max_steer_current = chassis_steering_pick(cfg->max_steer_current, CHASSIS_STEER_CURRENT_MAX_DEFAULT);
    cfg->power_limit = chassis_steering_pick(cfg->power_limit, CHASSIS_POWER_LIMIT_DEFAULT);
    cfg->half_length = chassis_steering_pick(cfg->half_length, CHASSIS_HALF_LENGTH_DEFAULT);
    cfg->half_width = chassis_steering_pick(cfg->half_width, CHASSIS_HALF_WIDTH_DEFAULT);
    chassis_steering_pid_default(&cfg->drive_pid,
                                 CHASSIS_DRIVE_PID_KP_DEFAULT,
                                 CHASSIS_DRIVE_PID_KI_DEFAULT,
                                 CHASSIS_DRIVE_PID_KD_DEFAULT,
                                 CHASSIS_DRIVE_PID_MAX_DEFAULT);
    chassis_steering_pid_default(&cfg->steer_angle_pid,
                                 CHASSIS_STEER_ANGLE_KP_DEFAULT,
                                 0.0F,
                                 0.0F,
                                 CHASSIS_STEER_ANGLE_MAX_DEFAULT);
    chassis_steering_pid_default(&cfg->steer_speed_pid,
                                 CHASSIS_STEER_SPEED_KP_DEFAULT,
                                 0.0F,
                                 0.0F,
                                 CHASSIS_STEER_SPEED_MAX_DEFAULT);
    chassis_steering_pid_default(&cfg->yaw_pid,
                                 CHASSIS_YAW_PID_KP_DEFAULT,
                                 0.0F,
                                 CHASSIS_YAW_PID_KD_DEFAULT,
                                 CHASSIS_YAW_PID_MAX_DEFAULT);
    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        cfg->drive_polarity[i] = chassis_steering_pol(cfg->drive_polarity[i]);
        cfg->steer_polarity[i] = chassis_steering_pol(cfg->steer_polarity[i]);
    }
}

/**
 * @brief 清空舵轮输出状态。
 *
 * @param ctx 舵轮上下文。
 */
static void chassis_steering_zero(ChassisSteeringCtx *ctx)
{
    uint32_t i;

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        ctx->drive[i].speed_set = 0.0F;
        ctx->drive[i].current = 0;
        ctx->steer[i].angle_set = ctx->steer[i].angle;
        ctx->steer[i].current = 0;
        ctx->drive_current[i] = 0;
        ctx->steer_current[i] = 0;
    }
    ctx->mode = ChassisModeStop;
}

/**
 * @brief 判断舵轮反馈是否全部在线。
 *
 * @param ctx 舵轮上下文。
 * @return 全部在线返回 1，否则返回 0。
 */
static uint8_t chassis_steering_online(const ChassisSteeringCtx *ctx)
{
    uint32_t i;

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        if ((ctx->drive[i].online == 0U) || (ctx->steer[i].online == 0U)) {
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
static float32_t chassis_steering_power_scale(ChassisSteeringCtx *ctx,
                                              const ChassisCfg *cfg,
                                              const ChassisInput *input,
                                              const float32_t drive_current[CHASSIS_STEERING_WHEEL_NUM])
{
    PowerCtrlInput power;
    uint32_t i;

    if ((ctx == NULL) || (cfg == NULL) || (input == NULL) || (drive_current == NULL)) {
        return 1.0F;
    }

    (void)memset(&power, 0, sizeof(power));
    power.motor_count = CHASSIS_STEERING_WHEEL_NUM;
    power.power_limit = (input->power_limit > 0.0F) ? input->power_limit : cfg->power_limit;
    power.measured_power = input->power;
    power.buffer_energy = input->buffer_energy;
    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        power.motor[i].speed_rpm = (float32_t)input->drive_fb[i].speed_rpm;
        power.motor[i].current = drive_current[i];
        power.motor[i].online = input->drive_fb[i].online;
    }

    return power_ctrl_scale(&ctx->power, &power);
}

/**
 * @brief 计算 yaw 跟随角速度。
 *
 * @param ctx 舵轮上下文。
 * @param input 周期输入。
 * @return yaw 闭环补偿角速度。
 */
static float32_t chassis_steering_yaw(ChassisSteeringCtx *ctx, const ChassisInput *input)
{
    if ((input->cmd.mode == ChassisModeFollowGimbalYaw) || (input->cmd.mode == ChassisModeFollowChassisYaw)) {
        return chassis_steering_pid_calc(&ctx->yaw_pid, input->yaw, input->cmd.yaw_ref);
    }

    return 0.0F;
}

/**
 * @brief 舵轮运动学逆解。
 *
 * @param cfg 底盘配置。
 * @param vx 纵向速度。
 * @param vy 横向速度。
 * @param wz 角速度。
 * @param wheel_angle 四轮目标舵向角。
 * @param wheel_speed 四轮目标速度。
 */
static void chassis_steering_kin(const ChassisCfg *cfg,
                                 float32_t vx,
                                 float32_t vy,
                                 float32_t wz,
                                 float32_t wheel_angle[4],
                                 float32_t wheel_speed[4])
{
    float32_t x[4] = {-cfg->half_length, -cfg->half_length, cfg->half_length, cfg->half_length};
    float32_t y[4] = {cfg->half_width, -cfg->half_width, -cfg->half_width, cfg->half_width};
    uint32_t i;

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        float32_t vx_total = vx - (wz * y[i]);
        float32_t vy_total = vy + (wz * x[i]);
        wheel_speed[i] = sqrtf((vx_total * vx_total) + (vy_total * vy_total));
        wheel_angle[i] = chassis_steering_wrap(atan2f(vy_total, vx_total) - cfg->steer_zero[i]);
    }
}

/**
 * @brief 应用舵轮最短转向策略。
 *
 * @param ctx 舵轮上下文。
 * @param wheel_angle 四轮目标舵向角。
 * @param wheel_speed 四轮目标速度。
 */
static void chassis_steering_short(ChassisSteeringCtx *ctx, float32_t wheel_angle[4], float32_t wheel_speed[4])
{
    uint32_t i;

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        float32_t err = chassis_steering_wrap(wheel_angle[i] - ctx->steer[i].angle);

        if (chassis_steering_abs(err) > (CHASSIS_PI * 0.5F)) {
            wheel_angle[i] = chassis_steering_wrap(wheel_angle[i] + CHASSIS_PI);
            wheel_speed[i] = -wheel_speed[i];
        }
    }
}

/**
 * @brief 限制四轮最大速度。
 *
 * @param cfg 底盘配置。
 * @param wheel_speed 四轮目标速度。
 */
static void chassis_steering_limit_speed(const ChassisCfg *cfg, float32_t wheel_speed[4])
{
    float32_t max_speed = 0.0F;
    uint32_t i;

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        float32_t abs_speed = chassis_steering_abs(wheel_speed[i]);
        if (abs_speed > max_speed) {
            max_speed = abs_speed;
        }
    }

    if (max_speed > cfg->max_drive_speed) {
        float32_t rate = cfg->max_drive_speed / max_speed;
        for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
            wheel_speed[i] *= rate;
        }
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ChassisStatus chassis_steering_init(Chassis *chassis, void *impl)
{
    ChassisSteeringCtx *ctx = (ChassisSteeringCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL)) {
        return ChassisNull;
    }

    chassis_steering_cfg(&chassis->cfg);
    memset(ctx, 0, sizeof(*ctx));
    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        chassis_steering_pid_init(&ctx->drive_pid[i], &chassis->cfg.drive_pid);
        chassis_steering_pid_init(&ctx->angle_pid[i], &chassis->cfg.steer_angle_pid);
        chassis_steering_pid_init(&ctx->steer_pid[i], &chassis->cfg.steer_speed_pid);
    }
    chassis_steering_pid_init(&ctx->yaw_pid, &chassis->cfg.yaw_pid);
    if (power_ctrl_init(&ctx->power, &chassis->cfg.power_cfg) != PowerCtrlOk) {
        return ChassisArg;
    }
    ctx->mode = ChassisModeStop;
    ctx->ready = 1U;

    return ChassisOk;
}

ChassisStatus chassis_steering_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output)
{
    ChassisStatus status;

    if ((chassis == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    status = chassis_steering_update(chassis, impl, input);
    if (status != ChassisOk) {
        return status;
    }

    status = chassis_steering_control(chassis, impl, input);
    if (status != ChassisOk) {
        return status;
    }

    return chassis_steering_output(chassis, impl, output);
}

ChassisStatus chassis_steering_update(Chassis *chassis, void *impl, const ChassisInput *input)
{
    ChassisSteeringCtx *ctx = (ChassisSteeringCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        float32_t drive_speed = input->drive_fb[i].speed;
        float32_t steer_speed = input->steer_fb[i].speed;
        float32_t steer_angle = input->steer_fb[i].angle;

        if (drive_speed == 0.0F) {
            drive_speed = (float32_t)input->drive_fb[i].speed_rpm * chassis->cfg.drive_rpm_to_speed;
        }
        if (steer_speed == 0.0F) {
            steer_speed = (float32_t)input->steer_fb[i].speed_rpm * chassis->cfg.steer_rpm_to_speed;
        }
        if (steer_angle == 0.0F) {
            steer_angle = (float32_t)input->steer_fb[i].ecd * chassis->cfg.steer_ecd_to_rad;
        }

        ctx->drive[i].speed = drive_speed;
        ctx->drive[i].online = input->drive_fb[i].online;
        ctx->steer[i].angle = chassis_steering_wrap(steer_angle);
        ctx->steer[i].speed = steer_speed;
        ctx->steer[i].online = input->steer_fb[i].online;
    }

    return ChassisOk;
}

ChassisStatus chassis_steering_control(Chassis *chassis, void *impl, const ChassisInput *input)
{
    ChassisSteeringCtx *ctx = (ChassisSteeringCtx *)impl;
    ChassisCfg *cfg;
    float32_t wheel_speed[4];
    float32_t wheel_angle[4];
    float32_t drive_raw[CHASSIS_STEERING_WHEEL_NUM];
    float32_t steer_raw[CHASSIS_STEERING_WHEEL_NUM];
    float32_t scale;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    cfg = &chassis->cfg;
    chassis_steering_zero(ctx);

    if ((input->cmd.online == 0U) || (input->cmd.force_stop != 0U) || (input->cmd.mode == ChassisModeStop)) {
        return ChassisOk;
    }

    ctx->mode = input->cmd.mode;

    if (input->cmd.mode == ChassisModeRaw) {
        for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
            float32_t drive_current = (float32_t)input->cmd.raw_drive_current[i] * (float32_t)cfg->drive_polarity[i];
            float32_t steer_current = (float32_t)input->cmd.raw_steer_current[i] * (float32_t)cfg->steer_polarity[i];
            drive_raw[i] = chassis_steering_clamp(drive_current, cfg->max_drive_current);
            steer_raw[i] = chassis_steering_clamp(steer_current, cfg->max_steer_current);
        }
        scale = chassis_steering_power_scale(ctx, cfg, input, drive_raw);
        for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
            ctx->drive_current[i] = chassis_steering_i16(drive_raw[i] * scale);
            ctx->steer_current[i] = chassis_steering_i16(steer_raw[i]);
            ctx->drive[i].current = ctx->drive_current[i];
            ctx->steer[i].current = ctx->steer_current[i];
        }
        return ChassisOk;
    }

    if (chassis_steering_online(ctx) == 0U) {
        chassis_steering_zero(ctx);
        return ChassisState;
    }

    chassis_steering_kin(cfg, input->cmd.vx, input->cmd.vy, input->cmd.wz + chassis_steering_yaw(ctx, input), wheel_angle, wheel_speed);
    chassis_steering_short(ctx, wheel_angle, wheel_speed);
    chassis_steering_limit_speed(cfg, wheel_speed);

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        float32_t angle_err = chassis_steering_wrap(wheel_angle[i] - ctx->steer[i].angle);
        float32_t steer_speed_set = chassis_steering_pid_calc(&ctx->angle_pid[i], 0.0F, angle_err);
        float32_t steer_current = chassis_steering_pid_calc(&ctx->steer_pid[i], ctx->steer[i].speed, steer_speed_set);
        float32_t drive_current = chassis_steering_pid_calc(&ctx->drive_pid[i], ctx->drive[i].speed, wheel_speed[i]);

        ctx->drive[i].speed_set = wheel_speed[i];
        ctx->steer[i].angle_set = wheel_angle[i];
        drive_current = chassis_steering_clamp(drive_current, cfg->max_drive_current);
        steer_current = chassis_steering_clamp(steer_current, cfg->max_steer_current);
        drive_raw[i] = drive_current * (float32_t)cfg->drive_polarity[i];
        steer_raw[i] = steer_current * (float32_t)cfg->steer_polarity[i];
    }

    scale = chassis_steering_power_scale(ctx, cfg, input, drive_raw);
    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        ctx->drive_current[i] = chassis_steering_i16(drive_raw[i] * scale);
        ctx->steer_current[i] = chassis_steering_i16(steer_raw[i]);
        ctx->drive[i].current = ctx->drive_current[i];
        ctx->steer[i].current = ctx->steer_current[i];
    }

    return ChassisOk;
}

ChassisStatus chassis_steering_output(const Chassis *chassis, void *impl, ChassisOutput *output)
{
    const ChassisSteeringCtx *ctx = (const ChassisSteeringCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    (void)chassis;
    memset(output, 0, sizeof(*output));
    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        output->drive_current[i] = ctx->drive_current[i];
        output->steer_current[i] = ctx->steer_current[i];
        output->drive_speed_set[i] = ctx->drive[i].speed_set;
        output->steer_angle_set[i] = ctx->steer[i].angle_set;
    }
    output->mode = ctx->mode;
    output->send_enable = 1U;

    return ChassisOk;
}

ChassisStatus chassis_steering_sync(Chassis *chassis, void *impl)
{
    ChassisSteeringCtx *ctx = (ChassisSteeringCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_NUM; i++) {
        chassis_steering_pid_reset(&ctx->drive_pid[i]);
        chassis_steering_pid_reset(&ctx->angle_pid[i]);
        chassis_steering_pid_reset(&ctx->steer_pid[i]);
        ctx->drive[i].speed_set = ctx->drive[i].speed;
        ctx->steer[i].angle_set = ctx->steer[i].angle;
    }
    chassis_steering_pid_reset(&ctx->yaw_pid);

    return ChassisOk;
}
