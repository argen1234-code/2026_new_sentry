/**
 ******************************************************************************
 * @file    chassis_balance.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   平衡底盘构型实现。
 *
 * @note    本文件只使用 ChassisInput 注入的数据，不读取任务、INS、CAN 或裁判系统。
 * @note    VMC/LQR/PID 相关计算作为构型内部静态实现保留，不依赖旧外部模块。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "chassis_balance.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 左腿索引。 */
#define CHASSIS_BALANCE_LEFT 0U

/** @brief 右腿索引。 */
#define CHASSIS_BALANCE_RIGHT 1U

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
static float32_t chassis_balance_abs(float32_t value)
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
static float32_t chassis_balance_pick(float32_t value, float32_t default_value)
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
static float32_t chassis_balance_clamp(float32_t value, float32_t limit)
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
 * @brief 读取极性配置。
 *
 * @param polarity 配置极性。
 * @return 有效极性。
 */
static int8_t chassis_balance_pol(int8_t polarity)
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
static void chassis_balance_pid_default(ChassisPidCfg *cfg,
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
    cfg->max_out = chassis_balance_pick(cfg->max_out, max_out);
}

/**
 * @brief 初始化 PID 状态。
 *
 * @param pid PID 状态。
 * @param cfg PID 配置。
 */
static void chassis_balance_pid_init(ChassisBalancePid *pid, const ChassisPidCfg *cfg)
{
    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
}

/**
 * @brief 清空 PID 动态状态。
 *
 * @param pid PID 状态。
 */
static void chassis_balance_pid_reset(ChassisBalancePid *pid)
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
static float32_t chassis_balance_pid_calc(ChassisBalancePid *pid, float32_t get, float32_t set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    pid->iout += pid->cfg.ki * pid->err;
    pid->iout = chassis_balance_clamp(pid->iout, pid->cfg.max_iout);
    pid->out = (pid->cfg.kp * pid->err) + pid->iout + (pid->cfg.kd * (pid->err - pid->last_err));
    pid->out = chassis_balance_clamp(pid->out, pid->cfg.max_out);
    pid->last_err = pid->err;

    return pid->out;
}

/**
 * @brief 归一化平衡底盘配置。
 *
 * @param cfg 底盘配置。
 */
static void chassis_balance_cfg(ChassisCfg *cfg)
{
    uint32_t i;

    cfg->max_joint_torque = chassis_balance_pick(cfg->max_joint_torque, CHASSIS_JOINT_TORQUE_MAX_DEFAULT);
    cfg->max_wheel_torque = chassis_balance_pick(cfg->max_wheel_torque, CHASSIS_WHEEL_TORQUE_MAX_DEFAULT);
    cfg->power_limit = chassis_balance_pick(cfg->power_limit, CHASSIS_POWER_LIMIT_DEFAULT);
    cfg->leg_default = chassis_balance_pick(cfg->leg_default, CHASSIS_LEG_DEFAULT);
    cfg->leg_min = chassis_balance_pick(cfg->leg_min, CHASSIS_LEG_MIN_DEFAULT);
    cfg->leg_max = chassis_balance_pick(cfg->leg_max, CHASSIS_LEG_MAX_DEFAULT);
    chassis_balance_pid_default(&cfg->leg_pid,
                                CHASSIS_LEG_PID_KP_DEFAULT,
                                0.0F,
                                CHASSIS_LEG_PID_KD_DEFAULT,
                                CHASSIS_LEG_PID_MAX_DEFAULT);
    chassis_balance_pid_default(&cfg->roll_pid,
                                CHASSIS_ROLL_PID_KP_DEFAULT,
                                0.0F,
                                0.0F,
                                CHASSIS_ROLL_PID_MAX_DEFAULT);
    chassis_balance_pid_default(&cfg->turn_pid,
                                CHASSIS_TURN_PID_KP_DEFAULT,
                                0.0F,
                                CHASSIS_TURN_PID_KD_DEFAULT,
                                CHASSIS_TURN_PID_MAX_DEFAULT);
    for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
        cfg->joint_polarity[i] = chassis_balance_pol(cfg->joint_polarity[i]);
    }
    for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
        cfg->wheel_polarity[i] = chassis_balance_pol(cfg->wheel_polarity[i]);
    }

    if (cfg->balance_lqr[0] == 0.0F) {
        cfg->balance_lqr[0] = -14.0F;
        cfg->balance_lqr[1] = -1.0F;
        cfg->balance_lqr[2] = 0.0F;
        cfg->balance_lqr[3] = -1.0F;
        cfg->balance_lqr[4] = 5.0F;
        cfg->balance_lqr[5] = 0.5F;
    }
}

/**
 * @brief 清空平衡底盘输出状态。
 *
 * @param ctx 平衡底盘上下文。
 */
static void chassis_balance_zero(ChassisBalanceCtx *ctx)
{
    uint32_t i;

    for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
        ctx->joint_torque[i] = 0.0F;
        ctx->joint_pos_set[i] = ctx->joint[i].position;
    }
    for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
        ctx->wheel_torque[i] = 0.0F;
    }
    ctx->mode = ChassisModeStop;
}

/**
 * @brief 判断平衡底盘反馈是否在线。
 *
 * @param ctx 平衡底盘上下文。
 * @return 全部在线返回 1，否则返回 0。
 */
static uint8_t chassis_balance_online(const ChassisBalanceCtx *ctx)
{
    uint32_t i;

    for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
        if (ctx->joint[i].online == 0U) {
            return 0U;
        }
    }
    for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
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
static float32_t chassis_balance_power_scale(const ChassisCfg *cfg, const ChassisInput *input)
{
    float32_t limit = (input->power_limit > 0.0F) ? input->power_limit : cfg->power_limit;

    if ((limit <= 0.0F) || (input->power <= limit) || (input->power <= 0.0F)) {
        return 1.0F;
    }

    return chassis_balance_clamp(limit / input->power, 1.0F);
}

/**
 * @brief 计算目标腿长。
 *
 * @param cfg 底盘配置。
 * @param input 周期输入。
 * @param side 腿索引。
 * @return 目标腿长。
 */
static float32_t chassis_balance_leg_set(const ChassisCfg *cfg, const ChassisInput *input, uint32_t side)
{
    float32_t set = input->cmd.leg_length;

    if (side == CHASSIS_BALANCE_LEFT) {
        set = (input->cmd.leg_left != 0.0F) ? input->cmd.leg_left : set;
    } else {
        set = (input->cmd.leg_right != 0.0F) ? input->cmd.leg_right : set;
    }

    if (set == 0.0F) {
        set = cfg->leg_default;
    }

    if (set < cfg->leg_min) {
        set = cfg->leg_min;
    }
    if (set > cfg->leg_max) {
        set = cfg->leg_max;
    }

    return set;
}

/**
 * @brief 计算单侧 LQR 轮毂力矩。
 *
 * @param cfg 底盘配置。
 * @param input 周期输入。
 * @param side 腿索引。
 * @return 轮毂力矩。
 */
static float32_t chassis_balance_lqr(const ChassisCfg *cfg, const ChassisInput *input, uint32_t side)
{
    const float32_t *k = cfg->balance_lqr;
    float32_t theta = input->leg_theta[side];
    float32_t d_theta = input->leg_theta_speed[side];
    float32_t wheel_speed = input->wheel_fb[side].velocity;
    float32_t pitch = input->pitch;
    float32_t pitch_speed = input->gyro[1];

    return (k[0] * theta) +
           (k[1] * d_theta) +
           (k[2] * 0.0F) +
           (k[3] * (wheel_speed - input->cmd.vx)) +
           (k[4] * pitch) +
           (k[5] * pitch_speed);
}

/**
 * @brief 计算平衡底盘力矩输出。
 *
 * @param ctx 平衡底盘上下文。
 * @param cfg 底盘配置。
 * @param input 周期输入。
 */
static void chassis_balance_calc(ChassisBalanceCtx *ctx, const ChassisCfg *cfg, const ChassisInput *input)
{
    float32_t left_set = chassis_balance_leg_set(cfg, input, CHASSIS_BALANCE_LEFT);
    float32_t right_set = chassis_balance_leg_set(cfg, input, CHASSIS_BALANCE_RIGHT);
    float32_t left_force = chassis_balance_pid_calc(&ctx->leg_pid[CHASSIS_BALANCE_LEFT],
                                                    ctx->leg[CHASSIS_BALANCE_LEFT].length,
                                                    left_set);
    float32_t right_force = chassis_balance_pid_calc(&ctx->leg_pid[CHASSIS_BALANCE_RIGHT],
                                                     ctx->leg[CHASSIS_BALANCE_RIGHT].length,
                                                     right_set);
    float32_t roll_support = chassis_balance_pid_calc(&ctx->roll_pid, input->roll, 0.0F);
    float32_t turn_torque = chassis_balance_pid_calc(&ctx->turn_pid, input->yaw, input->cmd.yaw_ref);
    float32_t left_wheel = chassis_balance_lqr(cfg, input, CHASSIS_BALANCE_LEFT) - turn_torque;
    float32_t right_wheel = chassis_balance_lqr(cfg, input, CHASSIS_BALANCE_RIGHT) + turn_torque;

    if (input->cmd.jump != 0U) {
        left_force += cfg->max_joint_torque * 0.5F;
        right_force += cfg->max_joint_torque * 0.5F;
    }

    ctx->joint_torque[0] = chassis_balance_clamp(left_force + roll_support, cfg->max_joint_torque);
    ctx->joint_torque[1] = chassis_balance_clamp(left_force - roll_support, cfg->max_joint_torque);
    ctx->joint_torque[2] = chassis_balance_clamp(right_force - roll_support, cfg->max_joint_torque);
    ctx->joint_torque[3] = chassis_balance_clamp(right_force + roll_support, cfg->max_joint_torque);
    ctx->wheel_torque[CHASSIS_BALANCE_LEFT] = chassis_balance_clamp(left_wheel, cfg->max_wheel_torque);
    ctx->wheel_torque[CHASSIS_BALANCE_RIGHT] = chassis_balance_clamp(right_wheel, cfg->max_wheel_torque);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ChassisStatus chassis_balance_init(Chassis *chassis, void *impl)
{
    ChassisBalanceCtx *ctx = (ChassisBalanceCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL)) {
        return ChassisNull;
    }

    chassis_balance_cfg(&chassis->cfg);
    memset(ctx, 0, sizeof(*ctx));
    for (i = 0U; i < CHASSIS_BALANCE_LEG_NUM; i++) {
        chassis_balance_pid_init(&ctx->leg_pid[i], &chassis->cfg.leg_pid);
    }
    chassis_balance_pid_init(&ctx->roll_pid, &chassis->cfg.roll_pid);
    chassis_balance_pid_init(&ctx->turn_pid, &chassis->cfg.turn_pid);
    ctx->mode = ChassisModeStop;
    ctx->ready = 1U;

    return ChassisOk;
}

ChassisStatus chassis_balance_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output)
{
    ChassisStatus status;

    if ((chassis == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    status = chassis_balance_update(chassis, impl, input);
    if (status != ChassisOk) {
        return status;
    }

    status = chassis_balance_control(chassis, impl, input);
    if (status != ChassisOk) {
        return status;
    }

    return chassis_balance_output(chassis, impl, output);
}

ChassisStatus chassis_balance_update(Chassis *chassis, void *impl, const ChassisInput *input)
{
    ChassisBalanceCtx *ctx = (ChassisBalanceCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
        ctx->joint[i] = input->joint_fb[i];
    }
    for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
        ctx->wheel[i] = input->wheel_fb[i];
        ctx->leg[i].length = input->leg_length[i];
        ctx->leg[i].length_speed = input->leg_speed[i];
        ctx->leg[i].theta = input->leg_theta[i];
        ctx->leg[i].theta_speed = input->leg_theta_speed[i];
        ctx->leg[i].online = input->wheel_fb[i].online;
    }

    return ChassisOk;
}

ChassisStatus chassis_balance_control(Chassis *chassis, void *impl, const ChassisInput *input)
{
    ChassisBalanceCtx *ctx = (ChassisBalanceCtx *)impl;
    ChassisCfg *cfg;
    float32_t scale;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (input == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    cfg = &chassis->cfg;
    chassis_balance_zero(ctx);

    if ((input->cmd.online == 0U) || (input->cmd.force_stop != 0U) || (input->cmd.mode == ChassisModeStop)) {
        return ChassisOk;
    }

    ctx->mode = input->cmd.mode;
    scale = chassis_balance_power_scale(cfg, input);

    if (input->cmd.mode == ChassisModeRaw) {
        for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
            ctx->joint_torque[i] = chassis_balance_clamp(input->cmd.raw_joint_torque[i], cfg->max_joint_torque) *
                                   (float32_t)cfg->joint_polarity[i] * scale;
        }
        for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
            ctx->wheel_torque[i] = chassis_balance_clamp(input->cmd.raw_wheel_torque[i], cfg->max_wheel_torque) *
                                   (float32_t)cfg->wheel_polarity[i] * scale;
        }
        return ChassisOk;
    }

    if ((input->cmd.mode != ChassisModeBalance) || (chassis_balance_online(ctx) == 0U)) {
        chassis_balance_zero(ctx);
        return ChassisState;
    }

    chassis_balance_calc(ctx, cfg, input);
    for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
        ctx->joint_torque[i] *= (float32_t)cfg->joint_polarity[i] * scale;
        ctx->joint_torque[i] = chassis_balance_clamp(ctx->joint_torque[i], cfg->max_joint_torque);
    }
    for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
        ctx->wheel_torque[i] *= (float32_t)cfg->wheel_polarity[i] * scale;
        ctx->wheel_torque[i] = chassis_balance_clamp(ctx->wheel_torque[i], cfg->max_wheel_torque);
    }

    return ChassisOk;
}

ChassisStatus chassis_balance_output(const Chassis *chassis, void *impl, ChassisOutput *output)
{
    const ChassisBalanceCtx *ctx = (const ChassisBalanceCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL) || (output == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    (void)chassis;
    memset(output, 0, sizeof(*output));
    for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
        output->joint_torque[i] = ctx->joint_torque[i];
        output->joint_pos_set[i] = ctx->joint_pos_set[i];
    }
    for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
        output->wheel_torque[i] = ctx->wheel_torque[i];
    }
    output->mode = ctx->mode;
    output->send_enable = 1U;

    return ChassisOk;
}

ChassisStatus chassis_balance_sync(Chassis *chassis, void *impl)
{
    ChassisBalanceCtx *ctx = (ChassisBalanceCtx *)impl;
    uint32_t i;

    if ((chassis == NULL) || (ctx == NULL)) {
        return ChassisNull;
    }

    if (ctx->ready == 0U) {
        return ChassisState;
    }

    for (i = 0U; i < CHASSIS_BALANCE_LEG_NUM; i++) {
        chassis_balance_pid_reset(&ctx->leg_pid[i]);
    }
    chassis_balance_pid_reset(&ctx->roll_pid);
    chassis_balance_pid_reset(&ctx->turn_pid);

    return ChassisOk;
}
