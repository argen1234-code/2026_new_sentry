/**
 ******************************************************************************
 * @file    shoot_mono.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   单组摩擦轮发射构型实现。
 *
 * @note    本文件只使用 ShootInput 注入的命令、反馈、热量和时间戳。
 * @note    本文件不读取遥控器、CAN、裁判系统、HAL tick 或云台全局状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "shoot_mono.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */

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
static float32_t shoot_mono_abs(float32_t value)
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
static float32_t shoot_mono_pick(float32_t value, float32_t default_value)
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
static float32_t shoot_mono_clamp(float32_t value, float32_t limit)
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
static int16_t shoot_mono_i16(float32_t value)
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
 * @brief 读取电流极性配置。
 *
 * @param polarity 配置极性。
 * @return 有效极性，未配置时为 1。
 */
static int8_t shoot_mono_pol(int8_t polarity)
{
    return (polarity == 0) ? 1 : polarity;
}

/**
 * @brief 初始化 PID 状态。
 *
 * @param pid PID 状态。
 * @param cfg PID 配置。
 */
static void shoot_mono_pid_init(ShootMonoPid *pid, const ShootPidCfg *cfg)
{
    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
}

/**
 * @brief 清空 PID 动态状态。
 *
 * @param pid PID 状态。
 */
static void shoot_mono_pid_reset(ShootMonoPid *pid)
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
static float32_t shoot_mono_pid_calc(ShootMonoPid *pid, float32_t get, float32_t set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;

    pid->iout += pid->cfg.ki * pid->err;
    pid->iout = shoot_mono_clamp(pid->iout, pid->cfg.max_iout);

    pid->out = (pid->cfg.kp * pid->err) + pid->iout + (pid->cfg.kd * (pid->err - pid->last_err));
    pid->out = shoot_mono_clamp(pid->out, pid->cfg.max_out);
    pid->last_err = pid->err;

    return pid->out;
}

/**
 * @brief 归一化单组发射配置。
 *
 * @param cfg 发射配置。
 */
static void shoot_mono_cfg(ShootCfg *cfg)
{
    uint32_t i;

    if (cfg->trigger_ecd_range <= 0) {
        cfg->trigger_ecd_range = SHOOT_ECD_RANGE_DEFAULT;
    }

    if (cfg->trigger_half_ecd_range <= 0) {
        cfg->trigger_half_ecd_range = cfg->trigger_ecd_range / 2;
    }

    cfg->trigger_ecd_to_rad = shoot_mono_pick(cfg->trigger_ecd_to_rad, SHOOT_ECD_TO_RAD_DEFAULT);
    cfg->trigger_rpm_to_rad = shoot_mono_pick(cfg->trigger_rpm_to_rad, SHOOT_RPM_TO_RAD_DEFAULT);
    cfg->fric_rpm_to_speed = shoot_mono_pick(cfg->fric_rpm_to_speed, SHOOT_FRIC_RPM_TO_SPEED_DEFAULT);
    cfg->trigger_step = shoot_mono_pick(cfg->trigger_step, SHOOT_TRIGGER_STEP_DEFAULT);
    cfg->trigger_done_err = shoot_mono_pick(cfg->trigger_done_err, SHOOT_TRIGGER_DONE_ERR_DEFAULT);
    cfg->trigger_speed = shoot_mono_pick(cfg->trigger_speed, SHOOT_TRIGGER_SPEED_DEFAULT);
    cfg->block_speed = shoot_mono_pick(cfg->block_speed, SHOOT_BLOCK_SPEED_DEFAULT);
    cfg->block_time = (cfg->block_time != 0U) ? cfg->block_time : SHOOT_BLOCK_TIME_DEFAULT;
    cfg->reverse_time = (cfg->reverse_time != 0U) ? cfg->reverse_time : SHOOT_REVERSE_TIME_DEFAULT;
    cfg->heat_per_bullet = shoot_mono_pick(cfg->heat_per_bullet, SHOOT_HEAT_PER_BULLET_DEFAULT);
    cfg->heat_cool = shoot_mono_pick(cfg->heat_cool, SHOOT_HEAT_COOL_DEFAULT);
    cfg->fric_ready_ratio = shoot_mono_pick(cfg->fric_ready_ratio, SHOOT_FRIC_READY_RATIO_DEFAULT);
    cfg->trigger_polarity = shoot_mono_pol(cfg->trigger_polarity);

    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        cfg->fric_speed[i] = shoot_mono_pick(cfg->fric_speed[i], SHOOT_FRIC_SPEED_DEFAULT);
        cfg->fric_polarity[i] = shoot_mono_pol(cfg->fric_polarity[i]);
    }
}

/**
 * @brief 清空单组发射输出状态。
 *
 * @param ctx 单组发射上下文。
 */
static void shoot_mono_zero(ShootMonoCtx *ctx)
{
    uint32_t i;

    ctx->trigger_speed_set = 0.0F;
    ctx->trigger_current = 0;
    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        ctx->fric[i].speed_set = 0.0F;
        ctx->fric[i].current = 0;
        ctx->fric_current[i] = 0;
    }
}

/**
 * @brief 进入停止状态并清空动态发弹目标。
 *
 * @param ctx 单组发射上下文。
 */
static void shoot_mono_stop(ShootMonoCtx *ctx)
{
    uint32_t i;

    shoot_mono_zero(ctx);
    shoot_mono_pid_reset(&ctx->trigger_pid);
    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        shoot_mono_pid_reset(&ctx->fric_pid[i]);
    }

    ctx->mode = ShootModeStop;
    ctx->move_flag = 0U;
    ctx->block_time = 0U;
    ctx->reverse_time = 0U;
    ctx->trigger_target = ctx->trigger_angle;
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 根据输入时间戳更新本地热量估计。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 * @param input 周期输入。
 */
static void shoot_mono_update_heat(ShootMonoCtx *ctx, const ShootCfg *cfg, const ShootInput *input)
{
    if ((ctx->last_tick != 0U) && (input->tick_ms >= ctx->last_tick)) {
        float32_t dt = (float32_t)(input->tick_ms - ctx->last_tick) * 0.001F;
        ctx->heat -= cfg->heat_cool * dt;
        if (ctx->heat < 0.0F) {
            ctx->heat = 0.0F;
        }
    }

    ctx->last_tick = input->tick_ms;
    if (input->heat > ctx->heat) {
        ctx->heat = input->heat;
    }
    ctx->heat_limit = input->heat_limit;
}

/**
 * @brief 更新拨弹电机连续角度和速度。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 * @param input 周期输入。
 */
static void shoot_mono_update_angle(ShootMonoCtx *ctx, const ShootCfg *cfg, const ShootInput *input)
{
    ctx->trigger_online = input->trigger_fb.online;
    if (ctx->trigger_online == 0U) {
        ctx->trigger_speed = 0.0F;
        return;
    }

    if (ctx->ecd_ready == 0U) {
        ctx->last_ecd = input->trigger_fb.ecd;
        ctx->trigger_count = 0;
        ctx->ecd_ready = 1U;
    } else {
        int32_t ecd_delta = (int32_t)input->trigger_fb.ecd - (int32_t)ctx->last_ecd;

        if (ecd_delta > cfg->trigger_half_ecd_range) {
            ctx->trigger_count--;
        } else if (ecd_delta < -cfg->trigger_half_ecd_range) {
            ctx->trigger_count++;
        }

        ctx->last_ecd = input->trigger_fb.ecd;
    }

    ctx->trigger_angle = ((float32_t)(ctx->trigger_count * cfg->trigger_ecd_range) +
                          (float32_t)input->trigger_fb.ecd) * cfg->trigger_ecd_to_rad;
    ctx->trigger_speed = (float32_t)input->trigger_fb.speed_rpm * cfg->trigger_rpm_to_rad;
}

/**
 * @brief 判断当前热量是否允许继续发弹。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 * @return 允许发弹返回 1，否则返回 0。
 */
static uint8_t shoot_mono_heat_ok(const ShootMonoCtx *ctx, const ShootCfg *cfg)
{
    if (ctx->heat_limit <= 0.0F) {
        return 1U;
    }

    return ((ctx->heat + cfg->heat_per_bullet) <= ctx->heat_limit) ? 1U : 0U;
}

/**
 * @brief 判断单组摩擦轮是否已经达到目标速度。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 * @return 达速返回 1，否则返回 0。
 */
static uint8_t shoot_mono_fric_ready(const ShootMonoCtx *ctx, const ShootCfg *cfg)
{
    uint32_t i;

    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        float32_t target = cfg->fric_speed[i];
        float32_t ready_speed = shoot_mono_abs(target) * cfg->fric_ready_ratio;

        if (ready_speed <= 0.0F) {
            continue;
        }

        if (ctx->fric[i].online == 0U) {
            return 0U;
        }

        if ((target > 0.0F) && (ctx->fric[i].speed < ready_speed)) {
            return 0U;
        }

        if ((target < 0.0F) && (ctx->fric[i].speed > -ready_speed)) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 计算单组摩擦轮输出电流。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 */
static void shoot_mono_fric_control(ShootMonoCtx *ctx, const ShootCfg *cfg)
{
    uint32_t i;

    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        float32_t current;

        ctx->fric[i].speed_set = cfg->fric_speed[i];
        current = shoot_mono_pid_calc(&ctx->fric_pid[i], ctx->fric[i].speed, ctx->fric[i].speed_set);
        current *= (float32_t)cfg->fric_polarity[i];
        ctx->fric[i].current = shoot_mono_i16(current);
        ctx->fric_current[i] = ctx->fric[i].current;
    }
}

/**
 * @brief 开始一发拨弹动作。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 */
static void shoot_mono_start(ShootMonoCtx *ctx, const ShootCfg *cfg)
{
    ctx->trigger_target = ctx->trigger_angle + cfg->trigger_step;
    ctx->move_flag = 1U;
    ctx->block_time = 0U;
    ctx->reverse_time = 0U;
}

/**
 * @brief 根据堵转状态计算拨弹目标速度。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 * @param forward_speed 正向拨弹速度。
 * @return 实际拨弹目标速度。
 */
static float32_t shoot_mono_turn(ShootMonoCtx *ctx, const ShootCfg *cfg, float32_t forward_speed)
{
    float32_t speed_set = forward_speed;

    if ((shoot_mono_abs(ctx->trigger_speed) < cfg->block_speed) && (ctx->block_time < cfg->block_time)) {
        ctx->block_time++;
        ctx->reverse_time = 0U;
    } else if ((ctx->block_time >= cfg->block_time) && (ctx->reverse_time < cfg->reverse_time)) {
        speed_set = -forward_speed;
        ctx->reverse_time++;
    } else {
        ctx->block_time = 0U;
        ctx->reverse_time = 0U;
    }

    return speed_set;
}

/**
 * @brief 执行拨弹闭环控制。
 *
 * @param ctx 单组发射上下文。
 * @param cfg 发射配置。
 * @return 完成一发返回 1，否则返回 0。
 */
static uint8_t shoot_mono_trigger(ShootMonoCtx *ctx, const ShootCfg *cfg)
{
    float32_t delta = ctx->trigger_target - ctx->trigger_angle;

    if (shoot_mono_abs(delta) <= cfg->trigger_done_err) {
        ctx->move_flag = 0U;
        ctx->trigger_speed_set = 0.0F;
        ctx->trigger_current = 0;
        ctx->heat += cfg->heat_per_bullet;
        return 1U;
    }

    ctx->trigger_speed_set = (delta >= 0.0F) ? cfg->trigger_speed : -cfg->trigger_speed;
    ctx->trigger_speed_set = shoot_mono_turn(ctx, cfg, ctx->trigger_speed_set);
    ctx->trigger_current = shoot_mono_i16(shoot_mono_pid_calc(&ctx->trigger_pid,
                                                              ctx->trigger_speed,
                                                              ctx->trigger_speed_set) *
                                          (float32_t)cfg->trigger_polarity);

    return 0U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ShootStatus shoot_mono_init(Shoot *shoot, void *impl)
{
    ShootMonoCtx *ctx = (ShootMonoCtx *)impl;
    uint32_t i;

    if ((shoot == NULL) || (ctx == NULL)) {
        return ShootNull;
    }

    shoot_mono_cfg(&shoot->cfg);

    memset(ctx, 0, sizeof(*ctx));
    ctx->mode = ShootModeStop;
    shoot_mono_pid_init(&ctx->trigger_pid, &shoot->cfg.trigger_pid);
    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        shoot_mono_pid_init(&ctx->fric_pid[i], &shoot->cfg.fric_pid);
    }
    ctx->ready = 1U;

    return ShootOk;
}

ShootStatus shoot_mono_step(Shoot *shoot, void *impl, const ShootInput *input, ShootOutput *output)
{
    ShootStatus status;

    if ((shoot == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return ShootNull;
    }

    status = shoot_mono_update(shoot, impl, input);
    if (status != ShootOk) {
        return status;
    }

    status = shoot_mono_control(shoot, impl, input);
    if (status != ShootOk) {
        return status;
    }

    return shoot_mono_output(shoot, impl, output);
}

ShootStatus shoot_mono_update(Shoot *shoot, void *impl, const ShootInput *input)
{
    ShootMonoCtx *ctx = (ShootMonoCtx *)impl;
    const ShootCfg *cfg;
    uint32_t i;

    if ((shoot == NULL) || (ctx == NULL) || (input == NULL)) {
        return ShootNull;
    }

    if (ctx->ready == 0U) {
        return ShootState;
    }

    cfg = &shoot->cfg;
    shoot_mono_update_angle(ctx, cfg, input);
    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        ctx->fric[i].speed = (float32_t)input->fric_fb[i].speed_rpm * cfg->fric_rpm_to_speed;
        ctx->fric[i].online = input->fric_fb[i].online;
    }
    shoot_mono_update_heat(ctx, cfg, input);

    return ShootOk;
}

ShootStatus shoot_mono_control(Shoot *shoot, void *impl, const ShootInput *input)
{
    ShootMonoCtx *ctx = (ShootMonoCtx *)impl;
    const ShootCfg *cfg;
    uint8_t active;
    uint8_t once_edge;
    uint8_t fire_req;
    uint8_t bullet_done;

    if ((shoot == NULL) || (ctx == NULL) || (input == NULL)) {
        return ShootNull;
    }

    if (ctx->ready == 0U) {
        return ShootState;
    }

    cfg = &shoot->cfg;
    active = ((input->cmd.online != 0U) && (input->cmd.force_stop == 0U) && (input->cmd.fric_enable != 0U)) ? 1U : 0U;
    once_edge = ((input->cmd.fire_once != 0U) && (ctx->last_fire_once == 0U)) ? 1U : 0U;
    fire_req = ((input->cmd.fire_hold != 0U) || (once_edge != 0U) || (ctx->move_flag != 0U)) ? 1U : 0U;

    shoot_mono_zero(ctx);

    if (active == 0U) {
        shoot_mono_stop(ctx);
        ctx->last_fire_once = input->cmd.fire_once;
        return ShootOk;
    }

    shoot_mono_fric_control(ctx, cfg);
    if (shoot_mono_fric_ready(ctx, cfg) == 0U) {
        ctx->mode = ShootModeReadyFric;
        ctx->move_flag = 0U;
        ctx->trigger_target = ctx->trigger_angle;
        ctx->last_fire_once = input->cmd.fire_once;
        return ShootOk;
    }

    if (ctx->trigger_online == 0U) {
        ctx->mode = ShootModeReadyBullet;
        ctx->move_flag = 0U;
        ctx->trigger_target = ctx->trigger_angle;
        ctx->last_fire_once = input->cmd.fire_once;
        return ShootOk;
    }

    if ((fire_req != 0U) && (shoot_mono_heat_ok(ctx, cfg) == 0U)) {
        ctx->mode = ShootModeDone;
        ctx->move_flag = 0U;
        ctx->trigger_target = ctx->trigger_angle;
        ctx->last_fire_once = input->cmd.fire_once;
        return ShootOk;
    }

    if (ctx->move_flag == 0U) {
        if (input->cmd.fire_hold != 0U) {
            shoot_mono_start(ctx, cfg);
            ctx->mode = ShootModeContinue;
        } else if (once_edge != 0U) {
            shoot_mono_start(ctx, cfg);
            ctx->mode = ShootModeBullet;
        } else {
            ctx->mode = ShootModeReadyBullet;
            ctx->last_fire_once = input->cmd.fire_once;
            return ShootOk;
        }
    } else {
        ctx->mode = (input->cmd.fire_hold != 0U) ? ShootModeContinue : ShootModeBullet;
    }

    bullet_done = shoot_mono_trigger(ctx, cfg);
    if (bullet_done != 0U) {
        ctx->mode = (input->cmd.fire_hold != 0U) ? ShootModeContinue : ShootModeDone;
    }

    ctx->last_fire_once = input->cmd.fire_once;
    return ShootOk;
}

ShootStatus shoot_mono_output(const Shoot *shoot, void *impl, ShootOutput *output)
{
    const ShootMonoCtx *ctx = (const ShootMonoCtx *)impl;
    uint32_t i;

    if ((shoot == NULL) || (ctx == NULL) || (output == NULL)) {
        return ShootNull;
    }

    if (ctx->ready == 0U) {
        return ShootState;
    }

    (void)shoot;
    memset(output, 0, sizeof(*output));
    output->trigger_current = ctx->trigger_current;
    for (i = 0U; i < SHOOT_MONO_FRIC_NUM; i++) {
        output->fric_current[i] = ctx->fric_current[i];
    }
    output->mode = ctx->mode;
    output->send_enable = 1U;

    return ShootOk;
}

ShootStatus shoot_mono_sync(Shoot *shoot, void *impl)
{
    ShootMonoCtx *ctx = (ShootMonoCtx *)impl;

    if ((shoot == NULL) || (ctx == NULL)) {
        return ShootNull;
    }

    if (ctx->ready == 0U) {
        return ShootState;
    }

    ctx->trigger_target = ctx->trigger_angle;
    ctx->move_flag = 0U;
    ctx->block_time = 0U;
    ctx->reverse_time = 0U;
    shoot_mono_pid_reset(&ctx->trigger_pid);

    return ShootOk;
}
