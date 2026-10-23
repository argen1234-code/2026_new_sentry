/**
 ******************************************************************************
 * @file    gimbal_direct_yaw_pitch.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   yaw-pitch 直驱云台构型实现。
 *
 * @note    本文件只处理直驱 yaw/pitch 反馈映射、目标更新和电流计算。
 * @note    本文件不读取任务、遥控、INS、CAN、发射机构或错误检测全局状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "gimbal_direct_yaw_pitch.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 圆周常量 2*pi。 */
#define GIMBAL_DIRECT_2PI (6.28318530717958647692F)

/** @brief 圆周常量 pi。 */
#define GIMBAL_DIRECT_PI (3.14159265358979323846F)

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
static uint8_t direct_finite(float32_t value)
{
    return isfinite((double)value) ? 1U : 0U;
}

/**
 * @brief 将角度规整到 [-pi, pi]。
 *
 * @param angle 输入角度，单位 rad。
 * @return 规整后的角度，单位 rad。
 */
static float32_t direct_wrap(float32_t angle)
{
    if (direct_finite(angle) == 0U) {
        return 0.0F;
    }

    while (angle > GIMBAL_DIRECT_PI) {
        angle -= GIMBAL_DIRECT_2PI;
    }

    while (angle < -GIMBAL_DIRECT_PI) {
        angle += GIMBAL_DIRECT_2PI;
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
static float32_t direct_limit(float32_t value, float32_t limit)
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
 * @brief 将浮点电流转换为 int16 电流。
 *
 * @param value 浮点电流。
 * @return int16 电流。
 */
static int16_t direct_i16(float32_t value)
{
    value = direct_limit(value, 32767.0F);
    return (int16_t)value;
}

/**
 * @brief 将控制周期转换为秒。
 *
 * @param gimbal 公共云台上下文。
 * @return 控制周期，单位 s，非法时返回 0。
 */
static float32_t direct_period_s(const Gimbal *gimbal)
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
static float32_t direct_ref_rate(float32_t delta, float32_t period_s)
{
    if ((direct_finite(delta) == 0U) ||
        (direct_finite(period_s) == 0U) ||
        (period_s <= 0.0F)) {
        return 0.0F;
    }

    return delta / period_s;
}

/**
 * @brief 限制 yaw 位置环纠偏速度。
 *
 * @param correction 原始纠偏速度。
 * @return 限制后的纠偏速度。
 */
static float32_t direct_yaw_correction(float32_t correction)
{
    if (direct_finite(correction) == 0U) {
        return 0.0F;
    }

    return direct_limit(correction, GIMBAL_YAW_CORR_RATE_MAX);
}

/**
 * @brief 初始化直驱 PID 状态。
 *
 * @param pid PID 状态。
 * @param cfg PID 参数。
 */
static void direct_pid_init(GimbalDirectPid *pid, const GimbalPidCfg *cfg)
{
    if ((pid == NULL) || (cfg == NULL)) {
        return;
    }

    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
}

/**
 * @brief 清零直驱 PID 动态项。
 *
 * @param pid PID 状态。
 */
static void direct_pid_clear(GimbalDirectPid *pid)
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
 * @brief 计算角度 PID。
 *
 * @param pid PID 状态。
 * @param get 反馈角，单位 rad。
 * @param set 设定角，单位 rad。
 * @param d_in 外部微分输入。
 * @return PID 输出。
 */
static float32_t direct_pid_angle(GimbalDirectPid *pid, float32_t get, float32_t set, float32_t d_in)
{
    float32_t pout;
    float32_t dout;

    if ((pid == NULL) ||
        (direct_finite(get) == 0U) ||
        (direct_finite(set) == 0U) ||
        (direct_finite(d_in) == 0U)) {
        return 0.0F;
    }

    pid->get = get;
    pid->set = set;
    pid->err = direct_wrap(set - get);
    pout = pid->cfg.kp * pid->err;
    pid->iout += pid->cfg.ki * pid->err;
    pid->iout = direct_limit(pid->iout, pid->cfg.max_iout);
    dout = pid->cfg.kd * d_in;
    pid->out = direct_limit(pout + pid->iout + dout, pid->cfg.max_out);
    pid->last_err = pid->err;
    return pid->out;
}

/**
 * @brief 计算速度 PID。
 *
 * @param pid PID 状态。
 * @param get 反馈速度，单位 rad/s。
 * @param set 设定速度，单位 rad/s。
 * @return PID 输出。
 */
static float32_t direct_pid_speed(GimbalDirectPid *pid, float32_t get, float32_t set)
{
    float32_t err;
    float32_t out;

    if ((pid == NULL) ||
        (direct_finite(get) == 0U) ||
        (direct_finite(set) == 0U)) {
        return 0.0F;
    }

    pid->get = get;
    pid->set = set;
    err = set - get;
    pid->iout += pid->cfg.ki * err;
    pid->iout = direct_limit(pid->iout, pid->cfg.max_iout);
    out = (pid->cfg.kp * err) + pid->iout + (pid->cfg.kd * (err - pid->last_err));
    pid->out = direct_limit(out, pid->cfg.max_out);
    pid->last_err = err;
    pid->err = err;
    return pid->out;
}

/**
 * @brief 将编码器值转换为相对角。
 *
 * @param cfg 公共配置。
 * @param ecd 当前编码器。
 * @param offset 中值编码器。
 * @param out 输出相对角，单位 rad。
 * @return 执行状态。
 */
static GimbalStatus direct_ecd_to_angle(const GimbalCfg *cfg, uint16_t ecd, uint16_t offset, float32_t *out)
{
    int32_t delta;

    if ((cfg == NULL) || (out == NULL)) {
        return GimbalNull;
    }

    if ((cfg->ecd_range <= 0) ||
        (cfg->half_ecd_range <= 0) ||
        (cfg->ecd_to_rad <= 0.0F)) {
        return GimbalValue;
    }

    delta = (int32_t)ecd - (int32_t)offset;
    if (delta > cfg->half_ecd_range) {
        delta -= cfg->ecd_range;
    } else if (delta < -cfg->half_ecd_range) {
        delta += cfg->ecd_range;
    }

    *out = (float32_t)delta * cfg->ecd_to_rad;
    return GimbalOk;
}

/**
 * @brief 初始化直驱单轴。
 *
 * @param axis 轴状态。
 * @param abs_cfg 绝对角 PID 参数。
 * @param rel_cfg 相对角 PID 参数。
 * @param speed_cfg 速度 PID 参数。
 */
static void direct_axis_init(GimbalDirectAxis *axis,
                             const GimbalPidCfg *abs_cfg,
                             const GimbalPidCfg *rel_cfg,
                             const GimbalPidCfg *speed_cfg)
{
    if (axis == NULL) {
        return;
    }

    memset(axis, 0, sizeof(*axis));
    axis->mode = GimbalModeZero;
    axis->last_mode = GimbalModeZero;
    direct_pid_init(&axis->abs_pid, abs_cfg);
    direct_pid_init(&axis->rel_pid, rel_cfg);
    direct_pid_init(&axis->speed_pid, speed_cfg);
}

/**
 * @brief 同步单轴目标到当前反馈。
 *
 * @param axis 轴状态。
 */
static void direct_axis_sync(GimbalDirectAxis *axis)
{
    if (axis == NULL) {
        return;
    }

    axis->relative_set = axis->relative;
    axis->absolute_set = axis->absolute;
    axis->ref_rate = 0.0F;
    axis->gyro_set = 0.0F;
    axis->raw_current = 0.0F;
    axis->current_set = 0.0F;
    axis->given_current = 0;
    direct_pid_clear(&axis->abs_pid);
    direct_pid_clear(&axis->rel_pid);
    direct_pid_clear(&axis->speed_pid);
}

/**
 * @brief 按相对角限位钳位。
 *
 * @param axis 轴状态。
 * @param value 输入目标角，单位 rad。
 * @return 钳位后的目标角，单位 rad。
 */
static float32_t direct_clamp_relative(const GimbalDirectAxis *axis, float32_t value)
{
    if ((axis == NULL) || (axis->limit_valid == 0U)) {
        return value;
    }

    if (value > axis->max_relative) {
        return axis->max_relative;
    }

    if (value < axis->min_relative) {
        return axis->min_relative;
    }

    return value;
}

/**
 * @brief 根据模式更新单轴控制目标。
 *
 * @param axis 轴状态。
 * @param mode 控制模式。
 * @param add 控制增量。
 * @param period_s 控制周期，单位 s。
 * @param rate_enable 非 0 表示由控制增量生成参考角速度。
 */
static void direct_axis_set(GimbalDirectAxis *axis,
                            GimbalMode mode,
                            float32_t add,
                            float32_t period_s,
                            uint8_t rate_enable)
{
    if (axis == NULL) {
        return;
    }

    axis->mode = mode;
    if (axis->last_mode != axis->mode) {
        direct_axis_sync(axis);
        axis->last_mode = axis->mode;
    }

    if (axis->mode == GimbalModeRaw) {
        axis->raw_current = add;
        axis->ref_rate = 0.0F;
    } else if (axis->mode == GimbalModeGyro) {
        axis->absolute_set = direct_wrap(axis->absolute_set + add);
        axis->ref_rate = (rate_enable != 0U) ? direct_ref_rate(add, period_s) : 0.0F;
    } else if (axis->mode == GimbalModeEncode) {
        axis->relative_set = direct_clamp_relative(axis, axis->relative_set + add);
        axis->ref_rate = (rate_enable != 0U) ? direct_ref_rate(add, period_s) : 0.0F;
    } else {
        axis->raw_current = 0.0F;
        axis->ref_rate = 0.0F;
        axis->gyro_set = 0.0F;
    }
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 运行单轴控制器。
 *
 * @param axis 轴状态。
 * @param turn 非 0 表示输出方向反向。
 * @param polarity 电流发送极性。
 * @param yaw_axis 非 0 表示当前轴按 yaw 二自由度速度前馈处理。
 */
static void direct_axis_control(GimbalDirectAxis *axis,
                                uint8_t turn,
                                int8_t polarity,
                                uint8_t yaw_axis)
{
    float32_t speed_set;
    float32_t correction;
    float32_t current;

    if (axis == NULL) {
        return;
    }

    if (axis->mode == GimbalModeRaw) {
        current = axis->raw_current;
    } else if (axis->mode == GimbalModeGyro) {
        correction = direct_pid_angle(&axis->abs_pid, axis->absolute, axis->absolute_set, -axis->gyro);
        speed_set = (yaw_axis != 0U) ? (axis->ref_rate + direct_yaw_correction(correction)) : correction;
        axis->gyro_set = speed_set;
        current = direct_pid_speed(&axis->speed_pid, axis->gyro, speed_set);
    } else if (axis->mode == GimbalModeEncode) {
        correction = direct_pid_angle(&axis->rel_pid, axis->relative, axis->relative_set, -axis->gyro);
        speed_set = (yaw_axis != 0U) ? (axis->ref_rate + direct_yaw_correction(correction)) : correction;
        axis->gyro_set = speed_set;
        current = direct_pid_speed(&axis->speed_pid, axis->gyro, speed_set);
    } else {
        current = 0.0F;
        axis->gyro_set = 0.0F;
    }

    if (turn != 0U) {
        current = -current;
    }

    current *= (float32_t)polarity;
    axis->current_set = current;
    axis->given_current = direct_i16(current);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
GimbalStatus gimbal_direct_init(Gimbal *gimbal, void *impl)
{
    GimbalDirectCtx *direct = (GimbalDirectCtx *)impl;

    if ((gimbal == NULL) || (direct == NULL)) {
        return GimbalNull;
    }

    memset(direct, 0, sizeof(*direct));
    direct_axis_init(&direct->yaw,
                     &gimbal->cfg.yaw_abs_pid,
                     &gimbal->cfg.yaw_rel_pid,
                     &gimbal->cfg.yaw_speed_pid);
    direct_axis_init(&direct->pitch,
                     &gimbal->cfg.pitch_abs_pid,
                     &gimbal->cfg.pitch_rel_pid,
                     &gimbal->cfg.pitch_speed_pid);
    direct->yaw.mode = GimbalModeZero;
    direct->pitch.mode = GimbalModeZero;
    return GimbalOk;
}

GimbalStatus gimbal_direct_step(Gimbal *gimbal,
                                void *impl,
                                const GimbalInput *input,
                                GimbalOutput *output)
{
    GimbalStatus status;

    if ((gimbal == NULL) || (impl == NULL) || (input == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    status = gimbal_direct_update(gimbal, impl, input);
    if (status != GimbalOk) {
        return status;
    }

    status = gimbal_direct_control(gimbal, impl, input);
    if (status != GimbalOk) {
        return status;
    }

    return gimbal_direct_output(gimbal, impl, output);
}

GimbalStatus gimbal_direct_update(Gimbal *gimbal, void *impl, const GimbalInput *input)
{
    GimbalDirectCtx *direct = (GimbalDirectCtx *)impl;
    float32_t angle;
    GimbalStatus status;

    if ((gimbal == NULL) || (direct == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    if ((input->cmd.online == 0U) ||
        (input->ins_ready == 0U) ||
        (input->yaw_fb.online == 0U) ||
        (input->pitch_fb.online == 0U)) {
        direct->yaw.mode = GimbalModeZero;
        direct->pitch.mode = GimbalModeZero;
        direct->feedback_ready = 0U;
        return GimbalOk;
    }

    direct->pitch.fb = input->pitch_fb;
    direct->pitch.absolute = input->ins_pitch;
    status = direct_ecd_to_angle(&gimbal->cfg,
                                 input->pitch_fb.ecd,
                                 direct->pitch.offset_ecd,
                                 &angle);
    if (status != GimbalOk) {
        return status;
    }
    direct->pitch.relative = (gimbal->cfg.pitch_turn != 0U) ? -angle : angle;
    direct->pitch.gyro = input->gyro_y;
    direct->pitch.speed_rpm = (float32_t)input->pitch_fb.speed_rpm;

    direct->yaw.fb = input->yaw_fb;
    direct->yaw.absolute = direct_wrap(input->ins_yaw);
    status = direct_ecd_to_angle(&gimbal->cfg,
                                 input->yaw_fb.ecd,
                                 direct->yaw.offset_ecd,
                                 &angle);
    if (status != GimbalOk) {
        return status;
    }
    direct->yaw.relative = (gimbal->cfg.yaw_turn != 0U) ? -angle : angle;
    direct->yaw.gyro = (arm_cos_f32(direct->pitch.relative) * input->gyro_z) -
                       (arm_sin_f32(direct->pitch.relative) * input->gyro_x);
    direct->yaw.speed_rpm = (float32_t)input->yaw_fb.speed_rpm;

    if (direct->feedback_ready == 0U) {
        (void)gimbal_direct_sync(gimbal, impl);
        direct->feedback_ready = 1U;
    }

    return GimbalOk;
}

GimbalStatus gimbal_direct_control(Gimbal *gimbal, void *impl, const GimbalInput *input)
{
    GimbalDirectCtx *direct = (GimbalDirectCtx *)impl;
    float32_t period_s;

    if ((gimbal == NULL) || (direct == NULL) || (input == NULL)) {
        return GimbalNull;
    }

    period_s = direct_period_s(gimbal);
    if ((input->cmd.online == 0U) ||
        (input->ins_ready == 0U) ||
        (input->yaw_fb.online == 0U) ||
        (input->pitch_fb.online == 0U)) {
        direct_axis_set(&direct->yaw, GimbalModeZero, 0.0F, period_s, 1U);
        direct_axis_set(&direct->pitch, GimbalModeZero, 0.0F, period_s, 0U);
    } else {
        direct_axis_set(&direct->yaw, input->cmd.mode, input->cmd.yaw, period_s, 1U);
        direct_axis_set(&direct->pitch, input->cmd.mode, input->cmd.pitch, period_s, 0U);
    }

    direct_axis_control(&direct->yaw, gimbal->cfg.yaw_turn, gimbal->cfg.yaw_current_polarity, 1U);
    direct_axis_control(&direct->pitch, gimbal->cfg.pitch_turn, gimbal->cfg.pitch_current_polarity, 0U);
    return GimbalOk;
}

GimbalStatus gimbal_direct_output(const Gimbal *gimbal, void *impl, GimbalOutput *output)
{
    GimbalDirectCtx *direct = (GimbalDirectCtx *)impl;

    if ((gimbal == NULL) || (direct == NULL) || (output == NULL)) {
        return GimbalNull;
    }

    memset(output, 0, sizeof(*output));
    output->yaw_current = direct->yaw.given_current;
    output->pitch_current = direct->pitch.given_current;
    output->mode = direct->yaw.mode;
    output->send_enable = (direct->feedback_ready != 0U) ? 1U : 0U;
    return GimbalOk;
}

GimbalStatus gimbal_direct_sync(Gimbal *gimbal, void *impl)
{
    GimbalDirectCtx *direct = (GimbalDirectCtx *)impl;

    if ((gimbal == NULL) || (direct == NULL)) {
        return GimbalNull;
    }

    direct_axis_sync(&direct->yaw);
    direct_axis_sync(&direct->pitch);
    return GimbalOk;
}

GimbalStatus gimbal_direct_set_cali(Gimbal *gimbal, void *impl, const GimbalCali *cali)
{
    GimbalDirectCtx *direct = (GimbalDirectCtx *)impl;

    if ((gimbal == NULL) || (direct == NULL) || (cali == NULL)) {
        return GimbalNull;
    }

    direct->cali = *cali;
    direct->yaw.offset_ecd = cali->yaw_offset;
    direct->yaw.max_relative = cali->yaw_max;
    direct->yaw.min_relative = cali->yaw_min;
    direct->yaw.limit_valid = (cali->yaw_max > cali->yaw_min) ? 1U : 0U;
    direct->pitch.offset_ecd = cali->pitch_offset;
    direct->pitch.max_relative = cali->pitch_max;
    direct->pitch.min_relative = cali->pitch_min;
    direct->pitch.limit_valid = (cali->pitch_max > cali->pitch_min) ? 1U : 0U;
    gimbal->cfg.yaw_current_polarity = cali->yaw_polarity;
    gimbal->cfg.pitch_current_polarity = cali->pitch_polarity;
    return GimbalOk;
}
