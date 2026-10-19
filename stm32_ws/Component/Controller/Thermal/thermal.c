/**
 ******************************************************************************
 * @file    thermal.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   温控控制器实现。
 *
 * @note    本文件只实现温度闭环和 compare 限幅，不包含任何 PWM/HAL 访问。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "thermal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief float32_t 可安全转换到 uint32_t 的最大值。 */
#define THERMAL_U32_MAX_F 4294967295.0F

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 判断浮点数是否为有限值。
 *
 * @param value 输入值。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t thermal_finite(float32_t value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

/**
 * @brief 选择非零配置值或默认值。
 *
 * @param value 配置值。
 * @param def 默认值。
 * @return 选择后的值。
 */
static float32_t thermal_default(float32_t value, float32_t def)
{
    return (value == 0.0F) ? def : value;
}

/**
 * @brief 裁剪浮点值到闭区间。
 *
 * @param value 输入值。
 * @param min_v 下限。
 * @param max_v 上限。
 * @return 裁剪后的值。
 */
static float32_t thermal_clamp(float32_t value, float32_t min_v, float32_t max_v)
{
    if (value < min_v) {
        return min_v;
    }

    if (value > max_v) {
        return max_v;
    }

    return value;
}

/**
 * @brief 按最大步进限制输出变化。
 *
 * @param current 当前值。
 * @param target 目标值。
 * @param step 单次最大变化量。
 * @return 限速后的值。
 */
static float32_t thermal_slew(float32_t current, float32_t target, float32_t step)
{
    if (step <= 0.0F) {
        return target;
    }

    if ((target - current) > step) {
        return current + step;
    }

    if ((current - target) > step) {
        return current - step;
    }

    return target;
}

/**
 * @brief 将浮点 compare 转换为 uint32_t compare。
 *
 * @param value 浮点 compare。
 * @return 四舍五入后的 compare。
 */
static uint32_t thermal_to_compare(float32_t value)
{
    float32_t limited = thermal_clamp(value, 0.0F, THERMAL_U32_MAX_F);

    return (uint32_t)(limited + 0.5F);
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 填充默认配置。
 *
 * @param cfg 温控配置。
 */
static void thermal_fill_default(ThermalCfg *cfg)
{
    cfg->pid.kp = thermal_default(cfg->pid.kp, THERMAL_PID_KP_DEFAULT);
    cfg->pid.ki = thermal_default(cfg->pid.ki, THERMAL_PID_KI_DEFAULT);
    cfg->pid.kd = thermal_default(cfg->pid.kd, THERMAL_PID_KD_DEFAULT);
    cfg->target = thermal_default(cfg->target, THERMAL_TARGET_DEFAULT);
    cfg->deadband = thermal_default(cfg->deadband, THERMAL_DEADBAND_DEFAULT);
    cfg->compare_min = thermal_default(cfg->compare_min, THERMAL_COMPARE_MIN_DEFAULT);
    cfg->compare_max = thermal_default(cfg->compare_max, THERMAL_COMPARE_MAX_DEFAULT);
    cfg->compare_slew = thermal_default(cfg->compare_slew, THERMAL_SLEW_DEFAULT);

    if (cfg->pid.max_out == 0.0F) {
        cfg->pid.max_out = cfg->compare_max;
    }
    if (cfg->pid.max_iout == 0.0F) {
        cfg->pid.max_iout = cfg->compare_max;
    }
}

/**
 * @brief 检查温控配置。
 *
 * @param cfg 温控配置。
 * @return 执行状态。
 */
static ThermalStatus thermal_check_cfg(const ThermalCfg *cfg)
{
    if (cfg == NULL) {
        return ThermalNull;
    }

    if ((thermal_finite(cfg->pid.kp) == 0U) ||
        (thermal_finite(cfg->pid.ki) == 0U) ||
        (thermal_finite(cfg->pid.kd) == 0U) ||
        (thermal_finite(cfg->pid.max_out) == 0U) ||
        (thermal_finite(cfg->pid.max_iout) == 0U) ||
        (thermal_finite(cfg->target) == 0U) ||
        (thermal_finite(cfg->deadband) == 0U) ||
        (thermal_finite(cfg->period) == 0U) ||
        (thermal_finite(cfg->compare_min) == 0U) ||
        (thermal_finite(cfg->compare_max) == 0U) ||
        (thermal_finite(cfg->compare_slew) == 0U)) {
        return ThermalValue;
    }

    if ((cfg->pid.kp < 0.0F) ||
        (cfg->pid.ki < 0.0F) ||
        (cfg->pid.kd < 0.0F) ||
        (cfg->pid.max_out < 0.0F) ||
        (cfg->pid.max_iout < 0.0F) ||
        (cfg->deadband < 0.0F) ||
        (cfg->period < 0.0F) ||
        (cfg->compare_min < 0.0F) ||
        (cfg->compare_max < cfg->compare_min) ||
        (cfg->compare_slew < 0.0F)) {
        return ThermalArg;
    }

    return ThermalOk;
}

/**
 * @brief 将 ThermalPidCfg 转换为算法层 PID 配置。
 *
 * @param cfg 温控 PID 配置。
 * @param out 算法层 PID 配置。
 */
static void thermal_make_pid(const ThermalPidCfg *cfg, pid_cfg_t *out)
{
    (void)memset(out, 0, sizeof(*out));
    out->kp = cfg->kp;
    out->ki = cfg->ki;
    out->kd = cfg->kd;
    out->max_out = cfg->max_out;
    out->max_iout = cfg->max_iout;
    out->mode = PID_LINEAR;
}

/**
 * @brief 清零温控输出。
 *
 * @param thermal 温控控制器上下文。
 */
static void thermal_zero_output(Thermal *thermal)
{
    thermal->output.compare = 0U;
    thermal->output.compare_f = 0.0F;
    thermal->output.pid_raw = 0.0F;
    thermal->output.enable = 0U;
    thermal->compare_state = 0.0F;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ThermalStatus thermal_init(Thermal *thermal, const ThermalCfg *cfg)
{
    ThermalCfg local_cfg;
    pid_cfg_t pid_cfg;
    ThermalStatus status;

    if ((thermal == NULL) || (cfg == NULL)) {
        return ThermalNull;
    }

    local_cfg = *cfg;
    thermal_fill_default(&local_cfg);
    status = thermal_check_cfg(&local_cfg);
    if (status != ThermalOk) {
        return status;
    }

    (void)memset(thermal, 0, sizeof(*thermal));
    thermal->cfg = local_cfg;
    thermal_make_pid(&thermal->cfg.pid, &pid_cfg);
    if (pid_init(&thermal->pid, &pid_cfg) != PID_CORE_OK) {
        return ThermalPid;
    }

    thermal->initialized = 1U;
    return thermal_reset(thermal);
}

ThermalStatus thermal_reset(Thermal *thermal)
{
    if (thermal == NULL) {
        return ThermalNull;
    }

    if (thermal->initialized == 0U) {
        return ThermalUninit;
    }

    if (pid_reset(&thermal->pid) != PID_CORE_OK) {
        return ThermalPid;
    }

    thermal->period_time = thermal->cfg.period;
    thermal_zero_output(thermal);
    thermal->output.target = thermal->cfg.target;

    return ThermalOk;
}

ThermalStatus thermal_update(Thermal *thermal, const ThermalInput *input)
{
    pid_in_t pid_in;
    pid_out_t pid_out;
    float32_t target;
    float32_t step;

    if ((thermal == NULL) || (input == NULL)) {
        return ThermalNull;
    }

    if (thermal->initialized == 0U) {
        return ThermalUninit;
    }

    if ((thermal_finite(input->temperature) == 0U) ||
        (thermal_finite(input->dt) == 0U) ||
        (input->dt <= 0.0F)) {
        thermal_zero_output(thermal);
        return ThermalValue;
    }

    thermal->output.target = thermal->cfg.target;
    thermal->output.temperature = input->temperature;

    if (input->online == 0U) {
        (void)pid_reset(&thermal->pid);
        thermal_zero_output(thermal);
        return ThermalOk;
    }

    if (input->temperature >= (thermal->cfg.target - thermal->cfg.deadband)) {
        (void)pid_reset(&thermal->pid);
        thermal_zero_output(thermal);
        thermal->output.target = thermal->cfg.target;
        thermal->output.temperature = input->temperature;
        return ThermalOk;
    }

    thermal->period_time += input->dt;
    if ((thermal->cfg.period > 0.0F) && (thermal->period_time < thermal->cfg.period)) {
        thermal->output.enable = (thermal->output.compare > 0U) ? 1U : 0U;
        return ThermalOk;
    }
    thermal->period_time = 0.0F;

    pid_in.set = thermal->cfg.target;
    pid_in.get = input->temperature;
    pid_in.d_in = 0.0F;
    pid_in.ff = 0.0F;
    if (pid_step(&thermal->pid, &pid_in, &pid_out) != PID_CORE_OK) {
        thermal_zero_output(thermal);
        return ThermalPid;
    }

    target = pid_out.out;
    if (target <= 0.0F) {
        target = 0.0F;
    } else {
        target = thermal_clamp(target, thermal->cfg.compare_min, thermal->cfg.compare_max);
    }

    if (thermal->cfg.compare_slew > 0.0F) {
        step = thermal->cfg.compare_slew * input->dt;
        target = thermal_slew(thermal->compare_state, target, step);
    }

    thermal->compare_state = thermal_clamp(target, 0.0F, thermal->cfg.compare_max);
    thermal->output.compare_f = thermal->compare_state;
    thermal->output.compare = thermal_to_compare(thermal->compare_state);
    thermal->output.pid_raw = pid_out.out;
    thermal->output.enable = (thermal->output.compare > 0U) ? 1U : 0U;

    return ThermalOk;
}

ThermalStatus thermal_output(const Thermal *thermal, ThermalOutput *output)
{
    if ((thermal == NULL) || (output == NULL)) {
        return ThermalNull;
    }

    if (thermal->initialized == 0U) {
        return ThermalUninit;
    }

    *output = thermal->output;
    return ThermalOk;
}

ThermalStatus thermal_step(Thermal *thermal, const ThermalInput *input, ThermalOutput *output)
{
    ThermalStatus status;

    status = thermal_update(thermal, input);
    if (status != ThermalOk) {
        return status;
    }

    return thermal_output(thermal, output);
}
