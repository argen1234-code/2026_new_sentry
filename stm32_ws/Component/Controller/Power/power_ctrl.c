/**
 ******************************************************************************
 * @file    power_ctrl.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   底盘功率预测控制器实现。
 *
 * @note    核心模型来自旧底盘功率预测逻辑：P=a+wI+k1*w^2+k2*I^2。
 * @note    当预测功率超过预算时，求解二次方程得到统一电流缩放系数。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "power_ctrl.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 浮点求解使用的极小阈值。 */
#define POWER_CTRL_EPS 1.0e-9F

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 判断浮点数是否有限。
 *
 * @param value 输入数值。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t power_ctrl_finite(float32_t value)
{
    return (isfinite((double)value) != 0) ? 1U : 0U;
}

/**
 * @brief 选择配置值或默认值。
 *
 * @param value 配置值。
 * @param default_value 默认值。
 * @return 配置值有效且非 0 时返回配置值，否则返回默认值。
 */
static float32_t power_ctrl_pick(float32_t value, float32_t default_value)
{
    if ((value != 0.0F) && (power_ctrl_finite(value) != 0U)) {
        return value;
    }

    return default_value;
}

/**
 * @brief 将数值限制在指定区间内。
 *
 * @param value 输入值。
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @return 限幅后的数值。
 */
static float32_t power_ctrl_clamp(float32_t value, float32_t min_value, float32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief 判断缩放系数是否落在可用解区间。
 *
 * @param value 待检查的缩放系数。
 * @return 合法返回 1，否则返回 0。
 */
static uint8_t power_ctrl_scale_ok(float32_t value)
{
    return ((value > 0.0F) && (value <= 1.0F) && (power_ctrl_finite(value) != 0U)) ? 1U : 0U;
}

/**
 * @brief 生成默认配置。
 *
 * @param cfg 输出配置。
 */
static void power_ctrl_default(PowerCtrlCfg *cfg)
{
    cfg->k1 = POWER_CTRL_K1_DEFAULT;
    cfg->k2 = POWER_CTRL_K2_DEFAULT;
    cfg->offset = POWER_CTRL_OFFSET_DEFAULT;
    cfg->torque_coeff = POWER_CTRL_TORQUE_DEFAULT;
    cfg->rpm_to_rad = POWER_CTRL_RPM_RAD_DEFAULT;
    cfg->reserve_power = POWER_CTRL_RESERVE_DEFAULT;
    cfg->min_scale = POWER_CTRL_SCALE_MIN;
    cfg->fallback_scale = POWER_CTRL_SCALE_FALLBACK;
}

/**
 * @brief 规范化配置参数。
 *
 * @param cfg 输出配置。
 * @param input 用户配置。
 * @return 执行状态。
 */
static PowerCtrlStatus power_ctrl_cfg(PowerCtrlCfg *cfg, const PowerCtrlCfg *input)
{
    if (cfg == NULL) {
        return PowerCtrlNull;
    }

    power_ctrl_default(cfg);
    if (input != NULL) {
        cfg->k1 = power_ctrl_pick(input->k1, cfg->k1);
        cfg->k2 = power_ctrl_pick(input->k2, cfg->k2);
        cfg->offset = power_ctrl_pick(input->offset, cfg->offset);
        cfg->torque_coeff = power_ctrl_pick(input->torque_coeff, cfg->torque_coeff);
        cfg->rpm_to_rad = power_ctrl_pick(input->rpm_to_rad, cfg->rpm_to_rad);
        cfg->reserve_power = power_ctrl_pick(input->reserve_power, cfg->reserve_power);
        cfg->min_scale = power_ctrl_pick(input->min_scale, cfg->min_scale);
        cfg->fallback_scale = power_ctrl_pick(input->fallback_scale, cfg->fallback_scale);
    }

    if ((cfg->k1 < 0.0F) || (cfg->k2 <= 0.0F) || (cfg->torque_coeff <= 0.0F) ||
        (cfg->rpm_to_rad <= 0.0F) || (cfg->min_scale <= 0.0F) ||
        (cfg->min_scale > 1.0F) || (cfg->fallback_scale <= 0.0F)) {
        return PowerCtrlArg;
    }

    cfg->fallback_scale = power_ctrl_clamp(cfg->fallback_scale, cfg->min_scale, 1.0F);
    return PowerCtrlOk;
}

/**
 * @brief 从二次方程中求解缩放系数。
 *
 * @param cfg 控制器配置。
 * @param sum_i2 电流平方累加项。
 * @param sum_wi 速度电流交叉项。
 * @param sum_w2 速度平方累加项。
 * @param available_power 可用功率。
 * @return 缩放系数。
 */
static float32_t power_ctrl_solve(const PowerCtrlCfg *cfg,
                                  float32_t sum_i2,
                                  float32_t sum_wi,
                                  float32_t sum_w2,
                                  float32_t available_power)
{
    float32_t a = cfg->k2 * sum_i2;
    float32_t b = sum_wi;
    float32_t c = cfg->offset + (cfg->k1 * sum_w2) - available_power;
    float32_t scale = cfg->fallback_scale;

    if (fabsf(a) <= POWER_CTRL_EPS) {
        if (fabsf(b) > POWER_CTRL_EPS) {
            scale = -c / b;
        }
        return power_ctrl_clamp(scale, cfg->min_scale, 1.0F);
    }

    {
        float32_t disc = (b * b) - (4.0F * a * c);
        float32_t sqrt_disc;
        float32_t root_a;
        float32_t root_b;

        if ((disc < 0.0F) || (power_ctrl_finite(disc) == 0U)) {
            return cfg->fallback_scale;
        }

        sqrt_disc = sqrtf(disc);
        root_a = (-b + sqrt_disc) / (2.0F * a);
        root_b = (-b - sqrt_disc) / (2.0F * a);

        if (power_ctrl_scale_ok(root_a) != 0U) {
            scale = root_a;
        } else if (power_ctrl_scale_ok(root_b) != 0U) {
            scale = root_b;
        }
    }

    return power_ctrl_clamp(scale, cfg->min_scale, 1.0F);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
PowerCtrlStatus power_ctrl_init(PowerCtrl *ctrl, const PowerCtrlCfg *cfg)
{
    PowerCtrlStatus status;

    if (ctrl == NULL) {
        return PowerCtrlNull;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    status = power_ctrl_cfg(&ctrl->cfg, cfg);
    if (status != PowerCtrlOk) {
        return status;
    }

    ctrl->last.scale = 1.0F;
    ctrl->initialized = 1U;
    return PowerCtrlOk;
}

PowerCtrlStatus power_ctrl_step(PowerCtrl *ctrl, const PowerCtrlInput *input, PowerCtrlOutput *output)
{
    float32_t sum_wi = 0.0F;
    float32_t sum_w2 = 0.0F;
    float32_t sum_i2 = 0.0F;
    float32_t predicted;
    float32_t available;
    uint32_t count;
    uint32_t used = 0U;
    uint32_t i;

    if ((ctrl == NULL) || (input == NULL) || (output == NULL)) {
        return PowerCtrlNull;
    }

    if (ctrl->initialized == 0U) {
        return PowerCtrlUninit;
    }

    if ((input->motor_count == 0U) || (input->motor_count > POWER_CTRL_MOTOR_MAX)) {
        return PowerCtrlArg;
    }

    if ((power_ctrl_finite(input->power_limit) == 0U) ||
        (power_ctrl_finite(input->measured_power) == 0U) ||
        (power_ctrl_finite(input->buffer_energy) == 0U)) {
        return PowerCtrlValue;
    }

    memset(output, 0, sizeof(*output));
    output->scale = 1.0F;
    ctrl->measured_power = input->measured_power;
    ctrl->buffer_energy = input->buffer_energy;

    if (input->power_limit <= 0.0F) {
        ctrl->last = *output;
        return PowerCtrlOk;
    }

    available = input->power_limit - ctrl->cfg.reserve_power;
    output->available_power = available;
    if (available <= 0.0F) {
        output->scale = ctrl->cfg.min_scale;
        output->predicted_power = ctrl->cfg.offset;
        ctrl->last = *output;
        return PowerCtrlOk;
    }

    count = input->motor_count;
    for (i = 0U; i < count; i++) {
        const PowerCtrlMotor *motor = &input->motor[i];

        if (motor->online == 0U) {
            continue;
        }

        if ((power_ctrl_finite(motor->speed_rpm) == 0U) ||
            (power_ctrl_finite(motor->current) == 0U)) {
            return PowerCtrlValue;
        }

        /*
         * 旧模型使用 rpm 反馈和 DJI 电流命令直接参与预测，
         * torque_coeff / rpm_to_rad 保持旧实现里的单位折算。
         */
        sum_wi += motor->speed_rpm * motor->current * ctrl->cfg.torque_coeff / ctrl->cfg.rpm_to_rad;
        sum_w2 += motor->speed_rpm * motor->speed_rpm;
        sum_i2 += motor->current * motor->current;
        used++;
    }

    if (used == 0U) {
        ctrl->last = *output;
        return PowerCtrlOk;
    }

    predicted = ctrl->cfg.offset + sum_wi + (ctrl->cfg.k1 * sum_w2) + (ctrl->cfg.k2 * sum_i2);
    output->predicted_power = predicted;

    if (predicted > available) {
        output->scale = power_ctrl_solve(&ctrl->cfg, sum_i2, sum_wi, sum_w2, available);
    }

    ctrl->last = *output;
    return PowerCtrlOk;
}

float32_t power_ctrl_scale(PowerCtrl *ctrl, const PowerCtrlInput *input)
{
    PowerCtrlOutput output;

    if (power_ctrl_step(ctrl, input, &output) != PowerCtrlOk) {
        return 1.0F;
    }

    return output.scale;
}
