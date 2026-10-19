/**
 ******************************************************************************
 * @file    aim.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   自瞄参考整形组件实现。
 *
 * @note    本文件迁移旧自瞄误差到角速度再到周期增量的核心整形逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "aim.h"

#include <math.h>
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
 * @brief 判断浮点值是否有限。
 *
 * @param value 待检查数值。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t aim_finite(float32_t value)
{
    return isfinite((double)value) ? 1U : 0U;
}

/**
 * @brief 按绝对值限幅。
 *
 * @param value 输入值。
 * @param limit 绝对值上限。
 * @return 限幅后的值。
 */
static float32_t aim_limit(float32_t value, float32_t limit)
{
    float32_t abs_limit = (limit >= 0.0F) ? limit : -limit;

    if (value > abs_limit) {
        return abs_limit;
    }
    if (value < -abs_limit) {
        return -abs_limit;
    }
    return value;
}

/**
 * @brief 钳位到闭区间。
 *
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @return 钳位后的值。
 */
static float32_t aim_clamp(float32_t value, float32_t min_value, float32_t max_value)
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
 * @brief 交换两个 float32_t 数值。
 *
 * @param a 第一个数值。
 * @param b 第二个数值。
 */
static void aim_swap(float32_t *a, float32_t *b)
{
    float32_t tmp;

    if ((a == NULL) || (b == NULL)) {
        return;
    }

    tmp = *a;
    *a = *b;
    *b = tmp;
}

/**
 * @brief 清空单轴整形状态。
 *
 * @param axis 单轴状态。
 */
static void aim_axis_clear(AimAxis *axis)
{
    if (axis == NULL) {
        return;
    }

    (void)memset(axis, 0, sizeof(*axis));
}

/**
 * @brief 清空单轴中值滤波状态，不影响控制器误差和速度状态。
 *
 * @param axis 单轴运行状态。
 */
static void aim_median_clear(AimAxis *axis)
{
    if (axis == NULL) {
        return;
    }

    (void)memset(axis->median_buf, 0, sizeof(axis->median_buf));
    axis->median_last = 0.0F;
    axis->median_idx = 0U;
    axis->zero_hold_cnt = 0U;
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 执行旧自瞄 5 点中值滤波和 0 值短保持。
 *
 * 旧视觉链路会在偶发丢帧时给出 0 增量。这里将短暂 0 值视作无效样本，
 * 先保持最近一次有效中值；连续超过配置次数后再清空滤波状态。
 *
 * @param axis 单轴运行状态。
 * @param value 本周期原始输出增量。
 * @param zero_hold_count 允许保持的连续 0 值次数。
 * @return 滤波后的输出增量。
 */
static float32_t aim_median_update(AimAxis *axis, float32_t value, uint8_t zero_hold_count)
{
    float32_t a;
    float32_t b;
    float32_t c;
    float32_t d;
    float32_t e;

    if (axis == NULL) {
        return value;
    }

    if (value == 0.0F) {
        if (axis->zero_hold_cnt < zero_hold_count) {
            axis->zero_hold_cnt++;
            return axis->median_last;
        }

        aim_median_clear(axis);
        return 0.0F;
    }

    axis->zero_hold_cnt = 0U;
    axis->median_buf[axis->median_idx] = value;
    axis->median_idx++;
    if (axis->median_idx >= 5U) {
        axis->median_idx = 0U;
    }

    a = axis->median_buf[0];
    b = axis->median_buf[1];
    c = axis->median_buf[2];
    d = axis->median_buf[3];
    e = axis->median_buf[4];

    if (a > b) { aim_swap(&a, &b); }
    if (d > e) { aim_swap(&d, &e); }
    if (a > c) { aim_swap(&a, &c); }
    if (b > c) { aim_swap(&b, &c); }
    if (a > d) { aim_swap(&a, &d); }
    if (c > d) { aim_swap(&c, &d); }
    if (b > e) { aim_swap(&b, &e); }
    if (b > c) { aim_swap(&b, &c); }
    if (d > e) { aim_swap(&d, &e); }

    axis->median_last = c;
    return c;
}

/**
 * @brief 校验单轴配置。
 *
 * @param cfg 单轴配置。
 * @return 合法返回 1，否则返回 0。
 */
static uint8_t aim_axis_cfg_ok(const AimAxisCfg *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }

    return ((aim_finite(cfg->kv) != 0U) &&
            (aim_finite(cfg->kd) != 0U) &&
            (aim_finite(cfg->omega_max) != 0U) &&
            (aim_finite(cfg->accel_max) != 0U) &&
            (cfg->omega_max > 0.0F) &&
            (cfg->accel_max > 0.0F)) ? 1U : 0U;
}

/**
 * @brief 校验 Aim 配置。
 *
 * @param cfg Aim 配置。
 * @return 合法返回 1，否则返回 0。
 */
static uint8_t aim_cfg_ok(const AimCfg *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }

    return ((aim_axis_cfg_ok(&cfg->yaw) != 0U) &&
            (aim_axis_cfg_ok(&cfg->pitch) != 0U) &&
            (aim_finite(cfg->err_soft_zone) != 0U) &&
            (aim_finite(cfg->accel_soft_zone) != 0U) &&
            (aim_finite(cfg->d_lpf_alpha) != 0U) &&
            (aim_finite(cfg->eps_angle) != 0U) &&
            (aim_finite(cfg->eps_omega) != 0U) &&
            (aim_finite(cfg->dt_min) != 0U) &&
            (aim_finite(cfg->dt_max) != 0U) &&
            (aim_finite(cfg->dt_default) != 0U) &&
            (cfg->err_soft_zone > 0.0F) &&
            (cfg->accel_soft_zone > 0.0F) &&
            (cfg->d_lpf_alpha >= 0.0F) &&
            (cfg->d_lpf_alpha <= 1.0F) &&
            (cfg->eps_angle >= 0.0F) &&
            (cfg->eps_omega >= 0.0F) &&
            (cfg->dt_min > 0.0F) &&
            (cfg->dt_max >= cfg->dt_min) &&
            (cfg->dt_default > 0.0F)) ? 1U : 0U;
}

/**
 * @brief 处理 dt，并按配置做安全钳位。
 *
 * @param cfg Aim 配置。
 * @param dt 输入 dt。
 * @return 可用于控制计算的 dt。
 */
static float32_t aim_dt(const AimCfg *cfg, float32_t dt)
{
    float32_t out;

    if (cfg == NULL) {
        return AIM_DT_DEFAULT;
    }

    out = ((aim_finite(dt) != 0U) && (dt > 0.0F)) ? dt : cfg->dt_default;
    return aim_clamp(out, cfg->dt_min, cfg->dt_max);
}

/**
 * @brief 计算误差软区内的有效 kv。
 *
 * 旧实现中目标附近将 kv 缩到 50%~100%，用于降低接近目标时的冲击。
 *
 * @param kv 原始 kv。
 * @param err_abs 误差绝对值。
 * @param zone 软区大小。
 * @return 有效 kv。
 */
static float32_t aim_soft_kv(float32_t kv, float32_t err_abs, float32_t zone)
{
    float32_t scale;

    if (err_abs >= zone) {
        return kv;
    }

    scale = err_abs / (zone + 1.0e-6F);
    return kv * (0.5F + (0.5F * scale));
}

/**
 * @brief 计算目标附近的有效加速度限幅。
 *
 * 旧实现中 10 度以内逐渐收紧加速度，至少保留 30% 加速度能力。
 *
 * @param accel 原始加速度上限。
 * @param err_abs 误差绝对值。
 * @param zone 软区大小。
 * @return 有效加速度上限。
 */
static float32_t aim_soft_accel(float32_t accel, float32_t err_abs, float32_t zone)
{
    float32_t scale;

    if (err_abs >= zone) {
        return accel;
    }

    scale = err_abs / (zone + 1.0e-3F);
    return accel * (0.3F + (0.7F * scale));
}

/**
 * @brief 执行单轴误差整形。
 *
 * @param axis 单轴运行状态。
 * @param cfg 单轴配置。
 * @param common 公共配置。
 * @param err 当前视觉误差。
 * @param dt 周期时间，单位 s。
 * @param out_delta 输出角度增量。
 * @return 执行状态。
 */
static AimStatus aim_axis_step(AimAxis *axis,
                               const AimAxisCfg *cfg,
                               const AimCfg *common,
                               float32_t err,
                               float32_t dt,
                               float32_t *out_delta)
{
    float32_t err_abs;
    float32_t kv;
    float32_t derr;
    float32_t omega_ref;
    float32_t accel;
    float32_t domega;
    float32_t domega_max;

    if ((axis == NULL) || (cfg == NULL) || (common == NULL) || (out_delta == NULL)) {
        return AimNull;
    }
    if ((aim_finite(err) == 0U) || (aim_finite(dt) == 0U) || (dt <= 0.0F)) {
        return AimValue;
    }

    err_abs = fabsf(err);
    kv = aim_soft_kv(cfg->kv, err_abs, common->err_soft_zone);
    derr = (axis->initialized != 0U) ? ((err - axis->last_err) / dt) : 0.0F;
    axis->last_err = err;
    axis->initialized = 1U;

    axis->d_lpf = (common->d_lpf_alpha * derr) +
                  ((1.0F - common->d_lpf_alpha) * axis->d_lpf);

    omega_ref = (kv * err) + (cfg->kd * axis->d_lpf);
    omega_ref = aim_limit(omega_ref, cfg->omega_max);

    accel = aim_soft_accel(cfg->accel_max, err_abs, common->accel_soft_zone);
    domega_max = accel * dt;
    domega = aim_limit(omega_ref - axis->omega_cmd, domega_max);
    axis->omega_cmd += domega;

    if ((fabsf(err) < common->eps_angle) &&
        (fabsf(axis->omega_cmd) < common->eps_omega)) {
        axis->omega_cmd = 0.0F;
    }

    *out_delta = axis->omega_cmd * dt;
    if (common->median_enable != 0U) {
        *out_delta = aim_median_update(axis, *out_delta, common->zero_hold_count);
    }
    return AimOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
AimStatus aim_default_cfg(AimCfg *cfg)
{
    if (cfg == NULL) {
        return AimNull;
    }

    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->yaw.kv = 4.0F;
    cfg->yaw.kd = 0.05F;
    cfg->yaw.omega_max = 3.14159265358979323846F;
    cfg->yaw.accel_max = 12.56637061435917295384F;
    cfg->pitch.kv = -2.0F;
    cfg->pitch.kd = 0.1F;
    cfg->pitch.omega_max = 3.14159265358979323846F;
    cfg->pitch.accel_max = 12.56637061435917295384F;
    cfg->err_soft_zone = 5.0F * 3.14159265358979323846F / 180.0F;
    cfg->accel_soft_zone = AIM_ACCEL_SOFT_ZONE_DEFAULT;
    cfg->d_lpf_alpha = 0.4F;
    cfg->eps_angle = 0.5F * 3.14159265358979323846F / 180.0F;
    cfg->eps_omega = 3.0F * 3.14159265358979323846F / 180.0F;
    cfg->dt_min = AIM_DT_MIN_DEFAULT;
    cfg->dt_max = AIM_DT_MAX_DEFAULT;
    cfg->dt_default = AIM_DT_DEFAULT;
    cfg->mode = GimbalModeGyro;
    cfg->median_enable = (uint8_t)AIM_MEDIAN_ENABLE_DEFAULT;
    cfg->zero_hold_count = (uint8_t)AIM_ZERO_HOLD_DEFAULT;

    return AimOk;
}

AimStatus aim_init(Aim *aim, const AimCfg *cfg)
{
    AimCfg local_cfg;

    if (aim == NULL) {
        return AimNull;
    }

    if (cfg == NULL) {
        (void)aim_default_cfg(&local_cfg);
        cfg = &local_cfg;
    }
    if (aim_cfg_ok(cfg) == 0U) {
        return AimArg;
    }

    (void)memset(aim, 0, sizeof(*aim));
    aim->cfg = *cfg;
    aim->initialized = 1U;
    return aim_reset(aim);
}

AimStatus aim_reset(Aim *aim)
{
    if (aim == NULL) {
        return AimNull;
    }
    if (aim->initialized == 0U) {
        return AimUninit;
    }

    aim_axis_clear(&aim->yaw);
    aim_axis_clear(&aim->pitch);
    (void)memset(&aim->output, 0, sizeof(aim->output));
    aim->output.mode = aim->cfg.mode;
    return AimOk;
}

AimStatus aim_step(Aim *aim, const AimInput *input, AimOutput *output)
{
    AimStatus status = AimOk;
    float32_t dt;
    AimOutput local_out;

    if ((aim == NULL) || (input == NULL) || (output == NULL)) {
        return AimNull;
    }
    if (aim->initialized == 0U) {
        return AimUninit;
    }

    (void)memset(&local_out, 0, sizeof(local_out));
    local_out.mode = aim->cfg.mode;
    local_out.timestamp_us = input->timestamp_us;

    if (input->enable == 0U) {
        (void)aim_reset(aim);
        *output = aim->output;
        return AimOk;
    }

    dt = aim_dt(&aim->cfg, input->dt);
    local_out.online = 1U;

    if ((input->valid_mask & AIM_YAW_VALID) != 0U) {
        status = aim_axis_step(&aim->yaw, &aim->cfg.yaw, &aim->cfg, input->yaw_err, dt, &local_out.yaw);
        if (status != AimOk) {
            (void)aim_reset(aim);
            *output = aim->output;
            return status;
        }
        local_out.valid_mask |= AIM_YAW_VALID;
    } else {
        aim_axis_clear(&aim->yaw);
    }

    if ((input->valid_mask & AIM_PITCH_VALID) != 0U) {
        status = aim_axis_step(&aim->pitch, &aim->cfg.pitch, &aim->cfg, input->pitch_err, dt, &local_out.pitch);
        if (status != AimOk) {
            (void)aim_reset(aim);
            *output = aim->output;
            return status;
        }
        local_out.valid_mask |= AIM_PITCH_VALID;
    } else {
        aim_axis_clear(&aim->pitch);
    }

    if (local_out.valid_mask == 0U) {
        local_out.online = 0U;
    }

    aim->output = local_out;
    *output = local_out;
    return AimOk;
}

AimStatus aim_get(const Aim *aim, AimOutput *output)
{
    if ((aim == NULL) || (output == NULL)) {
        return AimNull;
    }
    if (aim->initialized == 0U) {
        return AimUninit;
    }

    *output = aim->output;
    return AimOk;
}
