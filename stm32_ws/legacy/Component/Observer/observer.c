/**
 ******************************************************************************
 * @file    observer.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   观测器组件公共入口实现。
 *
 * @note    本文件只做参数检查、公共上下文管理和编译期开关转发。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "observer.h"

#include "Src/observer_balance.h"
#include "Src/observer_chassis.h"
#include "Src/observer_common.h"
#include "Src/observer_ins.h"

#include <string.h>

/**
 * @brief 当前使用的默认 Mahony 比例增益。
 */
#ifndef OBSERVER_MAHONY_KP_DEFAULT
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
#define OBSERVER_MAHONY_KP_DEFAULT 1.0F
#endif

/**
 * @brief 当前使用的默认 Mahony 积分增益。
 */
#ifndef OBSERVER_MAHONY_KI_DEFAULT
#define OBSERVER_MAHONY_KI_DEFAULT 0.0F
#endif

/**
 * @brief 当前使用的默认 Mahony 周期，单位 s。
 */
#ifndef OBSERVER_MAHONY_DT_DEFAULT
#define OBSERVER_MAHONY_DT_DEFAULT 0.001F
#endif

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 选择非零浮点配置或默认值。
 *
 * @param value 配置值。
 * @param def 默认值。
 * @return 选择后的值。
 */
static float32_t cfg_default_f32(float32_t value, float32_t def)
{
    return (value == 0.0F) ? def : value;
}

/**
 * @brief 选择非零 uint32_t 配置或默认值。
 *
 * @param value 配置值。
 * @param def 默认值。
 * @return 选择后的值。
 */
static uint32_t cfg_default_u32(uint32_t value, uint32_t def)
{
    return (value == 0U) ? def : value;
}

/**
 * @brief 填充单腿默认几何配置。
 *
 * @param cfg 单腿配置。
 * @param side 腿索引。
 */
static void fill_leg_default(ObserverLegCfg *cfg, uint32_t side)
{
    cfg->l1 = cfg_default_f32(cfg->l1, OBSERVER_LEG_L1_DEFAULT);
    cfg->l2 = cfg_default_f32(cfg->l2, OBSERVER_LEG_L2_DEFAULT);
    cfg->l3 = cfg_default_f32(cfg->l3, OBSERVER_LEG_L3_DEFAULT);
    cfg->l4 = cfg_default_f32(cfg->l4, OBSERVER_LEG_L4_DEFAULT);
    cfg->l5 = cfg_default_f32(cfg->l5, OBSERVER_LEG_L5_DEFAULT);
    if (cfg->phi1_sign == 0) {
        cfg->phi1_sign = 1;
    }
    if (cfg->phi4_sign == 0) {
        cfg->phi4_sign = 1;
    }
    if (cfg->pitch_sign == 0.0F) {
        cfg->pitch_sign = (side == OBSERVER_LEG_LEFT) ? -1.0F : 1.0F;
    }
    if (cfg->gyro_sign == 0.0F) {
        cfg->gyro_sign = (side == OBSERVER_LEG_LEFT) ? -1.0F : 1.0F;
    }
}

/**
 * @brief 填充观测器默认配置。
 *
 * @param cfg 观测器配置。
 */
static void fill_cfg_default(ObserverCfg *cfg)
{
    uint32_t i;

    cfg->mahony.kp = cfg_default_f32(cfg->mahony.kp, OBSERVER_MAHONY_KP_DEFAULT);
    cfg->mahony.dt = cfg_default_f32(cfg->mahony.dt, OBSERVER_MAHONY_DT_DEFAULT);
    cfg->gravity = cfg_default_f32(cfg->gravity, OBSERVER_GRAVITY_DEFAULT);
    cfg->ready_time = cfg_default_f32(cfg->ready_time, OBSERVER_READY_TIME_DEFAULT);
    cfg->stable_gyro = cfg_default_f32(cfg->stable_gyro, OBSERVER_STABLE_GYRO_DEFAULT);
    cfg->stable_accel = cfg_default_f32(cfg->stable_accel, OBSERVER_STABLE_ACCEL_DEFAULT);
    cfg->accel_lpf = cfg_default_f32(cfg->accel_lpf, OBSERVER_ACCEL_LPF_DEFAULT);
    cfg->accel_dead_xy = cfg_default_f32(cfg->accel_dead_xy, OBSERVER_ACCEL_DEAD_XY_DEFAULT);
    cfg->accel_dead_z = cfg_default_f32(cfg->accel_dead_z, OBSERVER_ACCEL_DEAD_Z_DEFAULT);
    cfg->gyro_cali_samples =
        cfg_default_u32(cfg->gyro_cali_samples, OBSERVER_GYRO_CALI_SAMPLES_DEFAULT);
    cfg->cali_gyro_th = cfg_default_f32(cfg->cali_gyro_th, OBSERVER_CALI_GYRO_TH_DEFAULT);
    cfg->cali_accel_th = cfg_default_f32(cfg->cali_accel_th, OBSERVER_CALI_ACCEL_TH_DEFAULT);
    cfg->kf_p = cfg_default_f32(cfg->kf_p, OBSERVER_KF_P_DEFAULT);
    cfg->kf_q = cfg_default_f32(cfg->kf_q, OBSERVER_KF_Q_DEFAULT);
    cfg->kf_r = cfg_default_f32(cfg->kf_r, OBSERVER_KF_R_DEFAULT);
    cfg->kf_dt = cfg_default_f32(cfg->kf_dt, OBSERVER_KF_DT_DEFAULT);
    cfg->ground_threshold = cfg_default_f32(cfg->ground_threshold, OBSERVER_GROUND_TH_DEFAULT);
    cfg->wheel_weight = cfg_default_f32(cfg->wheel_weight, OBSERVER_WHEEL_WEIGHT_DEFAULT);

    for (i = 0U; i < OBSERVER_LEG_NUM; ++i) {
        fill_leg_default(&cfg->leg[i], i);
    }
}

/**
 * @brief 检查单腿配置。
 *
 * @param cfg 单腿配置。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t check_leg_cfg(const ObserverLegCfg *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }

    if ((observer_finite(cfg->l1) == 0U) ||
        (observer_finite(cfg->l2) == 0U) ||
        (observer_finite(cfg->l3) == 0U) ||
        (observer_finite(cfg->l4) == 0U) ||
        (observer_finite(cfg->l5) == 0U) ||
        (observer_finite(cfg->phi1_offset) == 0U) ||
        (observer_finite(cfg->phi4_offset) == 0U) ||
        (observer_finite(cfg->pitch_sign) == 0U) ||
        (observer_finite(cfg->gyro_sign) == 0U)) {
        return 0U;
    }

    return ((cfg->l1 > 0.0F) &&
            (cfg->l2 > 0.0F) &&
            (cfg->l3 > 0.0F) &&
            (cfg->l4 > 0.0F)) ? 1U : 0U;
}

/**
 * @brief 检查观测器配置。
 *
 * @param cfg 观测器配置。
 * @return 执行状态。
 */
static ObserverStatus check_cfg(const ObserverCfg *cfg)
{
    uint32_t i;

    if (cfg == NULL) {
        return ObserverNull;
    }

    if ((observer_finite(cfg->mahony.kp) == 0U) ||
        (observer_finite(cfg->mahony.ki) == 0U) ||
        (observer_finite(cfg->mahony.dt) == 0U) ||
        (observer_finite(cfg->gravity) == 0U) ||
        (observer_finite(cfg->ready_time) == 0U) ||
        (observer_finite(cfg->stable_gyro) == 0U) ||
        (observer_finite(cfg->stable_accel) == 0U) ||
        (observer_finite(cfg->accel_lpf) == 0U) ||
        (observer_finite(cfg->accel_dead_xy) == 0U) ||
        (observer_finite(cfg->accel_dead_z) == 0U) ||
        (observer_finite(cfg->cali_gyro_th) == 0U) ||
        (observer_finite(cfg->cali_accel_th) == 0U) ||
        (observer_finite(cfg->kf_p) == 0U) ||
        (observer_finite(cfg->kf_q) == 0U) ||
        (observer_finite(cfg->kf_r) == 0U) ||
        (observer_finite(cfg->kf_dt) == 0U) ||
        (observer_finite(cfg->ground_threshold) == 0U) ||
        (observer_finite(cfg->wheel_weight) == 0U)) {
        return ObserverValue;
    }

    if ((cfg->mahony.kp < 0.0F) ||
        (cfg->mahony.ki < 0.0F) ||
        (cfg->mahony.dt <= 0.0F) ||
        (cfg->gravity <= 0.0F) ||
        (cfg->ready_time < 0.0F) ||
        (cfg->stable_gyro < 0.0F) ||
        (cfg->stable_accel < 0.0F) ||
        (cfg->accel_lpf < 0.0F) ||
        (cfg->accel_dead_xy < 0.0F) ||
        (cfg->accel_dead_z < 0.0F) ||
        (cfg->cali_gyro_th < 0.0F) ||
        (cfg->cali_accel_th < 0.0F) ||
        (cfg->kf_p <= 0.0F) ||
        (cfg->kf_q < 0.0F) ||
        (cfg->kf_r <= 0.0F) ||
        (cfg->kf_dt <= 0.0F)) {
        return ObserverArg;
    }

    for (i = 0U; i < OBSERVER_LEG_NUM; ++i) {
        if (check_leg_cfg(&cfg->leg[i]) == 0U) {
            return ObserverArg;
        }
    }

    return ObserverOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ObserverStatus observer_init(Observer *observer, const ObserverCfg *cfg)
{
    ObserverStatus status;
    ObserverCfg local_cfg;

    if ((observer == NULL) || (cfg == NULL)) {
        return ObserverNull;
    }

    local_cfg = *cfg;
    fill_cfg_default(&local_cfg);
    status = check_cfg(&local_cfg);
    if (status != ObserverOk) {
        return status;
    }

    (void)memset(observer, 0, sizeof(*observer));
    observer->cfg = local_cfg;
    observer_clear_state(&observer->state);

#if (OBSERVER_USE_INS != 0)
    status = observer_ins_init(observer);
    if (status != ObserverOk) {
        return status;
    }
#endif

#if (OBSERVER_USE_CHASSIS != 0)
    status = observer_chassis_init(observer);
    if (status != ObserverOk) {
        return status;
    }
#endif

#if (OBSERVER_USE_BALANCE != 0)
    status = observer_balance_init(observer);
    if (status != ObserverOk) {
        return status;
    }
#endif

    observer->initialized = 1U;
    return ObserverOk;
}

ObserverStatus observer_reset(Observer *observer)
{
    ObserverStatus status;

    if (observer == NULL) {
        return ObserverNull;
    }

    if (observer->initialized == 0U) {
        return ObserverUninit;
    }

    observer_clear_state(&observer->state);

#if (OBSERVER_USE_INS != 0)
    status = observer_ins_reset(observer);
    if (status != ObserverOk) {
        return status;
    }
#endif

#if (OBSERVER_USE_CHASSIS != 0)
    status = observer_chassis_reset(observer);
    if (status != ObserverOk) {
        return status;
    }
#endif

#if (OBSERVER_USE_BALANCE != 0)
    status = observer_balance_reset(observer);
    if (status != ObserverOk) {
        return status;
    }
#endif

    return ObserverOk;
}

ObserverStatus observer_update(Observer *observer, const ObserverInput *input)
{
    ObserverStatus status;

    if ((observer == NULL) || (input == NULL)) {
        return ObserverNull;
    }

    if (observer->initialized == 0U) {
        return ObserverUninit;
    }

    observer->state.timestamp_us =
        (input->timestamp_us != 0U) ? input->timestamp_us : input->imu.timestamp_us;

#if (OBSERVER_USE_INS != 0)
    status = observer_ins_update(observer, input);
    if (status != ObserverOk) {
        return status;
    }
#endif

#if (OBSERVER_USE_CHASSIS != 0)
    status = observer_chassis_update(observer, input);
    if (status != ObserverOk) {
        return status;
    }
#endif

#if (OBSERVER_USE_BALANCE != 0)
    status = observer_balance_update(observer, input);
    if (status != ObserverOk) {
        return status;
    }
#endif

    observer->state.timestamp_us =
        (input->timestamp_us != 0U) ? input->timestamp_us : input->imu.timestamp_us;
    return ObserverOk;
}

ObserverStatus observer_step(Observer *observer, const ObserverInput *input, ObserverState *state)
{
    ObserverStatus status;

    status = observer_update(observer, input);
    if (status != ObserverOk) {
        return status;
    }

    if (state != NULL) {
        return observer_get(observer, state);
    }

    return ObserverOk;
}

ObserverStatus observer_get(const Observer *observer, ObserverState *state)
{
    if ((observer == NULL) || (state == NULL)) {
        return ObserverNull;
    }

    if (observer->initialized == 0U) {
        return ObserverUninit;
    }

    return observer_copy_state(state, &observer->state);
}
