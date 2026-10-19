/**
 ******************************************************************************
 * @file    observer_chassis.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   底盘速度观测实现。
 *
 * @note    本文件把旧 observer 任务中的速度/加速度 Kalman 估计迁移为纯函数式组件实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "observer_chassis.h"

#include "observer_common.h"

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
 * @brief 获取三维向量指定轴的值。
 *
 * @param value 输入三维向量。
 * @param axis 轴序号。
 * @return 对应轴的值。
 */
static float32_t axis_get(const Vec3 *value, uint32_t axis)
{
    if (axis == 0U) {
        return value->x;
    }

    if (axis == 1U) {
        return value->y;
    }

    return value->z;
}

/**
 * @brief 设置三维向量指定轴的值。
 *
 * @param value 输出三维向量。
 * @param axis 轴序号。
 * @param data 需要写入的值。
 */
static void axis_set(Vec3 *value, uint32_t axis, float32_t data)
{
    if (axis == 0U) {
        value->x = data;
    } else if (axis == 1U) {
        value->y = data;
    } else {
        value->z = data;
    }
}

/**
 * @brief 获取轴序号对应的有效位。
 *
 * @param axis 轴序号。
 * @return 有效位掩码。
 */
static uint8_t axis_bit(uint32_t axis)
{
    return (uint8_t)(1U << axis);
}

/**
 * @brief 初始化单轴速度 Kalman。
 *
 * @param axis 单轴 Kalman 上下文。
 * @param cfg 观测器配置。
 * @return 执行状态。
 */
static ObserverStatus axis_kf_init(ObserverAxisKf *axis, const ObserverCfg *cfg)
{
    kf_cfg_t kf_cfg;

    if ((axis == NULL) || (cfg == NULL)) {
        return ObserverNull;
    }

    (void)memset(axis, 0, sizeof(*axis));

    axis->p[0] = cfg->kf_p;
    axis->p[3] = cfg->kf_p;
    axis->f[0] = 1.0F;
    axis->f[3] = 1.0F;
    axis->q[0] = cfg->kf_q;
    axis->q[3] = cfg->kf_q;
    axis->h[0] = 1.0F;
    axis->h[3] = 1.0F;
    axis->r[0] = cfg->kf_r;
    axis->r[3] = cfg->kf_r;

    kf_cfg.x_dim = 2U;
    kf_cfg.z_dim = 2U;
    kf_cfg.u_dim = 0U;
    kf_cfg.x = axis->x;
    kf_cfg.p = axis->p;
    kf_cfg.f = axis->f;
    kf_cfg.q = axis->q;
    kf_cfg.h = axis->h;
    kf_cfg.r = axis->r;
    kf_cfg.b = NULL;
    kf_cfg.u = NULL;
    kf_cfg.k = axis->k;
    kf_cfg.y = axis->y;
    kf_cfg.s = axis->s;
    kf_cfg.cov = KF_COV_SYMMETRIC;

    if (kf_init(&axis->kf, &kf_cfg) != KF_OK) {
        return ObserverKalman;
    }

    if (kf_work_init(&axis->work,
                     axis->work_buf,
                     OBSERVER_KF_WORK_LEN,
                     2U,
                     2U) != KF_OK) {
        return ObserverKalman;
    }

    axis->initialized = 1U;
    return ObserverOk;
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 更新单轴速度 Kalman。
 *
 * @param axis 单轴 Kalman 上下文。
 * @param dt 更新时间间隔，单位 s。
 * @param vel 速度测量。
 * @param acc 加速度测量。
 * @return 执行状态。
 */
static ObserverStatus axis_kf_update(ObserverAxisKf *axis,
                                     float32_t dt,
                                     float32_t vel,
                                     float32_t acc)
{
    float32_t z[2];

    if (axis == NULL) {
        return ObserverNull;
    }

    if (axis->initialized == 0U) {
        return ObserverUninit;
    }

    axis->f[1] = dt;
    z[0] = vel;
    z[1] = acc;

    if (kf_step(&axis->kf, &axis->work, z) != KF_OK) {
        return ObserverKalman;
    }

    return ObserverOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ObserverStatus observer_chassis_init(Observer *observer)
{
    uint32_t i;
    ObserverStatus status;

    if (observer == NULL) {
        return ObserverNull;
    }

    for (i = 0U; i < OBSERVER_CHASSIS_AXIS_NUM; ++i) {
        status = axis_kf_init(&observer->chassis_kf[i], &observer->cfg);
        if (status != ObserverOk) {
            return status;
        }
    }

    observer_clear_chassis(&observer->state.chassis);
    observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_CHASSIS);
    return ObserverOk;
}

ObserverStatus observer_chassis_reset(Observer *observer)
{
    if (observer == NULL) {
        return ObserverNull;
    }

    return observer_chassis_init(observer);
}

ObserverStatus observer_chassis_update(Observer *observer, const ObserverInput *input)
{
    uint32_t axis;
    uint8_t bit;
    uint8_t valid_mask = 0U;
    float32_t dt;
    float32_t vel;
    float32_t acc;
    ObserverStatus status;

    if ((observer == NULL) || (input == NULL)) {
        return ObserverNull;
    }

    if (observer->initialized == 0U) {
        return ObserverUninit;
    }

    dt = observer->cfg.kf_dt;
    if ((dt <= 0.0F) || (observer_finite(dt) == 0U)) {
        observer_clear_chassis(&observer->state.chassis);
        observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_CHASSIS);
        return ObserverValue;
    }

    observer_clear_chassis(&observer->state.chassis);
    for (axis = 0U; axis < OBSERVER_CHASSIS_AXIS_NUM; ++axis) {
        bit = axis_bit(axis);
        if (((input->vel_valid_mask & bit) == 0U) ||
            ((input->acc_valid_mask & bit) == 0U)) {
            continue;
        }

        vel = axis_get(&input->chassis_vel, axis);
        acc = axis_get(&input->chassis_acc, axis);
        if ((observer_finite(vel) == 0U) || (observer_finite(acc) == 0U)) {
            observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_CHASSIS);
            observer_clear_chassis(&observer->state.chassis);
            return ObserverValue;
        }

        status = axis_kf_update(&observer->chassis_kf[axis], dt, vel, acc);
        if (status != ObserverOk) {
            observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_CHASSIS);
            observer_clear_chassis(&observer->state.chassis);
            return status;
        }

        axis_set(&observer->state.chassis.vel, axis, observer->chassis_kf[axis].x[0]);
        axis_set(&observer->state.chassis.acc, axis, observer->chassis_kf[axis].x[1]);
        valid_mask |= bit;
    }

    observer->state.chassis.valid_mask = valid_mask;
    observer->state.chassis.valid = (valid_mask != 0U) ? 1U : 0U;
    observer->state.chassis.timestamp_us =
        (input->timestamp_us != 0U) ? input->timestamp_us : input->imu.timestamp_us;

    if (valid_mask != 0U) {
        observer->state.valid_mask |= OBSERVER_VALID_CHASSIS;
    } else {
        observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_CHASSIS);
    }

    return ObserverOk;
}
