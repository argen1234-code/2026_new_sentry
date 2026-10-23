/**
 ******************************************************************************
 * @file    observer_ins.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   INS 姿态观测实现。
 *
 * @note    本文件组合 Mahony 姿态解算、启动零偏采样、yaw 连续角展开和运动加速度估计。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "observer_ins.h"

#include "observer_common.h"

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
 * @brief 将导航系向量转换到机体系。
 *
 * @param q body 到 earth 的姿态四元数。
 * @param earth 导航系向量。
 * @param body 机体系向量。
 */
static void earth_to_body(const Quat *q, const Vec3 *earth, Vec3 *body)
{
    const float32_t q0 = q->w;
    const float32_t q1 = q->x;
    const float32_t q2 = q->y;
    const float32_t q3 = q->z;

    body->x = 2.0F * (((0.5F - (q2 * q2) - (q3 * q3)) * earth->x) +
                      (((q1 * q2) + (q0 * q3)) * earth->y) +
                      (((q1 * q3) - (q0 * q2)) * earth->z));
    body->y = 2.0F * ((((q1 * q2) - (q0 * q3)) * earth->x) +
                      ((0.5F - (q1 * q1) - (q3 * q3)) * earth->y) +
                      (((q2 * q3) + (q0 * q1)) * earth->z));
    body->z = 2.0F * ((((q1 * q3) + (q0 * q2)) * earth->x) +
                      (((q2 * q3) - (q0 * q1)) * earth->y) +
                      ((0.5F - (q1 * q1) - (q2 * q2)) * earth->z));
}

/**
 * @brief 将机体系向量转换到导航系。
 *
 * @param q body 到 earth 的姿态四元数。
 * @param body 机体系向量。
 * @param earth 导航系向量。
 */
static void body_to_earth(const Quat *q, const Vec3 *body, Vec3 *earth)
{
    const float32_t q0 = q->w;
    const float32_t q1 = q->x;
    const float32_t q2 = q->y;
    const float32_t q3 = q->z;

    earth->x = 2.0F * (((0.5F - (q2 * q2) - (q3 * q3)) * body->x) +
                       (((q1 * q2) - (q0 * q3)) * body->y) +
                       (((q1 * q3) + (q0 * q2)) * body->z));
    earth->y = 2.0F * ((((q1 * q2) + (q0 * q3)) * body->x) +
                       ((0.5F - (q1 * q1) - (q3 * q3)) * body->y) +
                       (((q2 * q3) - (q0 * q1)) * body->z));
    earth->z = 2.0F * ((((q1 * q3) - (q0 * q2)) * body->x) +
                       (((q2 * q3) + (q0 * q1)) * body->y) +
                       ((0.5F - (q1 * q1) - (q2 * q2)) * body->z));
}

/**
 * @brief 判断当前 IMU 输入是否适合零偏采样。
 *
 * @param observer 观测器上下文。
 * @param input IMU 输入。
 * @return 适合采样返回 1，否则返回 0。
 */
static uint8_t cali_still(const Observer *observer, const ObserverImuInput *input)
{
    float32_t gyro_norm;
    float32_t accel_norm;

    if ((observer == NULL) || (input == NULL)) {
        return 0U;
    }

    gyro_norm = vec3_norm(&input->gyro);
    accel_norm = vec3_norm(&input->accel);
    if ((observer_finite(gyro_norm) == 0U) || (observer_finite(accel_norm) == 0U)) {
        return 0U;
    }

    if (gyro_norm > observer->cfg.cali_gyro_th) {
        return 0U;
    }

    if (fabsf(accel_norm - observer->cfg.gravity) > observer->cfg.cali_accel_th) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 计算并更新重力和运动加速度。
 *
 * @param observer 观测器上下文。
 * @param input IMU 输入。
 */
static void update_accel(Observer *observer, const ObserverImuInput *input)
{
    Vec3 gravity_world;
    Vec3 motion_body;
    Vec3 motion_world;
    float32_t alpha;

    gravity_world.x = 0.0F;
    gravity_world.y = 0.0F;
    gravity_world.z = observer->cfg.gravity;
    earth_to_body(&observer->state.ins.quat, &gravity_world, &observer->state.ins.gravity_body);

    motion_body.x = input->accel.x - observer->state.ins.gravity_body.x;
    motion_body.y = input->accel.y - observer->state.ins.gravity_body.y;
    motion_body.z = input->accel.z - observer->state.ins.gravity_body.z;

    if (observer->cfg.accel_lpf > 0.0F) {
        alpha = input->dt / (observer->cfg.accel_lpf + input->dt);
        alpha = observer_clamp(alpha, 0.0F, 1.0F);
    } else {
        alpha = 1.0F;
    }

    observer->motion_accel_body_lpf.x =
        (motion_body.x * alpha) + (observer->motion_accel_body_lpf.x * (1.0F - alpha));
    observer->motion_accel_body_lpf.y =
        (motion_body.y * alpha) + (observer->motion_accel_body_lpf.y * (1.0F - alpha));
    observer->motion_accel_body_lpf.z =
        (motion_body.z * alpha) + (observer->motion_accel_body_lpf.z * (1.0F - alpha));

    body_to_earth(&observer->state.ins.quat, &observer->motion_accel_body_lpf, &motion_world);
    motion_world.x = observer_deadband(motion_world.x, observer->cfg.accel_dead_xy);
    motion_world.y = observer_deadband(motion_world.y, observer->cfg.accel_dead_xy);
    motion_world.z = observer_deadband(motion_world.z, observer->cfg.accel_dead_z);

    observer->state.ins.motion_accel_body = observer->motion_accel_body_lpf;
    observer->state.ins.motion_accel_world = motion_world;
}

/**
 * @brief 将 Mahony 输出同步到 INS 状态。
 *
 * @param observer 观测器上下文。
 * @param input IMU 输入。
 * @param out Mahony 输出。
 */
static void sync_att(Observer *observer, const ObserverImuInput *input, const MahonyOutput *out)
{
    float32_t yaw_now = out->euler.yaw;
    float32_t yaw_diff;

    observer->state.ins.quat = out->quat;
    observer->state.ins.gyro = out->gyro;
    observer->state.ins.gyro_raw = input->gyro;
    observer->state.ins.accel = input->accel;
    observer->state.ins.gyro_bias = out->gyro_bias;
    observer->state.ins.accel_norm = out->accel_norm;
    observer->state.ins.gyro_norm = vec3_norm(&out->gyro);
    observer->state.ins.temperature = input->temperature;
    observer->state.ins.timestamp_us = input->timestamp_us;

    /*
     * 旧姿态任务对外语义是 Pitch=mahony.roll、Roll=mahony.pitch，
     * 这里保持现有控制链路所需的坐标约定。
     */
    observer->state.ins.euler.yaw = yaw_now;
    observer->state.ins.euler.pitch = out->euler.roll;
    observer->state.ins.euler.roll = out->euler.pitch;

    if (observer->yaw_last_valid == 0U) {
        observer->yaw_last = yaw_now;
        observer->yaw_round_count = 0;
        observer->yaw_last_valid = 1U;
    }

    yaw_diff = yaw_now - observer->yaw_last;
    if (yaw_diff > COMMON_PI) {
        observer->yaw_round_count--;
    } else if (yaw_diff < -COMMON_PI) {
        observer->yaw_round_count++;
    }

    observer->state.ins.yaw_total =
        (COMMON_2PI * (float32_t)observer->yaw_round_count) + yaw_now;
    observer->yaw_last = yaw_now;

    observer->state.ins.stable =
        ((observer->state.ins.gyro_norm < observer->cfg.stable_gyro) &&
         (fabsf(out->accel_norm - observer->cfg.gravity) < observer->cfg.stable_accel)) ? 1U : 0U;
    observer->state.ins.valid = 1U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
ObserverStatus observer_ins_init(Observer *observer)
{
    MahonyStatus status;

    if (observer == NULL) {
        return ObserverNull;
    }

    status = mahony_init(&observer->mahony, &observer->cfg.mahony);
    if (status != MahonyOk) {
        return ObserverFusion;
    }

    return observer_ins_reset(observer);
}

ObserverStatus observer_ins_reset(Observer *observer)
{
    if (observer == NULL) {
        return ObserverNull;
    }

    if (mahony_reset(&observer->mahony) != MahonyOk) {
        return ObserverFusion;
    }

    observer_clear_ins(&observer->state.ins);
    (void)memset(&observer->motion_accel_body_lpf, 0, sizeof(observer->motion_accel_body_lpf));
    observer->run_time = 0.0F;
    observer->yaw_last = 0.0F;
    observer->yaw_round_count = 0;
    observer->yaw_last_valid = 0U;
    observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_INS);

    return ObserverOk;
}

ObserverStatus observer_ins_update(Observer *observer, const ObserverInput *input)
{
    MahonyInput mahony_input;
    MahonyOutput mahony_output;
    const ObserverImuInput *imu;

    if ((observer == NULL) || (input == NULL)) {
        return ObserverNull;
    }

    if (observer->initialized == 0U) {
        return ObserverUninit;
    }

    imu = &input->imu;
    if ((imu->online == 0U)) {
        observer_clear_ins(&observer->state.ins);
        observer->yaw_last_valid = 0U;
        observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_INS);
        return ObserverOk;
    }

    if ((imu->dt <= 0.0F) ||
        (observer_finite(imu->dt) == 0U) ||
        (observer_finite(imu->temperature) == 0U) ||
        (observer_vec_ok(&imu->gyro) == 0U) ||
        (observer_vec_ok(&imu->accel) == 0U)) {
        observer_clear_ins(&observer->state.ins);
        observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_INS);
        return ObserverValue;
    }

    observer->run_time += imu->dt;
    mahony_input.gyro = imu->gyro;
    mahony_input.accel = imu->accel;
    mahony_input.dt = imu->dt;

    if (mahony_update(&observer->mahony, &mahony_input) != MahonyOk) {
        observer_clear_ins(&observer->state.ins);
        observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_INS);
        return ObserverFusion;
    }

    if ((observer->cfg.gyro_cali_samples > 0U) &&
        (observer->mahony.gyro_count < observer->cfg.gyro_cali_samples) &&
        (cali_still(observer, imu) != 0U)) {
        (void)mahony_cali_gyro(&observer->mahony, &imu->gyro, observer->cfg.gyro_cali_samples);
    }

    if (mahony_get(&observer->mahony, &mahony_output) != MahonyOk) {
        observer_clear_ins(&observer->state.ins);
        observer->state.valid_mask &= (uint8_t)(~OBSERVER_VALID_INS);
        return ObserverFusion;
    }

    sync_att(observer, imu, &mahony_output);
    update_accel(observer, imu);

    if ((observer->run_time >= observer->cfg.ready_time) &&
        ((observer->cfg.gyro_cali_samples == 0U) ||
         (observer->mahony.gyro_count >= observer->cfg.gyro_cali_samples))) {
        observer->state.ins.ready = 1U;
    }

    observer->state.valid_mask |= OBSERVER_VALID_INS;
    return ObserverOk;
}
