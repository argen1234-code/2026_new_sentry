/**
 ******************************************************************************
 * @file    mahony.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   Mahony 姿态解算接口。
 *
 * @note    本模块按原始 DM-balanceV1 工程的 mahony_filter 实现迁移，负责
 *          由陀螺仪和加速度计估计姿态，不访问硬件和 RTOS。
 *
 * @note    输入角速度单位为 rad/s，加速度单位为 m/s^2，输出 Euler 角单位为 rad。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_MAHONY_H
#define PRINTK_ALGORITHM_MAHONY_H

#include <stdint.h>

#include "arm_math.h"
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 类型与配置
 * --------------------------------------------------------------------------
 * Mahony 类型只描述姿态解算需要的配置、输入和输出缓存，不绑定具体 IMU 设备。
 * ========================================================================== */
/**
 * @brief Mahony 执行状态。
 */
typedef enum
{
    MahonyOk = 0,       /**< 执行成功。 */
    MahonyNull = 1,     /**< 输入指针为空。 */
    MahonySize = 2,     /**< 参数范围非法。 */
    MahonyValue = 3,    /**< 输入或状态包含 NaN/Inf。 */
    MahonySingular = 4  /**< 加速度模长过小，无法归一化。 */
} MahonyStatus;

/**
 * @brief Mahony 配置。
 */
typedef struct
{
    float32_t kp; /**< 比例增益。 */
    float32_t ki; /**< 积分增益。 */
    float32_t dt; /**< 默认更新周期，单位 s。 */
} MahonyCfg;

/**
 * @brief Mahony 输入。
 */
typedef struct
{
    Vec3 gyro;       /**< 三轴角速度，单位 rad/s。 */
    Vec3 accel;      /**< 三轴加速度，单位 m/s^2。 */
    float32_t dt;    /**< 更新周期，单位 s。 */
} MahonyInput;

/**
 * @brief Mahony 输出。
 */
typedef struct
{
    Quat quat;             /**< 姿态四元数。 */
    Euler euler;           /**< Euler 角，单位 rad。 */
    float32_t gyro_z_bias; /**< 陀螺仪 Z 轴零偏估计，单位 rad/s。 */
    Vec3 gyro_bias;        /**< 三轴陀螺仪零偏估计，单位 rad/s。 */
    Vec3 gyro;             /**< 去零偏和误差修正后的角速度，单位 rad/s。 */
    float32_t accel_norm;  /**< 归一化前的加速度模长，单位 m/s^2。 */
    float32_t rmat[9];     /**< 旋转矩阵，行主序。 */
} MahonyOutput;

/**
 * @brief Mahony 上下文。
 */
typedef struct
{
    MahonyCfg cfg;       /**< 配置。 */
    MahonyOutput out;    /**< 当前输出。 */
    float32_t ex_int;    /**< 积分误差 X。 */
    float32_t ey_int;    /**< 积分误差 Y。 */
    float32_t ez_int;    /**< 积分误差 Z。 */
    Vec3 gyro_sum;       /**< 三轴零偏校准累计值。 */
    uint32_t gyro_count; /**< 三轴零偏校准采样数。 */
    uint8_t initialized; /**< 非 0 表示已经初始化。 */
} Mahony;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、复位、更新、零偏校准和读取输出排列，对应 Observer 的调用顺序。
 * ========================================================================== */
/**
 * @brief 初始化 Mahony。
 *
 * cfg 可为空；为空时使用默认参数。
 *
 * @param ctx Mahony 上下文。
 * @param cfg Mahony 配置。
 * @return 初始化状态。
 */
MahonyStatus mahony_init(Mahony *ctx, const MahonyCfg *cfg);

/**
 * @brief 复位 Mahony 动态状态。
 *
 * @param ctx Mahony 上下文。
 * @return 复位状态。
 */
MahonyStatus mahony_reset(Mahony *ctx);

/**
 * @brief 执行一次 Mahony 姿态更新。
 *
 * @param ctx Mahony 上下文。
 * @param input 本周期输入。
 * @return 更新状态。
 */
MahonyStatus mahony_update(Mahony *ctx, const MahonyInput *input);

/**
 * @brief 执行 Z 轴陀螺仪零偏均值校准。
 *
 * @param ctx Mahony 上下文。
 * @param gyro_z 当前 Z 轴角速度，单位 rad/s。
 * @param samples 目标采样数。
 * @return 校准状态。
 */
MahonyStatus mahony_cali_gyro_z(Mahony *ctx, float32_t gyro_z, uint32_t samples);

/**
 * @brief 执行三轴陀螺仪零偏均值校准。
 *
 * @param ctx Mahony 上下文。
 * @param gyro 当前三轴角速度，单位 rad/s。
 * @param samples 目标采样数。
 * @return 校准状态。
 */
MahonyStatus mahony_cali_gyro(Mahony *ctx, const Vec3 *gyro, uint32_t samples);

/**
 * @brief 设置 Z 轴陀螺仪零偏。
 *
 * @param ctx Mahony 上下文。
 * @param gyro_z_bias Z 轴陀螺仪零偏，单位 rad/s。
 * @return 设置状态。
 */
MahonyStatus mahony_set_gyro_z(Mahony *ctx, float32_t gyro_z_bias);

/**
 * @brief 设置三轴陀螺仪零偏。
 *
 * @param ctx Mahony 上下文。
 * @param gyro_bias 三轴陀螺仪零偏，单位 rad/s。
 * @return 设置状态。
 */
MahonyStatus mahony_set_gyro(Mahony *ctx, const Vec3 *gyro_bias);

/**
 * @brief 获取当前 Mahony 输出。
 *
 * @param ctx Mahony 上下文。
 * @param out 输出对象。
 * @return 获取状态。
 */
MahonyStatus mahony_get(const Mahony *ctx, MahonyOutput *out);

/**
 * @brief 获取状态码对应字符串。
 *
 * @param status 状态码。
 * @return 状态字符串。
 */
const char *mahony_status_str(MahonyStatus status);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_ALGORITHM_MAHONY_H
