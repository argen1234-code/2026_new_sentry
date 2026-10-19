/**
 ******************************************************************************
 * @file    ballistic.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   弹道补偿算法接口。
 *
 * @note    本模块只提供纯函数弹道估计，不读取传感器、不访问任务、不直接接入自瞄链路。
 *
 * @note    Host 或 Aim 需要 MCU 侧弹道补偿时，可用本模块计算有限迭代的 pitch 补偿角。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_BALLISTIC_H
#define PRINTK_ALGORITHM_BALLISTIC_H

#include <stdint.h>

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BALLISTIC_GRAVITY_DEFAULT
#define BALLISTIC_GRAVITY_DEFAULT 9.80665F
#endif

#ifndef BALLISTIC_DRAG_K_DEFAULT
#define BALLISTIC_DRAG_K_DEFAULT 0.092F
#endif

#ifndef BALLISTIC_MUZZLE_SPEED_DEFAULT
#define BALLISTIC_MUZZLE_SPEED_DEFAULT 16.2F
#endif

#ifndef BALLISTIC_MUZZLE_FORWARD_DEFAULT
#define BALLISTIC_MUZZLE_FORWARD_DEFAULT 0.15F
#endif

#ifndef BALLISTIC_MUZZLE_UP_DEFAULT
#define BALLISTIC_MUZZLE_UP_DEFAULT 0.21F
#endif

#ifndef BALLISTIC_DT_DEFAULT
#define BALLISTIC_DT_DEFAULT 0.001F
#endif

#ifndef BALLISTIC_EPS_DEFAULT
#define BALLISTIC_EPS_DEFAULT 0.0001F
#endif

#ifndef BALLISTIC_MIN_DISTANCE_DEFAULT
#define BALLISTIC_MIN_DISTANCE_DEFAULT 0.1F
#endif

#ifndef BALLISTIC_MAX_STEPS_DEFAULT
#define BALLISTIC_MAX_STEPS_DEFAULT 2000U
#endif

#ifndef BALLISTIC_MAX_ITER_DEFAULT
#define BALLISTIC_MAX_ITER_DEFAULT 10U
#endif

/* ==========================================================================
 * 类型与配置
 * --------------------------------------------------------------------------
 * 弹道模块只暴露纯函数配置和结果状态，不主动接入 Aim/Gimbal 控制链路。
 * ========================================================================== */

/**
 * @brief 弹道算法执行状态。
 */
typedef enum
{
    BallisticOk = 0,       /**< 执行成功。 */
    BallisticNull = 1,     /**< 输出指针为空。 */
    BallisticArg = 2,      /**< 参数范围非法。 */
    BallisticValue = 3,    /**< 参数或中间结果出现 NaN/Inf。 */
    BallisticUnreach = 4   /**< 当前参数下没有得到有限可用解。 */
} BallisticStatus;

/**
 * @brief 弹道补偿配置。
 */
typedef struct
{
    float32_t gravity;        /**< 重力加速度，单位 m/s^2。 */
    float32_t drag_k;         /**< 二次空气阻力系数，旧弹道模型默认 0.092。 */
    float32_t muzzle_speed;   /**< 弹丸初速度，单位 m/s。 */
    float32_t muzzle_forward; /**< 枪口相对 yaw 轴水平前向偏置，单位 m。 */
    float32_t muzzle_up;      /**< 枪口相对 yaw 轴垂直偏置，单位 m。 */
    float32_t dt;             /**< 数值积分步长，单位 s。 */
    float32_t eps;            /**< 收敛阈值和最小速度阈值。 */
    float32_t min_distance;   /**< 最小有效距离，单位 m。 */
    uint16_t max_steps;       /**< 单次弹道积分最大步数。 */
    uint8_t max_iter;         /**< pitch 牛顿迭代最大次数。 */
} BallisticCfg;

/**
 * @brief 弹道补偿计算结果。
 */
typedef struct
{
    float32_t pitch;       /**< 计算得到的发射 pitch 角，单位 rad。 */
    float32_t flight_time; /**< 估计飞行时间，单位 s。 */
    float32_t impact_z;    /**< 该 pitch 下目标水平距离处的弹丸高度，单位 m。 */
    float32_t error_z;     /**< impact_z 与目标高度的误差，单位 m。 */
    uint8_t reachable;     /**< 非 0 表示本次得到有限可用解。 */
} BallisticResult;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按默认配置、拖拽补偿和完整 pitch 求解排列，调用方可按复杂度逐级使用。
 * ========================================================================== */
/**
 * @brief 填充默认弹道配置。
 *
 * @param cfg 配置输出。
 * @return 执行状态。
 */
BallisticStatus ballistic_default_cfg(BallisticCfg *cfg);

/**
 * @brief 按目标水平距离和高度计算带空气阻力的 pitch 补偿。
 *
 * horizontal 必须为正，vertical 可为正负，配置为空时使用默认配置。
 *
 * @param horizontal 目标水平距离，单位 m。
 * @param vertical 目标相对枪口高度，单位 m。
 * @param cfg 弹道配置，可为空。
 * @param out 计算结果输出。
 * @return 执行状态。
 */
BallisticStatus ballistic_drag_pitch(float32_t horizontal,
                                     float32_t vertical,
                                     const BallisticCfg *cfg,
                                     BallisticResult *out);

/**
 * @brief 按视觉测得的距离和目标仰角计算带枪口偏置的 pitch 补偿。
 *
 * 本接口等价替代旧 calculateCompensationAngle 的有界版本。
 *
 * @param shoot_angle 目标视线仰角，单位 rad。
 * @param distance 目标视线距离，单位 m。
 * @param cfg 弹道配置，可为空。
 * @param out 计算结果输出。
 * @return 执行状态。
 */
BallisticStatus ballistic_pitch(float32_t shoot_angle,
                                float32_t distance,
                                const BallisticCfg *cfg,
                                BallisticResult *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ALGORITHM_BALLISTIC_H */
