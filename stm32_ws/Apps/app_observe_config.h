/**
 ******************************************************************************
 * @file    app_observe_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Observe 应用层配置。
 *
 * @note    本文件只放 Observe app 的任务周期、观测参数和温控参数默认值。
 * @note    具体车辆参数从 Robot/robot.h 进入，Component/Observer 不读取硬件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_OBSERVE_CONFIG_H
#define PRINTK_APPS_OBSERVE_CONFIG_H

#include "cmsis_os.h"
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 任务参数与 target 展开
 * --------------------------------------------------------------------------
 * 配置头只展开任务周期、超时、功能开关和 Robot target 参数，不在这里写运行时逻辑。
 * ========================================================================== */
/** @brief Observe 任务栈大小，单位 byte。 */

#ifndef APP_OBSERVE_STACK_SIZE
#define APP_OBSERVE_STACK_SIZE 3072U
#endif

/** @brief Observe 任务优先级。 */
#ifndef APP_OBSERVE_PRIORITY
#define APP_OBSERVE_PRIORITY osPriorityHigh
#endif

/** @brief Observe 任务周期，单位 ms。 */
#ifndef APP_OBSERVE_PERIOD_MS
#define APP_OBSERVE_PERIOD_MS 1U
#endif

/** @brief DWT 返回非法 dt 时使用的默认周期，单位 s。 */
#ifndef APP_OBSERVE_DEFAULT_DT
#define APP_OBSERVE_DEFAULT_DT 0.001F
#endif

/** @brief app 周期超过该值时认为调度断更，单位 s。 */
#ifndef APP_OBSERVE_STALE_DT
#define APP_OBSERVE_STALE_DT 0.1F
#endif

/** @brief 非 0 表示启用 INS 姿态观测。 */
#ifndef APP_OBSERVE_USE_INS
#define APP_OBSERVE_USE_INS ROBOT_OBSERVER_USE_INS
#endif

/** @brief 非 0 表示启用底盘速度观测。 */
#ifndef APP_OBSERVE_USE_CHASSIS
#define APP_OBSERVE_USE_CHASSIS ROBOT_OBSERVER_USE_CHASSIS
#endif

/** @brief 非 0 表示启用平衡腿状态观测。 */
#ifndef APP_OBSERVE_USE_BALANCE
#define APP_OBSERVE_USE_BALANCE ROBOT_OBSERVER_USE_BALANCE
#endif

/** @brief Mahony 比例反馈系数。 */
#ifndef APP_OBSERVE_MAHONY_KP
#define APP_OBSERVE_MAHONY_KP 1.0F
#endif

/** @brief Mahony 积分反馈系数。 */
#ifndef APP_OBSERVE_MAHONY_KI
#define APP_OBSERVE_MAHONY_KI 0.0F
#endif

/** @brief 运动加速度低通时间常数，单位 s。 */
#ifndef APP_OBSERVE_ACCEL_LPF
#define APP_OBSERVE_ACCEL_LPF 0.0089F
#endif

/** @brief 重力加速度，单位 m/s^2。 */
#ifndef APP_OBSERVE_GRAVITY
#define APP_OBSERVE_GRAVITY 9.81F
#endif

/** @brief 静止判据陀螺仪阈值，单位 rad/s。 */
#ifndef APP_OBSERVE_STABLE_GYRO
#define APP_OBSERVE_STABLE_GYRO 0.3F
#endif

/** @brief 静止判据加速度模长误差阈值，单位 m/s^2。 */
#ifndef APP_OBSERVE_STABLE_ACCEL
#define APP_OBSERVE_STABLE_ACCEL 0.5F
#endif

/** @brief 启动阶段陀螺仪零偏采样次数。 */
#ifndef APP_OBSERVE_GYRO_CALI_SAMPLES
#define APP_OBSERVE_GYRO_CALI_SAMPLES 1000U
#endif

/** @brief 零偏采样陀螺仪模长阈值，单位 rad/s。 */
#ifndef APP_OBSERVE_CALI_GYRO_TH
#define APP_OBSERVE_CALI_GYRO_TH 0.3F
#endif

/** @brief 零偏采样加速度模长误差阈值，单位 m/s^2。 */
#ifndef APP_OBSERVE_CALI_ACCEL_TH
#define APP_OBSERVE_CALI_ACCEL_TH 0.5F
#endif

/** @brief 运动加速度 X/Y 死区，单位 m/s^2。 */
#ifndef APP_OBSERVE_ACCEL_DEAD_XY
#define APP_OBSERVE_ACCEL_DEAD_XY 0.02F
#endif

/** @brief 运动加速度 Z 死区，单位 m/s^2。 */
#ifndef APP_OBSERVE_ACCEL_DEAD_Z
#define APP_OBSERVE_ACCEL_DEAD_Z 0.04F
#endif

/** @brief INS ready 时间，单位 s。 */
#ifndef APP_OBSERVE_READY_TIME
#define APP_OBSERVE_READY_TIME 3.0F
#endif

/** @brief 速度 Kalman 初始协方差。 */
#ifndef APP_OBSERVE_KF_P
#define APP_OBSERVE_KF_P 1.0F
#endif

/** @brief 速度 Kalman 过程噪声。 */
#ifndef APP_OBSERVE_KF_Q
#define APP_OBSERVE_KF_Q 0.5F
#endif

/** @brief 速度 Kalman 测量噪声。 */
#ifndef APP_OBSERVE_KF_R
#define APP_OBSERVE_KF_R 100.0F
#endif

/** @brief 速度 Kalman 状态转移 dt，单位 s。 */
#ifndef APP_OBSERVE_KF_DT
#define APP_OBSERVE_KF_DT 0.003F
#endif

/** @brief 非 0 表示启用 IMU 温控输出。 */
#ifndef APP_OBSERVE_HEAT_ENABLE
#if defined(ROBOT_MODE) && (ROBOT_MODE == ROBOT_MODE_RELEASE)
#define APP_OBSERVE_HEAT_ENABLE 1U
#else
#define APP_OBSERVE_HEAT_ENABLE 0U
#endif
#endif

/** @brief IMU 加热 compare 默认上限。 */
#ifndef APP_OBSERVE_HEAT_MAX
#define APP_OBSERVE_HEAT_MAX 500U
#endif

/** @brief IMU 温控硬关断温度，单位摄氏度。 */
#ifndef APP_OBSERVE_TEMP_SAFE_MAX
#define APP_OBSERVE_TEMP_SAFE_MAX 45.0F
#endif

/** @brief 温控 PID 比例系数。 */
#ifndef APP_OBSERVE_TEMP_KP
#define APP_OBSERVE_TEMP_KP 100.0F
#endif

/** @brief 温控 PID 积分系数。 */
#ifndef APP_OBSERVE_TEMP_KI
#define APP_OBSERVE_TEMP_KI 50.0F
#endif

/** @brief 温控 PID 微分系数。 */
#ifndef APP_OBSERVE_TEMP_KD
#define APP_OBSERVE_TEMP_KD 10.0F
#endif

/** @brief 温控计算周期，单位 s。 */
#ifndef APP_OBSERVE_TEMP_PERIOD
#define APP_OBSERVE_TEMP_PERIOD 1.0F
#endif

/** @brief 温控关断死区，单位摄氏度。 */
#ifndef APP_OBSERVE_TEMP_DEADBAND
#define APP_OBSERVE_TEMP_DEADBAND 0.25F
#endif

/** @brief 温控输出斜率限制，0 表示不限制。 */
#ifndef APP_OBSERVE_HEAT_SLEW
#define APP_OBSERVE_HEAT_SLEW 0.0F
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_OBSERVE_CONFIG_H */
