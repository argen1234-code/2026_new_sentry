/**
 ******************************************************************************
 * @file    app_can_test_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-19
 * @brief   CAN1 DMH6215 台架测试配置。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_CAN_TEST_CONFIG_H
#define PRINTK_APPS_CAN_TEST_CONFIG_H

#include "cmsis_os.h"
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef APP_CAN_TEST_STACK_SIZE
#define APP_CAN_TEST_STACK_SIZE                   1024U
#endif

#ifndef APP_CAN_TEST_PRIORITY
#define APP_CAN_TEST_PRIORITY                     osPriorityNormal
#endif

/** @brief 测试任务周期，单位 ms。 */
#ifndef APP_CAN_TEST_PERIOD_MS
#define APP_CAN_TEST_PERIOD_MS                    10U
#endif

/** @brief 扫描阶段时长，单位 ms。 */
#ifndef APP_CAN_TEST_SCAN_MS
#define APP_CAN_TEST_SCAN_MS                      2000U
#endif

/** @brief 驱动阶段时长，单位 ms。 */
#ifndef APP_CAN_TEST_DRIVE_MS
#define APP_CAN_TEST_DRIVE_MS                     4000U
#endif

/** @brief 悬空测试位置幅值，单位 rad。 */
#ifndef APP_CAN_TEST_AMP_RAD
#define APP_CAN_TEST_AMP_RAD                      0.10F
#endif

/** @brief 指定测试电机 ID；0 表示自动锁定第一个反馈电机。 */
#ifndef APP_CAN_TEST_TARGET_ID
#define APP_CAN_TEST_TARGET_ID                    0U
#endif

/** @brief 停止专用构建：只发零速/零力矩/失能，然后静默。 */
#ifndef APP_CAN_TEST_STOP_ONLY
#define APP_CAN_TEST_STOP_ONLY                    0U
#endif

/** @brief 扫描专用构建：只发零指令并记录响应的电机 ID，不驱动。 */
#ifndef APP_CAN_TEST_SCAN_ONLY
#define APP_CAN_TEST_SCAN_ONLY                    0U
#endif

/** @brief 持续运行速度目标，单位 rad/s。 */
#ifndef APP_CAN_TEST_VEL_RAD_S
#define APP_CAN_TEST_VEL_RAD_S                    3.0F
#endif

/** @brief 驱动阶段时长，单位 ms；0 表示一直运行。 */
#ifndef APP_CAN_TEST_RUN_MS
#define APP_CAN_TEST_RUN_MS                       0U
#endif

/** @brief 速度闭环阻尼（MIT 模式 Kd）。 */
#ifndef APP_CAN_TEST_VEL_KD
#define APP_CAN_TEST_VEL_KD                       1.0F
#endif

/** @brief 悬空测试位置刚度。 */
#ifndef APP_CAN_TEST_KP
#define APP_CAN_TEST_KP                           5.0F
#endif

/** @brief 悬空测试速度阻尼。 */
#ifndef APP_CAN_TEST_KD
#define APP_CAN_TEST_KD                           0.5F
#endif

/* ==========================================================================
 * 保守驱动测试
 * --------------------------------------------------------------------------
 * 只在指定 ID 的电机上做低速、限时、低增益 MIT 速度测试：
 *   T = Kd * (v_des - v_actual)，Kp=0、t_ff=0。
 * 因此最大输出力矩被限制为 Kd * |v_des|，适合在机械限位附近首次试转。
 * ========================================================================== */

/** @brief 保守测试构建：低速 MIT 速度模式，限时后自动零速/失能。 */
#ifndef APP_CAN_TEST_CONSERVATIVE
#define APP_CAN_TEST_CONSERVATIVE                 0U
#endif

/**
 * @brief 保守测试使用电机原生速度模式（float rad/s），而不是 MIT Kd 速度环。
 * @note  空载轮毂在 MIT Kd 速度环下容易超调，原生速度环由电机内部闭环。
 */
#ifndef APP_CAN_TEST_SAFE_NATIVE_SPEED
#define APP_CAN_TEST_SAFE_NATIVE_SPEED            0U
#endif

/**
 * @brief 四轮统一前进测试：原生速度模式，低速限时，结束后零速失能。
 * @note  只用于台架/地面直行测试，ID 1..4 对应右后、前左、左后、前右。
 */
#ifndef APP_CAN_TEST_ALL_FORWARD
#define APP_CAN_TEST_ALL_FORWARD                  0U
#endif

#ifndef APP_CAN_TEST_ALL_VEL_RAD_S
#define APP_CAN_TEST_ALL_VEL_RAD_S                0.3F
#endif

#ifndef APP_CAN_TEST_ALL_RUN_MS
#define APP_CAN_TEST_ALL_RUN_MS                   20000U
#endif

#ifndef APP_CAN_TEST_ALL_RAMP_MS
#define APP_CAN_TEST_ALL_RAMP_MS                  500U
#endif

#ifndef APP_CAN_TEST_ALL_ABORT_VEL_RAD_S
#define APP_CAN_TEST_ALL_ABORT_VEL_RAD_S          2.0F
#endif

/** @brief 四轮测试的电机使能掩码：bit0..bit3 对应 ID1..ID4。 */
#ifndef APP_CAN_TEST_ALL_MASK
#define APP_CAN_TEST_ALL_MASK                     0x0FU
#endif

/**
 * @brief 单轮遍历诊断：依次对 ID1..ID4 做低速正向速度测试并记录反馈。
 */
#ifndef APP_CAN_TEST_SWEEP
#define APP_CAN_TEST_SWEEP                        0U
#endif

#ifndef APP_CAN_TEST_SWEEP_VEL_RAD_S
#define APP_CAN_TEST_SWEEP_VEL_RAD_S              0.2F
#endif

#ifndef APP_CAN_TEST_SWEEP_RUN_MS
#define APP_CAN_TEST_SWEEP_RUN_MS                 2000U
#endif

#ifndef APP_CAN_TEST_SWEEP_RAMP_MS
#define APP_CAN_TEST_SWEEP_RAMP_MS                300U
#endif

#ifndef APP_CAN_TEST_SWEEP_ABORT_VEL_RAD_S
#define APP_CAN_TEST_SWEEP_ABORT_VEL_RAD_S        2.0F
#endif

/** @brief 保守测试目标速度，单位 rad/s。符号决定试转方向。 */
#ifndef APP_CAN_TEST_SAFE_VEL_RAD_S
#define APP_CAN_TEST_SAFE_VEL_RAD_S               0.5F
#endif

/** @brief 保守测试速度阻尼（MIT Kd），最大力矩约为 Kd * 目标速度。 */
#ifndef APP_CAN_TEST_SAFE_KD
#define APP_CAN_TEST_SAFE_KD                      0.5F
#endif

/** @brief 使能后的零力矩稳定时间，单位 ms。 */
#ifndef APP_CAN_TEST_SAFE_SETTLE_MS
#define APP_CAN_TEST_SAFE_SETTLE_MS               300U
#endif

/** @brief 速度斜坡时间，单位 ms。 */
#ifndef APP_CAN_TEST_SAFE_RAMP_MS
#define APP_CAN_TEST_SAFE_RAMP_MS                 500U
#endif

/** @brief 目标速度保持时间，单位 ms。 */
#ifndef APP_CAN_TEST_SAFE_HOLD_MS
#define APP_CAN_TEST_SAFE_HOLD_MS                 1000U
#endif

/** @brief 保守测试总超时，单位 ms；到时无条件停止。 */
#ifndef APP_CAN_TEST_SAFE_TIMEOUT_MS
#define APP_CAN_TEST_SAFE_TIMEOUT_MS              3000U
#endif

/** @brief 启动阶段等待首帧反馈的超时，单位 ms。 */
#ifndef APP_CAN_TEST_SAFE_START_FB_TIMEOUT_MS
#define APP_CAN_TEST_SAFE_START_FB_TIMEOUT_MS     2000U
#endif

/** @brief 运行阶段反馈中断超时，单位 ms。 */
#ifndef APP_CAN_TEST_SAFE_FB_TIMEOUT_MS
#define APP_CAN_TEST_SAFE_FB_TIMEOUT_MS           200U
#endif

/** @brief 允许的位置变化上限，单位 rad；超过立即停止。 */
#ifndef APP_CAN_TEST_SAFE_POS_LIMIT_RAD
#define APP_CAN_TEST_SAFE_POS_LIMIT_RAD           1.0F
#endif

/** @brief 堵转判定时间，单位 ms。 */
#ifndef APP_CAN_TEST_SAFE_STALL_MS
#define APP_CAN_TEST_SAFE_STALL_MS                800U
#endif

/** @brief 堵转判定速度阈值，单位 rad/s。 */
#ifndef APP_CAN_TEST_SAFE_STALL_VEL_RAD_S
#define APP_CAN_TEST_SAFE_STALL_VEL_RAD_S         0.02F
#endif

/** @brief 停止序列重复次数（零速 + 零力矩 + 失能）。 */
#ifndef APP_CAN_TEST_SAFE_STOP_ROUNDS
#define APP_CAN_TEST_SAFE_STOP_ROUNDS             10U
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_CAN_TEST_CONFIG_H */
