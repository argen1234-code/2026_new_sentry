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

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_CAN_TEST_CONFIG_H */
