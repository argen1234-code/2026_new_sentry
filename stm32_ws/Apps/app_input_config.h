/**
 ******************************************************************************
 * @file    app_input_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Input 应用层配置。
 *
 * @note    本文件只放 Input app 的任务周期、超时和当前 target 的映射展开。
 * @note    遥控器按键宏来自 Devices/Rc，车辆参数来自 Robot/robot.h。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_INPUT_CONFIG_H
#define PRINTK_APPS_INPUT_CONFIG_H

#include "cmsis_os.h"
#include "rc.h"
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 任务参数与 target 展开
 * --------------------------------------------------------------------------
 * 配置头只展开任务周期、超时、功能开关和 Robot target 参数，不在这里写运行时逻辑。
 * ========================================================================== */
/** @brief Input 任务栈大小，单位 byte。 */

#ifndef APP_INPUT_STACK_SIZE
#define APP_INPUT_STACK_SIZE 2048U
#endif

/** @brief Input 任务优先级。 */
#ifndef APP_INPUT_PRIORITY
#define APP_INPUT_PRIORITY osPriorityAboveNormal
#endif

/** @brief Input 任务周期，单位 ms。 */
#ifndef APP_INPUT_PERIOD_MS
#define APP_INPUT_PERIOD_MS 2U
#endif

/** @brief Input 任务周期，单位 s。 */
#ifndef APP_INPUT_PERIOD_S
#define APP_INPUT_PERIOD_S 0.002F
#endif

/** @brief RC 超过该时间无新状态后判定离线，单位 ms。 */
#ifndef APP_INPUT_RC_LOST_MS
#define APP_INPUT_RC_LOST_MS 100U
#endif

/** @brief RC 输入源优先级。 */
#ifndef APP_INPUT_RC_PRIORITY
#define APP_INPUT_RC_PRIORITY 10U
#endif

/** @brief 非 0 表示从 Apps/app_communication.* 读取 host 自瞄误差。 */
#ifndef APP_INPUT_USE_COMM_AIM
#define APP_INPUT_USE_COMM_AIM 1U
#endif

/** @brief 180 度转向完成误差，单位 rad。 */
#ifndef APP_INPUT_TURN_DONE_ERR
#define APP_INPUT_TURN_DONE_ERR ROBOT_INPUT_GIMBAL_TURN_DONE_ERR
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_INPUT_CONFIG_H */
