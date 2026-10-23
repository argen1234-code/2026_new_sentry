/**
 ******************************************************************************
 * @file    app_referee_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Referee 应用层配置。
 *
 * @note    本文件只放 Referee app 的任务周期、在线超时、UI 节流和当前 target 参数展开。
 * @note    裁判系统协议事实位于 Devices/Referee，客户端图元协议事实位于 Component/Ui。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_REFEREE_CONFIG_H
#define PRINTK_APPS_REFEREE_CONFIG_H

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
/** @brief Referee 任务栈大小，单位 byte。 */

#ifndef APP_REFEREE_STACK_SIZE
#define APP_REFEREE_STACK_SIZE 2048U
#endif

/** @brief Referee 任务优先级。 */
#ifndef APP_REFEREE_PRIORITY
#define APP_REFEREE_PRIORITY osPriorityBelowNormal
#endif

/** @brief Referee 应用层基础周期，单位 ms。 */
#ifndef APP_REFEREE_PERIOD_MS
#define APP_REFEREE_PERIOD_MS ROBOT_APP_REFEREE_PERIOD_MS
#endif

/** @brief 裁判系统在线超时，单位 ms。 */
#ifndef APP_REFEREE_TIMEOUT_MS
#define APP_REFEREE_TIMEOUT_MS ROBOT_APP_REFEREE_TIMEOUT_MS
#endif

/** @brief UI 发送节流周期，单位 ms。 */
#ifndef APP_REFEREE_UI_PERIOD_MS
#define APP_REFEREE_UI_PERIOD_MS ROBOT_APP_REFEREE_UI_PERIOD_MS
#endif

/** @brief 默认读取的 17mm 发射机构编号，1 或 2。 */
#ifndef APP_REFEREE_SHOOTER_ID
#define APP_REFEREE_SHOOTER_ID ROBOT_APP_REFEREE_SHOOTER_ID
#endif

/** @brief 每次 UI 周期最多发送的 0x0301 包数量。 */
#ifndef APP_REFEREE_UI_SEND_PER_STEP
#define APP_REFEREE_UI_SEND_PER_STEP ROBOT_APP_REFEREE_UI_SEND_PER_STEP
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_REFEREE_CONFIG_H */
