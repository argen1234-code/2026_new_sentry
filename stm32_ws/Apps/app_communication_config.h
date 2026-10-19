/**
 ******************************************************************************
 * @file    app_communication_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Communication 应用层配置。
 *
 * @note    本文件只放 Communication app 的任务周期、发送节流和当前 target
 *          参数展开。
 * @note    host 485 和 VOFA 的协议事实仍位于 Component/Communication。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_COMMUNICATION_CONFIG_H
#define PRINTK_APPS_COMMUNICATION_CONFIG_H

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
/** @brief Communication 任务栈大小，单位 byte。 */

#ifndef APP_COMM_STACK_SIZE
#define APP_COMM_STACK_SIZE 2048U
#endif

/** @brief Communication 任务优先级。 */
#ifndef APP_COMM_PRIORITY
#define APP_COMM_PRIORITY osPriorityBelowNormal
#endif

/** @brief Communication 任务基础周期，单位 ms。 */
#ifndef APP_COMM_PERIOD_MS
#define APP_COMM_PERIOD_MS 5U
#endif

/** @brief 云台状态上报周期，单位 ms。 */
#ifndef APP_COMM_STATE_PERIOD_MS
#define APP_COMM_STATE_PERIOD_MS 5U
#endif

/** @brief Debug 探针 VOFA 输出周期，单位 ms。 */
#ifndef APP_COMM_DEBUG_PERIOD_MS
#define APP_COMM_DEBUG_PERIOD_MS 20U
#endif

/** @brief USB 桥接状态上报周期，单位 ms。 */
#ifndef APP_COMM_USB_PERIOD_MS
#define APP_COMM_USB_PERIOD_MS 20U
#endif

/** @brief USB 桥接心跳周期，单位 ms。 */
#ifndef APP_COMM_USB_HEARTBEAT_MS
#define APP_COMM_USB_HEARTBEAT_MS 100U
#endif

/** @brief USB 桥接开关，由 Robot target 决定。 */
#ifndef ROBOT_APP_ENABLE_USB_BRIDGE
#define ROBOT_APP_ENABLE_USB_BRIDGE 0U
#endif

/** @brief 非 0 表示收到 host probe 后自动回复 ack。 */
#ifndef APP_COMM_AUTO_ACK_ENABLE
#define APP_COMM_AUTO_ACK_ENABLE 1U
#endif

/** @brief 非 0 表示周期输出 Debug 探针到 VOFA。 */
#ifndef APP_COMM_DEBUG_FLUSH_ENABLE
#define APP_COMM_DEBUG_FLUSH_ENABLE ROBOT_DEBUG_ENABLE
#endif

/** @brief 非 0 表示周期输出 profiler 采样到 VOFA。 */
#ifndef APP_COMM_DEBUG_PROFILER_ENABLE
#define APP_COMM_DEBUG_PROFILER_ENABLE ROBOT_DEBUG_PROFILER_ENABLE
#endif

/** @brief 非 0 表示本端主动维护 RTT/时钟偏移。 */
#ifndef APP_COMM_TIME_INITIATOR
#define APP_COMM_TIME_INITIATOR ROBOT_COMM_TIME_INITIATOR
#endif

/** @brief 时间同步可接受最大 RTT，单位 us，0 表示不限制。 */
#ifndef APP_COMM_TIME_MAX_RTT_US
#define APP_COMM_TIME_MAX_RTT_US ROBOT_COMM_TIME_MAX_RTT_US
#endif

/** @brief host 自瞄输入优先级。 */
#ifndef APP_COMM_AIM_PRIORITY
#define APP_COMM_AIM_PRIORITY ROBOT_COMM_AIM_PRIORITY
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_COMMUNICATION_CONFIG_H */
