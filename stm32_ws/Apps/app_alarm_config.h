/**
 ******************************************************************************
 * @file    app_alarm_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   告警应用层配置映射。
 *
 * @note    本文件只把 Robot target 中的告警参数收敛为 App 内部短宏。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_ALARM_CONFIG_H
#define PRINTK_APPS_ALARM_CONFIG_H

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

#ifndef ROBOT_APP_ALARM_PERIOD_MS
#define ROBOT_APP_ALARM_PERIOD_MS 10U
#endif

#ifndef ROBOT_APP_ALARM_STACK_SIZE
#define ROBOT_APP_ALARM_STACK_SIZE 1024U
#endif

#ifndef ROBOT_APP_ALARM_PRIORITY
#define ROBOT_APP_ALARM_PRIORITY osPriorityLow
#endif

#ifndef ROBOT_APP_ALARM_BEEP_OFF_MS
#define ROBOT_APP_ALARM_BEEP_OFF_MS 500U
#endif

#ifndef ROBOT_APP_ALARM_BEEP_ON_MS
#define ROBOT_APP_ALARM_BEEP_ON_MS 500U
#endif

#ifndef ROBOT_APP_ALARM_REPEAT_GAP_MS
#define ROBOT_APP_ALARM_REPEAT_GAP_MS 1000U
#endif

#ifndef ROBOT_APP_ALARM_FREQ_HZ
#define ROBOT_APP_ALARM_FREQ_HZ 3000U
#endif

#ifndef ROBOT_APP_ALARM_DUTY
#define ROBOT_APP_ALARM_DUTY 500U
#endif

#ifndef ROBOT_APP_ALARM_MAX_BEEP
#define ROBOT_APP_ALARM_MAX_BEEP 16U
#endif

/** @brief Alarm 周期，单位 ms。 */
#define APP_ALARM_PERIOD_MS ROBOT_APP_ALARM_PERIOD_MS

/** @brief Alarm 线程栈大小，单位 byte。 */
#define APP_ALARM_STACK_SIZE ROBOT_APP_ALARM_STACK_SIZE

/** @brief Alarm 线程优先级。 */
#define APP_ALARM_PRIORITY ROBOT_APP_ALARM_PRIORITY

/** @brief 单次响声前的静音时间，单位 ms。 */
#define APP_ALARM_BEEP_OFF_MS ROBOT_APP_ALARM_BEEP_OFF_MS

/** @brief 单次响声持续时间，单位 ms。 */
#define APP_ALARM_BEEP_ON_MS ROBOT_APP_ALARM_BEEP_ON_MS

/** @brief 一组响声结束后的重复间隔，单位 ms。 */
#define APP_ALARM_REPEAT_GAP_MS ROBOT_APP_ALARM_REPEAT_GAP_MS

/** @brief 蜂鸣器告警频率，单位 Hz。 */
#define APP_ALARM_FREQ_HZ ROBOT_APP_ALARM_FREQ_HZ

/** @brief 蜂鸣器告警占空比，单位千分比。 */
#define APP_ALARM_DUTY ROBOT_APP_ALARM_DUTY

/** @brief 单组告警最多响声次数。 */
#define APP_ALARM_MAX_BEEP ROBOT_APP_ALARM_MAX_BEEP

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_ALARM_CONFIG_H */
