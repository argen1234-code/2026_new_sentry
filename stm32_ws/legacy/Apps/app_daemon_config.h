/**
 ******************************************************************************
 * @file    app_daemon_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Daemon 应用层配置映射。
 *
 * @note    本文件只把 Robot target 宏收敛成 App 内部配置，不描述具体控制策略。
 * @note    当前健康项的默认超时和优先级来自旧健康检测表的业务语义。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_DAEMON_CONFIG_H
#define PRINTK_APPS_DAEMON_CONFIG_H

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

#ifndef ROBOT_APP_DAEMON_PERIOD_MS
#define ROBOT_APP_DAEMON_PERIOD_MS                10U
#endif

#ifndef ROBOT_APP_DAEMON_STACK_SIZE
#define ROBOT_APP_DAEMON_STACK_SIZE               2048U
#endif

#ifndef ROBOT_APP_DAEMON_PRIORITY
#define ROBOT_APP_DAEMON_PRIORITY                 osPriorityLow
#endif

#ifndef ROBOT_APP_DAEMON_EVENT_MAX
#define ROBOT_APP_DAEMON_EVENT_MAX                32U
#endif

#ifndef ROBOT_APP_DAEMON_INIT_DELAY_MS
#define ROBOT_APP_DAEMON_INIT_DELAY_MS            57U
#endif

#ifndef ROBOT_APP_DAEMON_USE_TFMINI
#define ROBOT_APP_DAEMON_USE_TFMINI               0U
#endif

#ifndef ROBOT_APP_DAEMON_RC_OFFLINE_MS
#define ROBOT_APP_DAEMON_RC_OFFLINE_MS            30U
#endif
#ifndef ROBOT_APP_DAEMON_RC_ONLINE_MS
#define ROBOT_APP_DAEMON_RC_ONLINE_MS             40U
#endif
#ifndef ROBOT_APP_DAEMON_RC_PRIORITY
#define ROBOT_APP_DAEMON_RC_PRIORITY              15U
#endif

#ifndef ROBOT_APP_DAEMON_REFEREE_OFFLINE_MS
#define ROBOT_APP_DAEMON_REFEREE_OFFLINE_MS       100U
#endif
#ifndef ROBOT_APP_DAEMON_REFEREE_ONLINE_MS
#define ROBOT_APP_DAEMON_REFEREE_ONLINE_MS        100U
#endif
#ifndef ROBOT_APP_DAEMON_REFEREE_PRIORITY
#define ROBOT_APP_DAEMON_REFEREE_PRIORITY         5U
#endif

#ifndef ROBOT_APP_DAEMON_COMM_OFFLINE_MS
#define ROBOT_APP_DAEMON_COMM_OFFLINE_MS          100U
#endif
#ifndef ROBOT_APP_DAEMON_COMM_ONLINE_MS
#define ROBOT_APP_DAEMON_COMM_ONLINE_MS           100U
#endif
#ifndef ROBOT_APP_DAEMON_COMM_PRIORITY
#define ROBOT_APP_DAEMON_COMM_PRIORITY            6U
#endif

#ifndef ROBOT_APP_DAEMON_VISION_OFFLINE_MS
#define ROBOT_APP_DAEMON_VISION_OFFLINE_MS        100U
#endif
#ifndef ROBOT_APP_DAEMON_VISION_ONLINE_MS
#define ROBOT_APP_DAEMON_VISION_ONLINE_MS         100U
#endif
#ifndef ROBOT_APP_DAEMON_VISION_PRIORITY
#define ROBOT_APP_DAEMON_VISION_PRIORITY          6U
#endif

#ifndef ROBOT_APP_DAEMON_IMU_OFFLINE_MS
#define ROBOT_APP_DAEMON_IMU_OFFLINE_MS           10U
#endif
#ifndef ROBOT_APP_DAEMON_IMU_ONLINE_MS
#define ROBOT_APP_DAEMON_IMU_ONLINE_MS            10U
#endif
#ifndef ROBOT_APP_DAEMON_IMU_PRIORITY
#define ROBOT_APP_DAEMON_IMU_PRIORITY             7U
#endif

#ifndef ROBOT_APP_DAEMON_GIMBAL_YAW_OFFLINE_MS
#define ROBOT_APP_DAEMON_GIMBAL_YAW_OFFLINE_MS    2U
#endif
#ifndef ROBOT_APP_DAEMON_GIMBAL_YAW_ONLINE_MS
#define ROBOT_APP_DAEMON_GIMBAL_YAW_ONLINE_MS     3U
#endif
#ifndef ROBOT_APP_DAEMON_GIMBAL_YAW_PRIORITY
#define ROBOT_APP_DAEMON_GIMBAL_YAW_PRIORITY      14U
#endif

#ifndef ROBOT_APP_DAEMON_GIMBAL_PITCH_OFFLINE_MS
#define ROBOT_APP_DAEMON_GIMBAL_PITCH_OFFLINE_MS  2U
#endif
#ifndef ROBOT_APP_DAEMON_GIMBAL_PITCH_ONLINE_MS
#define ROBOT_APP_DAEMON_GIMBAL_PITCH_ONLINE_MS   3U
#endif
#ifndef ROBOT_APP_DAEMON_GIMBAL_PITCH_PRIORITY
#define ROBOT_APP_DAEMON_GIMBAL_PITCH_PRIORITY    13U
#endif

#ifndef ROBOT_APP_DAEMON_SHOOT_OFFLINE_MS
#define ROBOT_APP_DAEMON_SHOOT_OFFLINE_MS         10U
#endif
#ifndef ROBOT_APP_DAEMON_SHOOT_ONLINE_MS
#define ROBOT_APP_DAEMON_SHOOT_ONLINE_MS          10U
#endif
#ifndef ROBOT_APP_DAEMON_SHOOT_PRIORITY
#define ROBOT_APP_DAEMON_SHOOT_PRIORITY           12U
#endif

#ifndef ROBOT_APP_DAEMON_CHASSIS_OFFLINE_MS
#define ROBOT_APP_DAEMON_CHASSIS_OFFLINE_MS       10U
#endif
#ifndef ROBOT_APP_DAEMON_CHASSIS_ONLINE_MS
#define ROBOT_APP_DAEMON_CHASSIS_ONLINE_MS        10U
#endif
#ifndef ROBOT_APP_DAEMON_CHASSIS_PRIORITY
#define ROBOT_APP_DAEMON_CHASSIS_PRIORITY         11U
#endif

#ifndef ROBOT_APP_DAEMON_TFMINI_OFFLINE_MS
#define ROBOT_APP_DAEMON_TFMINI_OFFLINE_MS        100U
#endif
#ifndef ROBOT_APP_DAEMON_TFMINI_ONLINE_MS
#define ROBOT_APP_DAEMON_TFMINI_ONLINE_MS         100U
#endif
#ifndef ROBOT_APP_DAEMON_TFMINI_PRIORITY
#define ROBOT_APP_DAEMON_TFMINI_PRIORITY          3U
#endif

/** @brief Daemon 周期，单位 ms。 */
#define APP_DAEMON_PERIOD_MS                      ROBOT_APP_DAEMON_PERIOD_MS

/** @brief Daemon 线程栈大小，单位 byte。 */
#define APP_DAEMON_STACK_SIZE                     ROBOT_APP_DAEMON_STACK_SIZE

/** @brief Daemon 线程优先级。 */
#define APP_DAEMON_PRIORITY                       ROBOT_APP_DAEMON_PRIORITY

/** @brief Daemon 边沿事件 ring 容量。 */
#define APP_DAEMON_EVENT_MAX                      ROBOT_APP_DAEMON_EVENT_MAX

/** @brief Daemon 启动后首次轮询前等待时间，单位 ms。 */
#define APP_DAEMON_INIT_DELAY_MS                  ROBOT_APP_DAEMON_INIT_DELAY_MS

/** @brief 非 0 表示 TFmini 项参与健康表。 */
#define APP_DAEMON_USE_TFMINI                     ROBOT_APP_DAEMON_USE_TFMINI

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_DAEMON_CONFIG_H */
