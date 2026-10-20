/**
 ******************************************************************************
 * @file    app_chassis_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Chassis 应用层配置。
 *
 * @note    本文件只放 Chassis app 的任务周期、反馈超时和当前 target 参数展开。
 * @note    底盘机构参数来自 Robot/robot.h，电机硬件绑定来自 Robot/robot_board.h。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_CHASSIS_CONFIG_H
#define PRINTK_APPS_CHASSIS_CONFIG_H

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
/** @brief Chassis 任务栈大小，单位 byte。 */

#ifndef APP_CHASSIS_STACK_SIZE
#define APP_CHASSIS_STACK_SIZE 2048U
#endif

/** @brief Chassis 任务优先级。 */
#ifndef APP_CHASSIS_PRIORITY
#define APP_CHASSIS_PRIORITY osPriorityAboveNormal
#endif

/** @brief Chassis 控制周期，单位 ms。 */
#ifndef APP_CHASSIS_PERIOD_MS
#define APP_CHASSIS_PERIOD_MS ROBOT_CHASSIS_CONTROL_TIME_MS
#endif

/** @brief 底盘电机反馈超过该时间未更新则判定离线，单位 ms。 */
#ifndef APP_CHASSIS_MOTOR_LOST_MS
#define APP_CHASSIS_MOTOR_LOST_MS 100U
#endif

/** @brief 底盘任务心跳超过该时间未更新则判定任务卡死，单位 ms。 */
#ifndef APP_CHASSIS_CMD_STALE_MS
#define APP_CHASSIS_CMD_STALE_MS 20U
#endif

/** @brief 非 0 表示底盘控制必须等待云台 ready；独立底盘 target 置 0。 */
#ifndef APP_CHASSIS_REQUIRE_GIMBAL_READY
#define APP_CHASSIS_REQUIRE_GIMBAL_READY 1U
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_CHASSIS_CONFIG_H */
