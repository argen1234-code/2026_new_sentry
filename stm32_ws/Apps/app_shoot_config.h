/**
 ******************************************************************************
 * @file    app_shoot_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Shoot 应用层配置。
 *
 * @note    本文件只放 Shoot app 的任务周期、反馈超时和当前 target 参数展开。
 * @note    发射机构参数来自 Robot/robot.h，电机硬件绑定来自 Robot/robot_board.h。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_SHOOT_CONFIG_H
#define PRINTK_APPS_SHOOT_CONFIG_H

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
/** @brief Shoot 任务栈大小，单位 byte。 */

#ifndef APP_SHOOT_STACK_SIZE
#define APP_SHOOT_STACK_SIZE 2048U
#endif

/** @brief Shoot 任务优先级。 */
#ifndef APP_SHOOT_PRIORITY
#define APP_SHOOT_PRIORITY osPriorityHigh
#endif

/** @brief Shoot 控制周期，单位 ms。 */
#ifndef APP_SHOOT_PERIOD_MS
#define APP_SHOOT_PERIOD_MS ROBOT_SHOOT_CONTROL_TIME_MS
#endif

/** @brief DJI 发射电机反馈超过该时间未更新则判定离线，单位 ms。 */
#ifndef APP_SHOOT_MOTOR_LOST_MS
#define APP_SHOOT_MOTOR_LOST_MS 100U
#endif

/** @brief 摩擦轮温度安全上限，单位摄氏度。 */
#ifndef APP_SHOOT_FRIC_TEMP_LIMIT
#define APP_SHOOT_FRIC_TEMP_LIMIT ROBOT_SHOOT_FRIC_TEMP_LIMIT
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_SHOOT_CONFIG_H */
