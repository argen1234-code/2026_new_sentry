/**
 ******************************************************************************
 * @file    app_magazine_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   弹舱应用层配置。
 *
 * @note    本文件只放弹舱 app 的任务周期和线程参数，具体 PWM 绑定来自 Robot map。
 * @note    弹舱控制命令来自 Apps/app_input.*，不在本模块解析遥控器。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_MAGAZINE_CONFIG_H
#define PRINTK_APPS_MAGAZINE_CONFIG_H

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
/** @brief 弹舱任务栈大小，单位 byte。 */

#ifndef APP_MAGAZINE_STACK_SIZE
#define APP_MAGAZINE_STACK_SIZE ROBOT_APP_MAGAZINE_STACK_SIZE
#endif

/** @brief 弹舱任务优先级。 */
#ifndef APP_MAGAZINE_PRIORITY
#define APP_MAGAZINE_PRIORITY ROBOT_APP_MAGAZINE_PRIORITY
#endif

/** @brief 弹舱任务周期，单位 ms。 */
#ifndef APP_MAGAZINE_PERIOD_MS
#define APP_MAGAZINE_PERIOD_MS ROBOT_APP_MAGAZINE_PERIOD_MS
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_MAGAZINE_CONFIG_H */
