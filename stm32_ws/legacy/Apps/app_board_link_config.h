/**
 ******************************************************************************
 * @file    app_board_link_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   BoardLink 应用层配置。
 *
 * @note    本文件只描述板间状态发送周期与任务参数；实际 UART 绑定由
 *          Component/Communication 配置决定。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_BOARD_LINK_CONFIG_H
#define PRINTK_APPS_BOARD_LINK_CONFIG_H

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
/** @brief BoardLink 任务栈大小，单位 byte。 */

#ifndef APP_BOARD_LINK_STACK_SIZE
#define APP_BOARD_LINK_STACK_SIZE ROBOT_APP_BOARD_LINK_STACK_SIZE
#endif

/** @brief BoardLink 任务优先级。 */
#ifndef APP_BOARD_LINK_PRIORITY
#define APP_BOARD_LINK_PRIORITY ROBOT_APP_BOARD_LINK_PRIORITY
#endif

/** @brief BoardLink 任务周期，单位 ms。 */
#ifndef APP_BOARD_LINK_PERIOD_MS
#define APP_BOARD_LINK_PERIOD_MS ROBOT_APP_BOARD_LINK_PERIOD_MS
#endif

/** @brief 板间云台状态发送周期，单位 ms。 */
#ifndef APP_BOARD_LINK_STATE_PERIOD_MS
#define APP_BOARD_LINK_STATE_PERIOD_MS 5U
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_BOARD_LINK_CONFIG_H */
