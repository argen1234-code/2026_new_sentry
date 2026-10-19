/**
 ******************************************************************************
 * @file    robot.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   机器人 target 选择入口。
 *
 * @note    本文件只负责选择当前具体车辆 target，不承载公共 profile 或继承关系。
 * @note    新增车辆时按“兵种 + 底盘构型 + 年份”添加独立配置文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_H
#define PRINTK_ROBOT_H

#include <stdint.h>

#include "robot_id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Target 选择
 * --------------------------------------------------------------------------
 * 默认选中 DM02 光板测试 target；切回整车固件时在这里或工程宏中显式覆盖。
 * ========================================================================== */

#ifndef ROBOT_TARGET
#define ROBOT_TARGET                              ROBOT_TARGET_BOARD_ONLY_DM02 /**< 当前默认机器人 target。 */
#endif

#if ROBOT_TARGET == ROBOT_TARGET_INFANTRY_STEER_2025
#include "infantry_steer_2025.h"
#elif ROBOT_TARGET == ROBOT_TARGET_SENTRY_2026
#include "sentry_2026.h"
#elif ROBOT_TARGET == ROBOT_TARGET_SENTRY_2026_BRINGUP
#include "sentry_2026_bringup.h"
#elif ROBOT_TARGET == ROBOT_TARGET_BOARD_ONLY_DM02
#include "board_only_dm02.h"
#elif ROBOT_TARGET == ROBOT_TARGET_CAN1_DMH6215_TEST
#include "can1_dmh6215_test.h"
#else
#error "Unknown ROBOT_TARGET. Please add a Robot target and select it in Robot/robot.h."
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_H */
