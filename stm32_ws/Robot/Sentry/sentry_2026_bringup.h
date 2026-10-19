/**
 ******************************************************************************
 * @file    sentry_2026_bringup.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-19
 * @brief   2026 哨兵台架 bring-up target。
 *
 * @note    本 target 只启动 Communication（含 USB CDC 桥接），不启动 IMU、
 *          云台、发射、裁判等依赖外设的 App，用于台架验证 USB 链路。
 * @note    硬件 map/param 复用 sentry_2026，避免台架与整车配置分叉。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_SENTRY_2026_BRINGUP_H
#define PRINTK_ROBOT_SENTRY_2026_BRINGUP_H

#include "robot_id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Target 身份
 * ========================================================================== */

#define ROBOT_TARGET_NAME                         "sentry_2026_bringup"          /**< 当前 target 名称。 */
#define ROBOT_TARGET_ID                           ROBOT_TARGET_SENTRY_2026_BRINGUP /**< 当前 target 编号。 */
#define ROBOT_ROLE                                ROBOT_ROLE_SENTRY                  /**< 当前机器人角色：哨兵。 */
#define ROBOT_MODE                                ROBOT_MODE_DEBUG                   /**< 台架调试模式。 */
#define ROBOT_BOARD                               ROBOT_BOARD_DM02_STM32_H723        /**< 当前控制板。 */
#define ROBOT_BOARD_MOUNT                         ROBOT_BOARD_MOUNT_GIMBAL           /**< 控制板安装位置。 */
#define ROBOT_CAP_ENABLE                          ROBOT_CAP_STATE_OFF                /**< 台架不启用超级电容。 */
#define ROBOT_PACK_SIZE                           128U                               /**< 通讯默认包容量。 */

/* ==========================================================================
 * App 启动选择
 * --------------------------------------------------------------------------
 * 台架 bring-up 只保留 Communication；USB 桥接由 Communication app 内部启用。
 * ========================================================================== */

#define ROBOT_APP_ENABLE_OBSERVE                  0U                                 /**< 台架不启动 Observe。 */
#define ROBOT_APP_ENABLE_COMMUNICATION            1U                                 /**< 启动 Communication + USB 桥接。 */
#define ROBOT_APP_ENABLE_INPUT                    0U                                 /**< 台架不启动 Input。 */
#define ROBOT_APP_ENABLE_GIMBAL                   0U                                 /**< 台架不启动 Gimbal。 */
#define ROBOT_APP_ENABLE_CHASSIS                  0U                                 /**< 台架不启动 Chassis。 */
#define ROBOT_APP_ENABLE_SHOOT                    0U                                 /**< 台架不启动 Shoot。 */
#define ROBOT_APP_ENABLE_MAGAZINE                 0U                                 /**< 台架不启动 Magazine。 */
#define ROBOT_APP_ENABLE_CALIBRATION              0U                                 /**< 台架不启动 Calibration。 */
#define ROBOT_APP_ENABLE_REFEREE                  0U                                 /**< 台架不启动 Referee。 */
#define ROBOT_APP_ENABLE_DAEMON                   0U                                 /**< 台架不启动 Daemon。 */
#define ROBOT_APP_ENABLE_ALARM                    0U                                 /**< 台架不启动 Alarm。 */
#define ROBOT_APP_ENABLE_BOARD_LINK               0U                                 /**< 台架不启动 BoardLink。 */
#define ROBOT_APP_ENABLE_BOARD_TEST               0U                                 /**< 台架不启动 BoardTest。 */
#define ROBOT_APP_ENABLE_USB_BRIDGE               1U                                 /**< 启用 Jetson <-> STM32 USB CDC 桥接。 */
#define ROBOT_APP_ENABLE_CAN_TEST                 0U                                 /**< 台架 bring-up 不启动 CAN 测试。 */

/* ==========================================================================
 * 机构构型占位
 * --------------------------------------------------------------------------
 * 台架不启动控制 App，但仍需给出确定构型，保证 Controller 源码可编译。
 * ========================================================================== */

#define CHASSIS_TYPE                              ROBOT_CHASSIS_KIND_MECANUM         /**< 底盘构型：麦克纳姆轮。 */
#define GIMBAL_TYPE                               ROBOT_GIMBAL_KIND_DIRECT_YAW_PITCH /**< 云台构型占位。 */
#define SHOOT_TYPE                                ROBOT_SHOOT_KIND_MONO              /**< 发射构型占位。 */

/* ==========================================================================
 * 参数分组
 * ========================================================================== */

#include "sentry_2026_map.h"
#include "sentry_2026_param.h"

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_SENTRY_2026_BRINGUP_H */
