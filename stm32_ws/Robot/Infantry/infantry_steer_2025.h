/**
 ******************************************************************************
 * @file    infantry_steer_2025.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   2025 舵轮步兵 target 聚合配置。
 *
 * @note    target 唯一标识符为“步兵 + 舵轮底盘 + 2025”。
 * @note    本文件只聚合身份、机械构型、硬件 map 和运行时参数。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_INFANTRY_STEER_2025_H
#define PRINTK_ROBOT_INFANTRY_STEER_2025_H

#include "robot_id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Target 身份
 * --------------------------------------------------------------------------
 * 明确当前固件服务的具体车辆，不使用兵种或构型作为模糊目标。
 * ========================================================================== */

#define ROBOT_TARGET_NAME                         "infantry_steer_2025"              /**< 当前 target 名称。 */
#define ROBOT_TARGET_ID                           ROBOT_TARGET_INFANTRY_STEER_2025   /**< 当前 target 编号。 */
#define ROBOT_ROLE                                ROBOT_ROLE_INFANTRY                /**< 当前机器人角色。 */
#define ROBOT_MODE                                ROBOT_MODE_DEBUG                   /**< 当前代码模式。 */
#define ROBOT_BOARD                               ROBOT_BOARD_DM02_STM32_H723        /**< 当前控制板。 */
#define ROBOT_BOARD_MOUNT                         ROBOT_BOARD_MOUNT_GIMBAL           /**< 当前控制板安装位置。 */
#define ROBOT_CAP_ENABLE                          ROBOT_CAP_STATE_OFF                /**< 超级电容启用状态。 */
#define ROBOT_PACK_SIZE                           128U                               /**< 调试和通讯默认包容量。 */

/* ==========================================================================
 * App 启动选择
 * --------------------------------------------------------------------------
 * 描述当前控制板实际承载的应用。当前 DM02 安装在云台侧，只运行云台、
 * 发射、观测、输入和通信；底盘 app 留给底盘侧控制板。
 * ========================================================================== */

#define ROBOT_APP_ENABLE_OBSERVE                  1U                                 /**< 启动观测 app。 */
#define ROBOT_APP_ENABLE_COMMUNICATION            1U                                 /**< 启动通信 app。 */
#define ROBOT_APP_ENABLE_INPUT                    1U                                 /**< 启动输入 app。 */
#define ROBOT_APP_ENABLE_GIMBAL                   1U                                 /**< 启动云台 app。 */
#define ROBOT_APP_ENABLE_CHASSIS                  0U                                 /**< 当前云台板不启动底盘 app。 */
#define ROBOT_APP_ENABLE_SHOOT                    1U                                 /**< 启动发射 app。 */
#define ROBOT_APP_ENABLE_MAGAZINE                 1U                                 /**< 启动弹舱 app。 */
#define ROBOT_APP_ENABLE_CALIBRATION              1U                                 /**< 启动校准 app。 */
#define ROBOT_APP_ENABLE_REFEREE                  1U                                 /**< 启动裁判系统 app。 */
#define ROBOT_APP_ENABLE_DAEMON                   1U                                 /**< 启动守护 app。 */
#define ROBOT_APP_ENABLE_ALARM                    1U                                 /**< 启动蜂鸣器告警 app。 */
#define ROBOT_APP_ENABLE_BOARD_LINK               0U                                 /**< 当前云台板暂不启用板间通信。 */
#define ROBOT_APP_ENABLE_BOARD_TEST               0U                                 /**< 正常车辆 target 不启动光板测试 app。 */
#define ROBOT_APP_ENABLE_USB_BRIDGE               0U                                 /**< 步兵 target 暂不启用 USB 桥接。 */
#define ROBOT_APP_ENABLE_CAN_TEST                 0U                                 /**< 步兵 target 不启动 CAN 台架测试。 */

/* ==========================================================================
 * 机构构型
 * --------------------------------------------------------------------------
 * 构型选择是编译期事实，不做运行时机械构型切换。
 * ========================================================================== */

#define CHASSIS_TYPE                              ROBOT_CHASSIS_KIND_STEERING        /**< 底盘构型：舵轮。 */
#define GIMBAL_TYPE                               ROBOT_GIMBAL_KIND_DIRECT_YAW_PITCH /**< 云台构型：yaw/pitch 直驱。 */
#define SHOOT_TYPE                                ROBOT_SHOOT_KIND_MONO              /**< 发射构型：单组摩擦轮。 */

/* ==========================================================================
 * 参数分组
 * --------------------------------------------------------------------------
 * 硬件接线、设备默认策略和运行时控制参数分文件维护。
 * ========================================================================== */

#include "infantry_steer_2025_map.h"
#include "infantry_steer_2025_param.h"

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_INFANTRY_STEER_2025_H */
