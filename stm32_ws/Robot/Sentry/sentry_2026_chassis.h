/**
 ******************************************************************************
 * @file    sentry_2026_chassis.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-20
 * @brief   2026 哨兵底盘验证 target。
 *
 * @note    与 sentry_2026 使用同一块 DM-MC02 与同一套 map/param，但只启动
 *          底盘相关 App；用于在接入云台/发射前验证 DM6215 原生速度模式。
 * @note    本 target 启用 ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED，底盘输出改为
 *          速度目标，速度环在电机内部。验证通过后再决定是否合并到整车 target。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_SENTRY_2026_CHASSIS_H
#define PRINTK_ROBOT_SENTRY_2026_CHASSIS_H

#include "robot_id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Target 身份
 * ========================================================================== */

#define ROBOT_TARGET_NAME                         "sentry_2026_chassis"          /**< 当前 target 名称。 */
#define ROBOT_TARGET_ID                           ROBOT_TARGET_SENTRY_2026_CHASSIS /**< 当前 target 编号。 */
#define ROBOT_ROLE                                ROBOT_ROLE_SENTRY                  /**< 当前机器人角色：哨兵。 */
#define ROBOT_MODE                                ROBOT_MODE_DEBUG                   /**< 当前代码模式。 */
#define ROBOT_BOARD                               ROBOT_BOARD_DM02_STM32_H723        /**< 当前控制板。 */
#define ROBOT_BOARD_MOUNT                         ROBOT_BOARD_MOUNT_GIMBAL           /**< 同一块云台侧 DM02。 */
#define ROBOT_CAP_ENABLE                          ROBOT_CAP_STATE_OFF                /**< 验证阶段不启用超级电容。 */
#define ROBOT_PACK_SIZE                           128U                               /**< 通讯默认包容量。 */

/* ==========================================================================
 * App 启动选择
 * --------------------------------------------------------------------------
 * 只启动底盘验证必需的 Observe/Communication/Input/Chassis/Daemon/校准。
 * ========================================================================== */

#define ROBOT_APP_ENABLE_OBSERVE                  0U                                 /**< 验证 target 不启动 Observe。 */
#define ROBOT_APP_ENABLE_COMMUNICATION            1U                                 /**< 启动通信 app + USB 桥接。 */
#define ROBOT_APP_ENABLE_INPUT                    0U                                 /**< 验证 target 不启动 Input。 */
#define ROBOT_APP_ENABLE_GIMBAL                   0U                                 /**< 验证 target 不启动云台。 */
#define ROBOT_APP_ENABLE_CHASSIS                  1U                                 /**< 启动底盘 app。 */
#define ROBOT_APP_ENABLE_SHOOT                    0U                                 /**< 验证 target 不启动发射。 */
#define ROBOT_APP_ENABLE_MAGAZINE                 0U                                 /**< 验证 target 不启动弹舱。 */
#define ROBOT_APP_ENABLE_CALIBRATION              0U                                 /**< 验证 target 不启动校准。 */
#define ROBOT_APP_ENABLE_REFEREE                  0U                                 /**< 验证 target 不启动裁判系统。 */
#define ROBOT_APP_ENABLE_DAEMON                   0U                                 /**< 验证 target 不启动守护 app。 */
#define ROBOT_APP_ENABLE_ALARM                    0U                                 /**< 验证 target 不启动蜂鸣器告警。 */
#define ROBOT_APP_ENABLE_BOARD_LINK               0U                                 /**< 验证 target 不启用板间通信。 */
#define ROBOT_APP_ENABLE_BOARD_TEST               0U                                 /**< 不启动光板测试 app。 */
#define ROBOT_APP_ENABLE_USB_BRIDGE               1U                                 /**< 启用 Jetson <-> STM32 USB CDC 桥接。 */
#define ROBOT_APP_ENABLE_CAN_TEST                 0U                                 /**< 不启动 CAN 台架测试。 */

/* ==========================================================================
 * 机构构型与底盘输出语义
 * ========================================================================== */

#define CHASSIS_TYPE                              ROBOT_CHASSIS_KIND_MECANUM         /**< 底盘构型：麦克纳姆轮。 */
#define GIMBAL_TYPE                               ROBOT_GIMBAL_KIND_DIRECT_YAW_PITCH /**< 占位构型。 */
#define SHOOT_TYPE                                ROBOT_SHOOT_KIND_NONE              /**< 验证 target 不启用发射。 */

/** @brief 底盘驱动改用 DM6215 原生速度模式。 */
#define ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED         1U

/** @brief 四轮全部在线才允许输出；ID3 反馈已用 stop-only 固件确认正常。 */
#define ROBOT_CHASSIS_MIN_ONLINE_MOTORS           4U

/** @brief 底盘验证不等待云台 ready。 */
#define APP_CHASSIS_REQUIRE_GIMBAL_READY          0U

/*
 * 台架注入：无遥控器时直接给前进速度，超时后自动停止。
 * 默认关闭；需要复测时可用 CMake CFLAGS 覆盖，或在本地临时改回 1。
 */
#ifndef APP_CHASSIS_BENCH_TEST
#define APP_CHASSIS_BENCH_TEST                    0U
#endif
#ifndef APP_CHASSIS_BENCH_MS
#define APP_CHASSIS_BENCH_MS                      20000U
#endif
#ifndef APP_CHASSIS_BENCH_VX_MPS
#define APP_CHASSIS_BENCH_VX_MPS                  0.0135F
#endif
#ifndef APP_CHASSIS_BENCH_RAMP_MS
#define APP_CHASSIS_BENCH_RAMP_MS                 0U
#endif

/* ==========================================================================
 * 参数分组
 * ========================================================================== */

#include "sentry_2026_map.h"
#include "sentry_2026_param.h"

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_SENTRY_2026_CHASSIS_H */
