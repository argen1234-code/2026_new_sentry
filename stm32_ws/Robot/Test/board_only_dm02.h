/**
 ******************************************************************************
 * @file    board_only_dm02.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   DM02 光板测试 target 聚合配置。
 *
 * @note    本 target 只服务于未接电机的裸板验证，默认只启动 BoardTest app。
 * @note    本文件仍完整给出构型和参数入口，保证所有源码参与编译时宏均可展开。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_TEST_BOARD_ONLY_DM02_H
#define PRINTK_ROBOT_TEST_BOARD_ONLY_DM02_H

#include "robot_id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Target 身份
 * --------------------------------------------------------------------------
 * 光板测试固件不是比赛机器人运行链路，只验证 DM02 板载基础资源。
 * ========================================================================== */

#define ROBOT_TARGET_NAME                         "board_only_dm02"                  /**< 当前 target 名称。 */
#define ROBOT_TARGET_ID                           ROBOT_TARGET_BOARD_ONLY_DM02       /**< 当前 target 编号。 */
#define ROBOT_ROLE                                ROBOT_ROLE_TEST                    /**< 当前 target 角色。 */
#define ROBOT_MODE                                ROBOT_MODE_DEBUG                   /**< 当前代码模式。 */
#define ROBOT_BOARD                               ROBOT_BOARD_DM02_STM32_H723        /**< 当前控制板。 */
#define ROBOT_BOARD_MOUNT                         ROBOT_BOARD_MOUNT_GIMBAL           /**< 测试固件不区分真实安装位，沿用云台侧默认。 */
#define ROBOT_CAP_ENABLE                          ROBOT_CAP_STATE_OFF                /**< 光板测试不启用超级电容。 */
#define ROBOT_PACK_SIZE                           128U                               /**< 调试和通讯默认包容量。 */

/* ==========================================================================
 * App 启动选择
 * --------------------------------------------------------------------------
 * 只启动光板测试任务，避免不接电机时启动控制链路、CAN 发送或离线告警。
 * ========================================================================== */

#define ROBOT_APP_ENABLE_OBSERVE                  0U                                 /**< 光板测试不启动 Observe。 */
#define ROBOT_APP_ENABLE_COMMUNICATION            0U                                 /**< 光板测试不启动正式 Communication app。 */
#define ROBOT_APP_ENABLE_INPUT                    0U                                 /**< 光板测试不启动 Input。 */
#define ROBOT_APP_ENABLE_GIMBAL                   0U                                 /**< 光板测试不启动 Gimbal。 */
#define ROBOT_APP_ENABLE_CHASSIS                  0U                                 /**< 光板测试不启动 Chassis。 */
#define ROBOT_APP_ENABLE_SHOOT                    0U                                 /**< 光板测试不启动 Shoot。 */
#define ROBOT_APP_ENABLE_MAGAZINE                 0U                                 /**< 光板测试不启动 Magazine。 */
#define ROBOT_APP_ENABLE_CALIBRATION              0U                                 /**< 光板测试不启动 Calibration。 */
#define ROBOT_APP_ENABLE_REFEREE                  0U                                 /**< 光板测试不启动 Referee。 */
#define ROBOT_APP_ENABLE_DAEMON                   0U                                 /**< 光板测试不启动 Daemon。 */
#define ROBOT_APP_ENABLE_ALARM                    0U                                 /**< 光板测试不启动 Alarm。 */
#define ROBOT_APP_ENABLE_BOARD_LINK               0U                                 /**< 光板测试不启动 BoardLink。 */
#define ROBOT_APP_ENABLE_BOARD_TEST               1U                                 /**< 启动光板测试 app。 */
#define ROBOT_APP_ENABLE_USB_BRIDGE               0U                                 /**< 光板测试不启用 USB 桥接。 */
#define ROBOT_APP_ENABLE_CAN_TEST                 0U                                 /**< 光板测试不启动 CAN 台架测试。 */

/* ==========================================================================
 * 机构构型占位
 * --------------------------------------------------------------------------
 * 构型宏仍需给出，保证 Controller 源文件参与编译时能够走确定的宏路径。
 * ========================================================================== */

#define CHASSIS_TYPE                              ROBOT_CHASSIS_KIND_STEERING        /**< 底盘构型占位：舵轮。 */
#define GIMBAL_TYPE                               ROBOT_GIMBAL_KIND_DIRECT_YAW_PITCH /**< 云台构型占位：yaw/pitch 直驱。 */
#define SHOOT_TYPE                                ROBOT_SHOOT_KIND_NONE              /**< 光板测试不启用发射机构。 */

/* ==========================================================================
 * 参数分组
 * --------------------------------------------------------------------------
 * map 描述 DM02 板载资源接线；param 保存测试任务参数和禁用链路所需默认展开值。
 * ========================================================================== */

#include "board_only_dm02_map.h"
#include "board_only_dm02_param.h"

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_TEST_BOARD_ONLY_DM02_H */
