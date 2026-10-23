/**
 ******************************************************************************
 * @file    can1_dmh6215_angle_test.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-20
 * @brief   CAN1 单电机 DMH6215 角度环 PID 台架测试 target。
 *
 * @note    只启动 Communication（含 USB 桥接）和角度 PID 测试 App。
 * @note    电机需已保存为速度模式（CTRL_MODE=3）；测试输出为速度指令。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_TEST_CAN1_DMH6215_ANGLE_TEST_H
#define PRINTK_ROBOT_TEST_CAN1_DMH6215_ANGLE_TEST_H

#include "robot_id.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_TARGET_NAME                         "can1_dmh6215_angle_test"      /**< 当前 target 名称。 */
#define ROBOT_TARGET_ID                           ROBOT_TARGET_CAN1_DMH6215_ANGLE_TEST /**< 当前 target 编号。 */
#define ROBOT_ROLE                                ROBOT_ROLE_TEST                    /**< 台架测试角色。 */
#define ROBOT_MODE                                ROBOT_MODE_DEBUG                   /**< 台架调试模式。 */
#define ROBOT_BOARD                               ROBOT_BOARD_DM02_STM32_H723        /**< 当前控制板。 */
#define ROBOT_BOARD_MOUNT                         ROBOT_BOARD_MOUNT_GIMBAL           /**< 控制板安装位置。 */
#define ROBOT_CAP_ENABLE                          ROBOT_CAP_STATE_OFF                /**< 台架不启用超级电容。 */
#define ROBOT_PACK_SIZE                           128U                               /**< 通讯默认包容量。 */

#define ROBOT_APP_ENABLE_OBSERVE                  0U                                 /**< 不启动 Observe。 */
#define ROBOT_APP_ENABLE_COMMUNICATION            1U                                 /**< 启动 Communication + USB 桥接。 */
#define ROBOT_APP_ENABLE_INPUT                    0U                                 /**< 不启动 Input。 */
#define ROBOT_APP_ENABLE_GIMBAL                   0U                                 /**< 不启动 Gimbal。 */
#define ROBOT_APP_ENABLE_CHASSIS                  0U                                 /**< 不启动 Chassis。 */
#define ROBOT_APP_ENABLE_SHOOT                    0U                                 /**< 不启动 Shoot。 */
#define ROBOT_APP_ENABLE_MAGAZINE                 0U                                 /**< 不启动 Magazine。 */
#define ROBOT_APP_ENABLE_CALIBRATION              0U                                 /**< 不启动 Calibration。 */
#define ROBOT_APP_ENABLE_REFEREE                  0U                                 /**< 不启动 Referee。 */
#define ROBOT_APP_ENABLE_DAEMON                   0U                                 /**< 不启动 Daemon。 */
#define ROBOT_APP_ENABLE_ALARM                    0U                                 /**< 不启动 Alarm。 */
#define ROBOT_APP_ENABLE_BOARD_LINK               0U                                 /**< 不启动 BoardLink。 */
#define ROBOT_APP_ENABLE_BOARD_TEST               0U                                 /**< 不启动 BoardTest。 */
#define ROBOT_APP_ENABLE_USB_BRIDGE               1U                                 /**< 启用 USB 桥接。 */
#define ROBOT_APP_ENABLE_CAN_TEST                 0U                                 /**< 不启动 CAN 扫描测试。 */
#define ROBOT_APP_ENABLE_ANGLE_TEST               1U                                 /**< 启用角度环 PID 测试。 */

#define CHASSIS_TYPE                              ROBOT_CHASSIS_KIND_MECANUM         /**< 占位构型：麦克纳姆轮。 */
#define GIMBAL_TYPE                               ROBOT_GIMBAL_KIND_DIRECT_YAW_PITCH /**< 占位构型。 */
#define SHOOT_TYPE                                ROBOT_SHOOT_KIND_NONE              /**< 台架不启用发射。 */

#include "sentry_2026_map.h"
#include "sentry_2026_param.h"

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_TEST_CAN1_DMH6215_ANGLE_TEST_H */
