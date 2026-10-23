/**
 ******************************************************************************
 * @file    app_board_test_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   光板测试应用层配置映射。
 *
 * @note    本文件只把 Robot target 中的板载测试参数收敛为 App 内部短宏。
 * @note    测试任务不承担机器人运行链路职责，也不启动电机、遥控器或裁判系统。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_BOARD_TEST_CONFIG_H
#define PRINTK_APPS_BOARD_TEST_CONFIG_H

#include "cmsis_os.h"
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 任务参数与 target 展开
 * --------------------------------------------------------------------------
 * 配置头只描述周期、栈大小、优先级和板载测试参数，不放运行时测试逻辑。
 * ========================================================================== */

#ifndef ROBOT_APP_BOARD_TEST_PERIOD_MS
#define ROBOT_APP_BOARD_TEST_PERIOD_MS            10U
#endif

#ifndef ROBOT_APP_BOARD_TEST_STACK_SIZE
#define ROBOT_APP_BOARD_TEST_STACK_SIZE           2048U
#endif

#ifndef ROBOT_APP_BOARD_TEST_PRIORITY
#define ROBOT_APP_BOARD_TEST_PRIORITY             osPriorityNormal
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_PERIOD_MS
#define ROBOT_APP_BOARD_TEST_IMU_PERIOD_MS        10U
#endif

#ifndef ROBOT_APP_BOARD_TEST_LOG_PERIOD_MS
#define ROBOT_APP_BOARD_TEST_LOG_PERIOD_MS        500U
#endif

#ifndef ROBOT_APP_BOARD_TEST_TEXT_LOG_ENABLE
#define ROBOT_APP_BOARD_TEST_TEXT_LOG_ENABLE      0U
#endif

#ifndef ROBOT_APP_BOARD_TEST_VOFA_ENABLE
#define ROBOT_APP_BOARD_TEST_VOFA_ENABLE          1U
#endif

#ifndef ROBOT_APP_BOARD_TEST_VOFA_PERIOD_MS
#define ROBOT_APP_BOARD_TEST_VOFA_PERIOD_MS       20U
#endif

#ifndef ROBOT_APP_BOARD_TEST_DTS_PERIOD_MS
#define ROBOT_APP_BOARD_TEST_DTS_PERIOD_MS        200U
#endif

#ifndef ROBOT_APP_BOARD_TEST_LED_PERIOD_MS
#define ROBOT_APP_BOARD_TEST_LED_PERIOD_MS        200U
#endif

#ifndef ROBOT_APP_BOARD_TEST_FLASH_ADDR
#define ROBOT_APP_BOARD_TEST_FLASH_ADDR           (ROBOT_FLASH_DEVICE_SIZE - (2UL * ROBOT_FLASH_DEVICE_ERASE_SIZE))
#endif

#ifndef ROBOT_APP_BOARD_TEST_FLASH_LEN
#define ROBOT_APP_BOARD_TEST_FLASH_LEN            ROBOT_FLASH_DEVICE_ERASE_SIZE
#endif

#ifndef ROBOT_APP_BOARD_TEST_FLASH_PATTERN_LEN
#define ROBOT_APP_BOARD_TEST_FLASH_PATTERN_LEN    64U
#endif

#ifndef ROBOT_APP_BOARD_TEST_WS2812_COUNT
#define ROBOT_APP_BOARD_TEST_WS2812_COUNT         ROBOT_WS2812_LED_MAX
#endif

#ifndef ROBOT_APP_BOARD_TEST_BUZZER_FREQ_HZ
#define ROBOT_APP_BOARD_TEST_BUZZER_FREQ_HZ       1800U
#endif

#ifndef ROBOT_APP_BOARD_TEST_BUZZER_DUTY
#define ROBOT_APP_BOARD_TEST_BUZZER_DUTY          500U
#endif

#ifndef ROBOT_APP_BOARD_TEST_BEEP_ON_MS
#define ROBOT_APP_BOARD_TEST_BEEP_ON_MS           100U
#endif

#ifndef ROBOT_APP_BOARD_TEST_BEEP_OFF_MS
#define ROBOT_APP_BOARD_TEST_BEEP_OFF_MS          100U
#endif

#ifndef ROBOT_APP_BOARD_TEST_FAIL_GAP_MS
#define ROBOT_APP_BOARD_TEST_FAIL_GAP_MS          2000U
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_SOLVE_ENABLE
#define ROBOT_APP_BOARD_TEST_IMU_SOLVE_ENABLE     1U
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_READY_TIME
#define ROBOT_APP_BOARD_TEST_IMU_READY_TIME       3.0F
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_CALI_SAMPLES
#define ROBOT_APP_BOARD_TEST_IMU_CALI_SAMPLES     300U
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_CALI_GYRO_TH
#define ROBOT_APP_BOARD_TEST_IMU_CALI_GYRO_TH     0.08F
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_CALI_ACCEL_TH
#define ROBOT_APP_BOARD_TEST_IMU_CALI_ACCEL_TH    0.80F
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_STABLE_GYRO_TH
#define ROBOT_APP_BOARD_TEST_IMU_STABLE_GYRO_TH   0.05F
#endif

#ifndef ROBOT_APP_BOARD_TEST_IMU_STABLE_ACCEL_TH
#define ROBOT_APP_BOARD_TEST_IMU_STABLE_ACCEL_TH  0.80F
#endif

/** @brief BoardTest 基础周期，单位 ms。 */
#define APP_BOARD_TEST_PERIOD_MS                  ROBOT_APP_BOARD_TEST_PERIOD_MS

/** @brief BoardTest 线程栈大小，单位 byte。 */
#define APP_BOARD_TEST_STACK_SIZE                 ROBOT_APP_BOARD_TEST_STACK_SIZE

/** @brief BoardTest 线程优先级。 */
#define APP_BOARD_TEST_PRIORITY                   ROBOT_APP_BOARD_TEST_PRIORITY

/** @brief IMU 周期采样间隔，单位 ms。 */
#define APP_BOARD_TEST_IMU_PERIOD_MS              ROBOT_APP_BOARD_TEST_IMU_PERIOD_MS

/** @brief CDC 文本日志输出间隔，单位 ms。 */
#define APP_BOARD_TEST_LOG_PERIOD_MS              ROBOT_APP_BOARD_TEST_LOG_PERIOD_MS

/** @brief 非 0 表示输出 FireWater 文本日志。 */
#define APP_BOARD_TEST_TEXT_LOG_ENABLE            ROBOT_APP_BOARD_TEST_TEXT_LOG_ENABLE

/** @brief 非 0 表示输出 VOFA JustFloat 波形。 */
#define APP_BOARD_TEST_VOFA_ENABLE                ROBOT_APP_BOARD_TEST_VOFA_ENABLE

/** @brief VOFA JustFloat 输出周期，单位 ms。 */
#define APP_BOARD_TEST_VOFA_PERIOD_MS             ROBOT_APP_BOARD_TEST_VOFA_PERIOD_MS

/** @brief MCU DTS 温度采样周期，单位 ms。 */
#define APP_BOARD_TEST_DTS_PERIOD_MS              ROBOT_APP_BOARD_TEST_DTS_PERIOD_MS

/** @brief WS2812 状态刷新间隔，单位 ms。 */
#define APP_BOARD_TEST_LED_PERIOD_MS              ROBOT_APP_BOARD_TEST_LED_PERIOD_MS

/** @brief Flash 专用测试扇区起始地址。 */
#define APP_BOARD_TEST_FLASH_ADDR                 ROBOT_APP_BOARD_TEST_FLASH_ADDR

/** @brief Flash 专用测试擦写长度，单位 byte。 */
#define APP_BOARD_TEST_FLASH_LEN                  ROBOT_APP_BOARD_TEST_FLASH_LEN

/** @brief Flash 写入校验模式长度，单位 byte。 */
#define APP_BOARD_TEST_FLASH_PATTERN_LEN          ROBOT_APP_BOARD_TEST_FLASH_PATTERN_LEN

/** @brief BoardTest 使用的 WS2812 灯珠数量。 */
#define APP_BOARD_TEST_WS2812_COUNT               ROBOT_APP_BOARD_TEST_WS2812_COUNT

/** @brief BoardTest 蜂鸣器提示频率，单位 Hz。 */
#define APP_BOARD_TEST_BUZZER_FREQ_HZ             ROBOT_APP_BOARD_TEST_BUZZER_FREQ_HZ

/** @brief BoardTest 蜂鸣器提示占空比，千分比。 */
#define APP_BOARD_TEST_BUZZER_DUTY                ROBOT_APP_BOARD_TEST_BUZZER_DUTY

/** @brief 单次提示鸣叫时长，单位 ms。 */
#define APP_BOARD_TEST_BEEP_ON_MS                 ROBOT_APP_BOARD_TEST_BEEP_ON_MS

/** @brief 多次提示鸣叫间隔，单位 ms。 */
#define APP_BOARD_TEST_BEEP_OFF_MS                ROBOT_APP_BOARD_TEST_BEEP_OFF_MS

/** @brief 失败提示重复间隔，单位 ms。 */
#define APP_BOARD_TEST_FAIL_GAP_MS                ROBOT_APP_BOARD_TEST_FAIL_GAP_MS

/** @brief 非 0 表示启用 IMU 姿态解算。 */
#define APP_BOARD_TEST_IMU_SOLVE_ENABLE           ROBOT_APP_BOARD_TEST_IMU_SOLVE_ENABLE

/** @brief 姿态解算 ready 最短运行时间，单位 s。 */
#define APP_BOARD_TEST_IMU_READY_TIME             ROBOT_APP_BOARD_TEST_IMU_READY_TIME

/** @brief 姿态解算启动零偏采样次数。 */
#define APP_BOARD_TEST_IMU_CALI_SAMPLES           ROBOT_APP_BOARD_TEST_IMU_CALI_SAMPLES

/** @brief 零偏采样静止陀螺阈值，单位 rad/s。 */
#define APP_BOARD_TEST_IMU_CALI_GYRO_TH           ROBOT_APP_BOARD_TEST_IMU_CALI_GYRO_TH

/** @brief 零偏采样重力模长误差阈值，单位 m/s^2。 */
#define APP_BOARD_TEST_IMU_CALI_ACCEL_TH          ROBOT_APP_BOARD_TEST_IMU_CALI_ACCEL_TH

/** @brief 静止状态陀螺阈值，单位 rad/s。 */
#define APP_BOARD_TEST_IMU_STABLE_GYRO_TH         ROBOT_APP_BOARD_TEST_IMU_STABLE_GYRO_TH

/** @brief 静止状态加速度模长误差阈值，单位 m/s^2。 */
#define APP_BOARD_TEST_IMU_STABLE_ACCEL_TH        ROBOT_APP_BOARD_TEST_IMU_STABLE_ACCEL_TH

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_BOARD_TEST_CONFIG_H */
