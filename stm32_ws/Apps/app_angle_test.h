/**
 ******************************************************************************
 * @file    app_angle_test.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-20
 * @brief   单电机角度环 PID 台架测试接口。
 *
 * @note    本 App 只在 can1_dmh6215_angle_test target 启用，不接入整车。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_ANGLE_TEST_H
#define PRINTK_APPS_ANGLE_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_ANGLE_TEST_STATE_IDLE                  0U  /**< 未开始。 */
#define APP_ANGLE_TEST_STATE_WAIT_FEEDBACK         1U  /**< 等待反馈。 */
#define APP_ANGLE_TEST_STATE_RUN                   2U  /**< 阶跃运行。 */
#define APP_ANGLE_TEST_STATE_STOP                  3U  /**< 停止序列。 */
#define APP_ANGLE_TEST_STATE_DONE                  4U  /**< 已完成。 */

#define APP_ANGLE_TEST_FAULT_NONE                  0U  /**< 无异常。 */
#define APP_ANGLE_TEST_FAULT_NO_FEEDBACK           1U  /**< 启动无反馈。 */
#define APP_ANGLE_TEST_FAULT_FEEDBACK_LOST         2U  /**< 运行中反馈丢失。 */
#define APP_ANGLE_TEST_FAULT_TIMEOUT               3U  /**< 总超时。 */
#define APP_ANGLE_TEST_FAULT_RANGE                 4U  /**< 位置超出测试范围。 */

/** @brief 当前测试状态。 */
extern volatile uint32_t app_angle_test_state;

/** @brief 当前故障原因。 */
extern volatile uint32_t app_angle_test_fault;

/** @brief 当前持久目标角，rad。 */
extern volatile float app_angle_test_target;

/** @brief 最近一次角度反馈，rad。 */
extern volatile float app_angle_test_actual;

/** @brief 最近一次 wrap 后误差，rad。 */
extern volatile float app_angle_test_err;

/** @brief 最近一次 PID 输出（速度指令），rad/s。 */
extern volatile float app_angle_test_out;

/** @brief 已处理的反馈帧数。 */
extern volatile uint32_t app_angle_test_fb_count;

void app_angle_test_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_ANGLE_TEST_H */
