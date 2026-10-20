/**
 ******************************************************************************
 * @file    app_can_test.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-19
 * @brief   CAN1 DMH6215 台架测试接口。
 *
 * @note    只扫描 FDCAN1 上的达妙 MIT 反馈，不启动其它控制 App。
 * @note    测试状态通过全局变量暴露，便于 GDB 直接读取。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_CAN_TEST_H
#define PRINTK_APPS_CAN_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 有效接收帧数。 */
extern volatile uint32_t app_can_test_rx_count;

/** @brief 发送帧数。 */
extern volatile uint32_t app_can_test_tx_count;

/** @brief 最近一帧标准 ID。 */
extern volatile uint32_t app_can_test_last_id;

/** @brief 最近一帧数据。 */
extern volatile uint8_t app_can_test_last_data[8];

/** @brief 已出现的标准 ID 位图（2048 bit）。 */
extern volatile uint8_t app_can_test_seen[256];

/** @brief 当前阶段：0 扫描，1 驱动，2 停止。 */
extern volatile uint32_t app_can_test_phase;

/** @brief 驱动阶段使用的电机 ID。 */
extern volatile uint32_t app_can_test_drive_id;

/** @brief 扫描阶段自动识别到的第一个电机 ID（0 表示未识别）。 */
extern volatile uint32_t app_can_test_detected_id;

/** @brief 已响应的电机 ID 位图，bit1..bit8 对应 ID 1..8。 */
extern volatile uint16_t app_can_test_motor_mask;

/** @brief 最近一次位置指令，单位 rad。 */
extern volatile float app_can_test_cmd_pos;

/** @brief 驱动阶段的位置中心（取自首次反馈），单位 rad。 */
extern volatile float app_can_test_center_pos;

/** @brief 最近一次速度指令，单位 rad/s。 */
extern volatile float app_can_test_cmd_vel;

/** @brief 最近一次发送的 8 字节数据，便于 GDB 检查帧内容。 */
extern volatile uint8_t app_can_test_last_tx[8];

/** @brief FDCAN 发送错误计数（TEC）。 */
extern volatile uint32_t app_can_test_tec;

/** @brief FDCAN 接收错误计数（REC）。 */
extern volatile uint32_t app_can_test_rec;

/** @brief FDCAN 协议状态寄存器。 */
extern volatile uint32_t app_can_test_psr;

/* ==========================================================================
 * 保守驱动测试状态
 * --------------------------------------------------------------------------
 * 这些变量只在 APP_CAN_TEST_CONSERVATIVE=1 时由状态机更新，便于 GDB 读取。
 * ========================================================================== */

#define APP_CAN_TEST_SAFE_STATE_IDLE               0U  /**< 未开始。 */
#define APP_CAN_TEST_SAFE_STATE_WAIT_FEEDBACK      1U  /**< 等待目标电机首帧反馈。 */
#define APP_CAN_TEST_SAFE_STATE_SETTLE             2U  /**< 零力矩稳定阶段。 */
#define APP_CAN_TEST_SAFE_STATE_RUN                3U  /**< 低速斜坡/保持阶段。 */
#define APP_CAN_TEST_SAFE_STATE_STOP               4U  /**< 正在执行停止序列。 */
#define APP_CAN_TEST_SAFE_STATE_DONE               5U  /**< 已停止并静默。 */

#define APP_CAN_TEST_SAFE_REASON_NONE              0U  /**< 无异常。 */
#define APP_CAN_TEST_SAFE_REASON_RUN_DONE          1U  /**< 正常跑完时长。 */
#define APP_CAN_TEST_SAFE_REASON_NO_FEEDBACK       2U  /**< 启动后没有反馈。 */
#define APP_CAN_TEST_SAFE_REASON_FEEDBACK_LOST     3U  /**< 运行中反馈中断。 */
#define APP_CAN_TEST_SAFE_REASON_POS_LIMIT         4U  /**< 位置变化超限。 */
#define APP_CAN_TEST_SAFE_REASON_STALL             5U  /**< 持续指令下没有转速。 */
#define APP_CAN_TEST_SAFE_REASON_TIMEOUT           6U  /**< 总超时。 */

/** @brief 保守测试状态机当前状态。 */
extern volatile uint32_t app_can_test_safe_state;

/** @brief 保守测试停止原因。 */
extern volatile uint32_t app_can_test_safe_reason;

/** @brief 目标电机的反馈帧计数。 */
extern volatile uint32_t app_can_test_safe_fb_count;

/** @brief 目标电机最近一帧原始反馈数据。 */
extern volatile uint8_t app_can_test_safe_fb_data[8];

/** @brief 最近解算的反馈位置，单位 rad。 */
extern volatile float app_can_test_safe_pos;

/** @brief 最近解算的反馈速度，单位 rad/s。 */
extern volatile float app_can_test_safe_vel;

/** @brief 启动时的位置基准，单位 rad。 */
extern volatile float app_can_test_safe_start_pos;

/** @brief 相对启动位置的变化量，单位 rad。 */
extern volatile float app_can_test_safe_moved_rad;

/** @brief 当前速度指令，单位 rad/s。 */
extern volatile float app_can_test_safe_cmd_vel;

/** @brief 距最近一帧反馈的时间，单位 ms。 */
extern volatile uint32_t app_can_test_safe_fb_age_ms;

/** @brief 保守测试已运行时间，单位 ms。 */
extern volatile uint32_t app_can_test_safe_elapsed_ms;

void app_can_test_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_CAN_TEST_H */
