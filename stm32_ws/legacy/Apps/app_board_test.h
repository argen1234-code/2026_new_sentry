/**
 ******************************************************************************
 * @file    app_board_test.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   光板板载资源测试应用层接口。
 *
 * @note    本 App 只用于未接电机的 DM02 光板验证，覆盖 Flash、IMU、蜂鸣器、
 *          WS2812 和 USB CDC 日志。
 * @note    本 App 不读取电机、遥控器、裁判系统、云台、发射或旧任务全局。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_BOARD_TEST_H
#define PRINTK_APPS_BOARD_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态位与快照类型
 * --------------------------------------------------------------------------
 * 对外只暴露测试结果和诊断快照，便于后续串口日志或调试界面读取。
 * ========================================================================== */

/** @brief Flash 测试状态位。 */
#define APP_BOARD_TEST_BIT_FLASH                  (1UL << 0)

/** @brief IMU 测试状态位。 */
#define APP_BOARD_TEST_BIT_IMU                    (1UL << 1)

/** @brief WS2812 测试状态位。 */
#define APP_BOARD_TEST_BIT_WS2812                 (1UL << 2)

/** @brief 蜂鸣器测试状态位。 */
#define APP_BOARD_TEST_BIT_BUZZER                 (1UL << 3)

/** @brief CDC 打开状态位。 */
#define APP_BOARD_TEST_BIT_CDC                    (1UL << 4)

/**
 * @brief BoardTest 状态快照。
 */
typedef struct
{
    int32_t flash_status;     /**< 最近一次 Flash 测试状态。 */
    int32_t imu_status;       /**< 最近一次 IMU 初始化或采样状态。 */
    int32_t ws2812_status;    /**< 最近一次 WS2812 操作状态。 */
    int32_t buzzer_status;    /**< 最近一次蜂鸣器操作状态。 */
    int32_t dts_status;       /**< 最近一次 MCU DTS 采样状态。 */
    float imu_temp;           /**< 最近一次 IMU 温度，单位摄氏度。 */
    float mcu_temp;           /**< 最近一次 MCU DTS 温度，单位摄氏度。 */
    float accel[3];           /**< 最近一次 IMU 加速度，单位 m/s^2。 */
    float gyro[3];            /**< 最近一次 IMU 角速度，单位 rad/s。 */
    float euler[3];           /**< 最近一次姿态解算欧拉角，顺序 roll/pitch/yaw，单位 rad。 */
    float yaw_total;          /**< 连续 yaw 角，单位 rad。 */
    float gyro_bias[3];       /**< Observer 启动阶段估计的陀螺零偏，单位 rad/s。 */
    float yaw_drift;          /**< ready 后相对基准 yaw 漂移，单位 rad。 */
    float yaw_drift_deg_min;  /**< ready 后按当前窗口折算的 yaw 漂移，单位 deg/min。 */
    uint32_t pass_mask;       /**< 已通过测试项 bit mask。 */
    uint32_t fail_mask;       /**< 已失败测试项 bit mask，失败会锁存。 */
    uint32_t loop_count;      /**< 周期循环次数。 */
    uint32_t last_tick_ms;    /**< 最近一次周期时间戳，单位 ms。 */
    int32_t observer_status;  /**< 最近一次 Observer 执行状态。 */
    uint8_t cdc_ready;        /**< 非 0 表示 USB CDC 已打开且 DTR 就绪。 */
    uint8_t ins_valid;        /**< 非 0 表示 INS 解算输出有效。 */
    uint8_t ins_ready;        /**< 非 0 表示 INS 启动零偏采样完成。 */
    uint8_t ins_stable;       /**< 非 0 表示当前 IMU 静止判定成立。 */
    uint8_t flash_done;       /**< 非 0 表示 Flash 一次性测试已执行。 */
    uint8_t ready;            /**< 非 0 表示 BoardTest 任务已创建。 */
} AppBoardTestState;

/**
 * @brief BoardTest 诊断快照。
 */
typedef struct
{
    uint32_t log_ok_count;       /**< CDC 文本日志发送成功次数。 */
    uint32_t log_busy_count;     /**< CDC busy 次数。 */
    uint32_t log_error_count;    /**< CDC 发送错误次数。 */
    uint32_t vofa_ok_count;      /**< VOFA JustFloat 发送成功次数。 */
    uint32_t vofa_busy_count;    /**< VOFA JustFloat 发送 busy 次数。 */
    uint32_t vofa_error_count;   /**< VOFA JustFloat 发送失败次数。 */
    uint32_t imu_ok_count;       /**< IMU 周期采样成功次数。 */
    uint32_t imu_error_count;    /**< IMU 周期采样失败次数。 */
    uint32_t dts_ok_count;       /**< MCU DTS 周期采样成功次数。 */
    uint32_t dts_error_count;    /**< MCU DTS 周期采样失败次数。 */
    uint32_t observer_ok_count;  /**< Observer 姿态解算成功次数。 */
    uint32_t observer_error_count; /**< Observer 姿态解算失败次数。 */
    uint32_t ws2812_busy_count;  /**< WS2812 发送 busy 次数。 */
    uint32_t ws2812_error_count; /**< WS2812 操作错误次数。 */
    uint32_t buzzer_error_count; /**< 蜂鸣器操作错误次数。 */
    uint32_t flash_error_count;  /**< Flash 测试错误次数。 */
} AppBoardTestDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 初始化只启动测试任务；快照接口用于后续调试或临时人工查看。
 * ========================================================================== */

/**
 * @brief 初始化并启动光板测试任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_board_test_init(void);

/**
 * @brief 读取 BoardTest 状态快照。
 *
 * @param out 状态输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_board_test_get_state(AppBoardTestState *out);

/**
 * @brief 读取 BoardTest 诊断快照。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_board_test_get_diag(AppBoardTestDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_BOARD_TEST_H */
