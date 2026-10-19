/**
 ******************************************************************************
 * @file    app_gimbal.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Gimbal 应用层接口。
 *
 * @note    本模块负责把 Input、Observe 和电机反馈注入 Component/Controller/Gimbal。
 * @note    本模块不实现自瞄、遥控映射、状态估计或旧云台任务兼容接口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_GIMBAL_H
#define PRINTK_APPS_GIMBAL_H

#include <stdint.h>

#include "gimbal.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief Gimbal 应用层阶段。
 */
typedef enum
{
    AppGimbalPhaseInit = 0,  /**< 上电初始化回中阶段。 */
    AppGimbalPhaseReady = 1, /**< 正常控制阶段。 */
    AppGimbalPhaseCali = 2,  /**< 限位校准阶段。 */
    AppGimbalPhaseError = 3  /**< 错误阶段，输出保持清零。 */
} AppGimbalPhase;

/**
 * @brief Gimbal 应用层诊断信息。
 */
typedef struct
{
    uint32_t loop_count;          /**< 周期执行次数。 */
    uint32_t input_lost_count;    /**< 输入命令离线或读取失败次数。 */
    uint32_t observe_lost_count;  /**< INS 未 ready 或读取失败次数。 */
    uint32_t yaw_fb_lost_count;   /**< yaw 电机反馈离线次数。 */
    uint32_t pitch_fb_lost_count; /**< pitch 电机反馈离线次数。 */
    uint32_t gimbal_fail_count;   /**< Gimbal controller 返回非 GimbalOk 次数。 */
    uint32_t send_count;          /**< 电机发送成功次数。 */
    uint32_t send_fail_count;     /**< 电机发送失败次数。 */
    uint32_t zero_count;          /**< 输出被清零发送次数。 */
    uint32_t init_done_count;     /**< 初始化完成次数。 */
    uint32_t cali_done_count;     /**< 校准完成次数。 */
    uint32_t cali_fail_count;     /**< 校准失败次数。 */
    uint32_t last_tick_ms;        /**< 最近一次周期 tick，单位 ms。 */
    uint32_t last_time_us;        /**< 最近一次周期时间戳低 32 位，单位 us。 */
    GimbalStatus gimbal_status;   /**< 最近一次 Gimbal controller 状态。 */
    motor_status_t yaw_status;    /**< 最近一次 yaw 反馈读取状态。 */
    motor_status_t pitch_status;  /**< 最近一次 pitch 反馈读取状态。 */
    motor_status_t send_status;   /**< 最近一次电机发送状态。 */
    GimbalMode mode;              /**< 最近一次输出模式。 */
    AppGimbalPhase phase;         /**< 当前应用层阶段。 */
    int16_t yaw_current;          /**< 最近一次 yaw 输出电流。 */
    int16_t pitch_current;        /**< 最近一次 pitch 输出电流。 */
    int16_t aux_current;          /**< 最近一次辅助输出电流。 */
    uint8_t cali_step;            /**< 当前校准步骤。 */
    uint8_t input_online;         /**< 非 0 表示最近一次输入命令在线。 */
    uint8_t observe_ready;        /**< 非 0 表示最近一次 INS ready。 */
    uint8_t yaw_online;           /**< 非 0 表示最近一次 yaw 反馈在线。 */
    uint8_t pitch_online;         /**< 非 0 表示最近一次 pitch 反馈在线。 */
    uint8_t send_enable;          /**< 非 0 表示最近一次 controller 允许发送。 */
} AppGimbalDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Gimbal 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_gimbal_init(void);

/**
 * @brief 获取当前 Gimbal 公共上下文快照。
 *
 * @param out 公共上下文输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_gimbal_get_state(Gimbal *out);

/**
 * @brief 获取最近一次 GimbalInput 快照。
 *
 * @param out 周期输入输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_gimbal_get_input(GimbalInput *out);

/**
 * @brief 获取最近一次 GimbalOutput 快照。
 *
 * @param out 周期输出输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_gimbal_get_output(GimbalOutput *out);

/**
 * @brief 获取 Gimbal 应用层诊断快照。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_gimbal_get_diag(AppGimbalDiag *out);

/**
 * @brief 判断云台应用层是否已 ready。
 *
 * @return ready 返回 1，否则返回 0。
 */
uint8_t app_gimbal_ready(void);

/**
 * @brief 读取当前云台应用层阶段。
 *
 * @param out 阶段输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_gimbal_get_phase(AppGimbalPhase *out);

/**
 * @brief 请求进入云台限位校准阶段。
 *
 * @return 成功接收请求返回 1，当前状态不允许或参数非法返回 0。
 */
uint8_t app_gimbal_start_cali(void);

/**
 * @brief 注入云台校准结果。
 *
 * @param cali 校准结果。
 * @return 成功返回 1，参数无效或 controller 拒绝返回 0。
 */
uint8_t app_gimbal_set_cali(const GimbalCali *cali);

/**
 * @brief 读取最近一次有效云台校准结果。
 *
 * @param out 校准结果输出。
 * @return 有有效校准返回 1，否则返回 0。
 */
uint8_t app_gimbal_get_cali(GimbalCali *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_GIMBAL_H */
