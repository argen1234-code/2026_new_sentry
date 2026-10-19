/**
 ******************************************************************************
 * @file    app_chassis.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Chassis 应用层接口。
 *
 * @note    本模块负责把 Input、Observe 和电机反馈注入 Component/Controller/Chassis。
 * @note    本模块不实现遥控映射、状态估计、功率读取或旧底盘任务兼容接口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_CHASSIS_H
#define PRINTK_APPS_CHASSIS_H

#include <stdint.h>

#include "chassis.h"
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
 * @brief Chassis 应用层诊断信息。
 */
typedef struct
{
    uint32_t loop_count;                         /**< 周期执行次数。 */
    uint32_t input_lost_count;                   /**< 输入命令离线或读取失败次数。 */
    uint32_t observe_lost_count;                 /**< INS 姿态无效次数。 */
    uint32_t drive_fb_lost_count;                /**< 驱动电机反馈不完整次数。 */
    uint32_t steer_fb_lost_count;                /**< 舵向电机反馈不完整次数。 */
    uint32_t chassis_fail_count;                 /**< Chassis controller 返回非 ChassisOk 次数。 */
    uint32_t send_count;                         /**< 电机发送成功次数。 */
    uint32_t send_fail_count;                    /**< 电机发送失败次数。 */
    uint32_t zero_count;                         /**< 输出被清零发送次数。 */
    uint32_t last_tick_ms;                       /**< 最近一次周期 tick，单位 ms。 */
    uint32_t last_time_us;                       /**< 最近一次周期时间戳低 32 位，单位 us。 */
    ChassisStatus chassis_status;                /**< 最近一次 Chassis controller 状态。 */
    motor_status_t drive_status[CHASSIS_DRIVE_MAX]; /**< 最近一次驱动反馈读取状态。 */
    motor_status_t steer_status[CHASSIS_STEER_MAX]; /**< 最近一次舵向反馈读取状态。 */
    motor_status_t drive_send_status;            /**< 最近一次驱动电机发送状态。 */
    motor_status_t steer_send_status;            /**< 最近一次舵向电机发送状态。 */
    ChassisMode mode;                            /**< 最近一次输出模式。 */
    int16_t drive_current[CHASSIS_DRIVE_MAX];    /**< 最近一次驱动电流。 */
    int16_t steer_current[CHASSIS_STEER_MAX];    /**< 最近一次舵向电流。 */
    uint8_t input_online;                        /**< 非 0 表示最近一次输入命令在线。 */
    uint8_t observe_ready;                       /**< 非 0 表示最近一次 INS ready。 */
    uint8_t drive_online_mask;                   /**< 驱动反馈在线 bit mask。 */
    uint8_t steer_online_mask;                   /**< 舵向反馈在线 bit mask。 */
    uint8_t send_enable;                         /**< 非 0 表示最近一次 controller 允许发送。 */
} AppChassisDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Chassis 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_chassis_init(void);

/**
 * @brief 获取当前 Chassis 公共上下文快照。
 *
 * @param out 公共上下文输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_chassis_get_state(Chassis *out);

/**
 * @brief 获取最近一次 ChassisInput 快照。
 *
 * @param out 周期输入输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_chassis_get_input(ChassisInput *out);

/**
 * @brief 获取最近一次 ChassisOutput 快照。
 *
 * @param out 周期输出输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_chassis_get_output(ChassisOutput *out);

/**
 * @brief 获取 Chassis 应用层诊断快照。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_chassis_get_diag(AppChassisDiag *out);

/**
 * @brief 注入底盘功率测量。
 *
 * @param power 当前底盘功率，单位 W。
 * @param limit 当前底盘功率上限，单位 W；小于等于 0 时使用 target 默认值。
 * @param buffer 当前底盘缓冲能量。
 * @return 成功返回 1，参数非法返回 0。
 */
uint8_t app_chassis_set_power(float32_t power, float32_t limit, float32_t buffer);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_CHASSIS_H */
