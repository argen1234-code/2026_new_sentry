/**
 ******************************************************************************
 * @file    app_shoot.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Shoot 应用层接口。
 *
 * @note    本模块负责把 Input、电机反馈和外部热量注入 Component/Controller/Shoot。
 * @note    本模块不读取遥控器、裁判系统、云台任务或旧发射任务全局状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_SHOOT_H
#define PRINTK_APPS_SHOOT_H

#include <stdint.h>

#include "shoot.h"
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
 * @brief Shoot 应用层诊断信息。
 */
typedef struct
{
    uint32_t loop_count;                       /**< 周期执行次数。 */
    uint32_t input_lost_count;                 /**< 输入命令离线或读取失败次数。 */
    uint32_t trigger_fb_lost_count;            /**< trigger 反馈离线次数。 */
    uint32_t fric_fb_lost_count;               /**< 摩擦轮反馈不完整次数。 */
    uint32_t shoot_fail_count;                 /**< Shoot controller 返回非 ShootOk 次数。 */
    uint32_t fric_temp_over_count;             /**< 摩擦轮温度触发安全钳位次数。 */
    uint32_t send_count;                       /**< 电机发送成功次数。 */
    uint32_t send_fail_count;                  /**< 电机发送失败次数。 */
    uint32_t zero_count;                       /**< 输出被清零发送次数。 */
    uint32_t last_tick_ms;                     /**< 最近一次周期 tick，单位 ms。 */
    uint32_t last_time_us;                     /**< 最近一次周期时间戳低 32 位，单位 us。 */
    ShootStatus shoot_status;                  /**< 最近一次 Shoot controller 状态。 */
    motor_status_t trigger_status;             /**< 最近一次 trigger 反馈读取状态。 */
    motor_status_t fric_status[SHOOT_FRIC_MAX];/**< 最近一次摩擦轮反馈读取状态。 */
    motor_status_t trigger_send_status;        /**< 最近一次 trigger 发送状态。 */
    motor_status_t fric_send_status;           /**< 最近一次摩擦轮发送状态。 */
    ShootMode mode;                            /**< 最近一次输出模式。 */
    int16_t trigger_current;                   /**< 最近一次 trigger 输出电流。 */
    int16_t fric_current[SHOOT_FRIC_MAX];      /**< 最近一次摩擦轮输出电流。 */
    uint8_t fric_temperature[SHOOT_FRIC_MAX];  /**< 最近一次摩擦轮温度。 */
    float32_t heat;                            /**< 最近一次注入到 controller 的热量。 */
    float32_t heat_limit;                      /**< 最近一次注入到 controller 的热量上限。 */
    uint8_t input_online;                      /**< 非 0 表示最近一次输入命令在线。 */
    uint8_t trigger_online;                    /**< 非 0 表示最近一次 trigger 反馈在线。 */
    uint8_t fric_online_mask;                  /**< 摩擦轮反馈在线 bit mask。 */
    uint8_t fric_temp_safe;                    /**< 非 0 表示摩擦轮温度未触发安全钳位。 */
    uint8_t send_enable;                       /**< 非 0 表示最近一次 controller 允许发送。 */
} AppShootDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Shoot 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_shoot_init(void);

/**
 * @brief 获取当前 Shoot 公共上下文快照。
 *
 * @param out 公共上下文输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_shoot_get_state(Shoot *out);

/**
 * @brief 获取最近一次 ShootInput 快照。
 *
 * @param out 周期输入输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_shoot_get_input(ShootInput *out);

/**
 * @brief 获取最近一次 ShootOutput 快照。
 *
 * @param out 周期输出输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_shoot_get_output(ShootOutput *out);

/**
 * @brief 获取 Shoot 应用层诊断快照。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_shoot_get_diag(AppShootDiag *out);

/**
 * @brief 注入当前发射热量测量。
 *
 * @param heat 当前热量。
 * @param limit 当前热量上限；小于等于 0 时周期内使用 target 默认值。
 * @return 成功返回 1，参数非法返回 0。
 */
uint8_t app_shoot_set_heat(float32_t heat, float32_t limit);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_SHOOT_H */
