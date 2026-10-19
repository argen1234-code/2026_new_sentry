/**
 ******************************************************************************
 * @file    app_referee.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Referee 应用层接口。
 *
 * @note    本模块维护裁判系统应用快照，并把热量、功率和 UI 发送作为应用层 glue 处理。
 * @note    裁判系统是不可靠输入源；离线或有效位不足时不向控制链路注入过期数据。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_REFEREE_H
#define PRINTK_APPS_REFEREE_H

#include <stdint.h>

#include "referee.h"
#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief Referee 应用层状态快照。
 */
typedef struct
{
    uint32_t valid_mask;          /**< 最近一次裁判系统有效位，使用 REFEREE_VALID_*。 */
    uint32_t last_rx_tick;        /**< 最近一次合法裁判帧 tick，单位 ms。 */
    uint32_t last_update_ms;      /**< App 最近一次更新时刻，单位 ms。 */
    uint16_t client_id;           /**< 当前机器人对应的客户端 ID。 */
    uint8_t robot_id;             /**< 当前机器人 ID，未知时为 0。 */
    uint8_t online;               /**< 非 0 表示裁判系统在线。 */
    uint8_t stale;                /**< 非 0 表示存在历史帧但当前已经超时。 */
    uint8_t heat_valid;           /**< 非 0 表示 shoot_heat/shoot_heat_limit 来自当前有效帧。 */
    float shoot_heat;             /**< 当前发射热量。 */
    float shoot_heat_limit;       /**< 当前发射热量上限。 */
    uint8_t power_valid;          /**< 非 0 表示 chassis_power* 字段来自当前有效帧。 */
    float chassis_power;          /**< 当前底盘功率，协议未提供时为 0。 */
    float chassis_power_limit;    /**< 当前底盘功率上限。 */
    float chassis_buffer;         /**< 当前底盘缓冲能量。 */
    RefereeWarning warning;       /**< 最近一次裁判警告数据。 */
    RefereeHurt hurt;             /**< 最近一次受击数据。 */
    RefereeShootData shoot_data;  /**< 最近一次发射数据。 */
} AppRefereeState;

/**
 * @brief Referee 应用层诊断信息。
 */
typedef struct
{
    uint32_t loop_count;             /**< 周期执行次数。 */
    uint32_t offline_count;          /**< 裁判系统离线周期次数。 */
    uint32_t stale_count;            /**< 裁判系统历史数据超时次数。 */
    uint32_t valid_fail_count;       /**< 在线但关键有效位不足次数。 */
    uint32_t heat_inject_count;      /**< 发射热量注入成功次数。 */
    uint32_t heat_fail_count;        /**< 发射热量读取或注入失败次数。 */
    uint32_t power_inject_count;     /**< 底盘功率注入成功次数。 */
    uint32_t power_fail_count;       /**< 底盘功率读取或注入失败次数。 */
    uint32_t ui_init_count;          /**< UI 初始化成功次数。 */
    uint32_t ui_init_fail_count;     /**< UI 初始化失败次数。 */
    uint32_t ui_send_count;          /**< UI step 成功次数。 */
    uint32_t ui_fail_count;          /**< UI step 失败次数。 */
    uint32_t last_tick_ms;           /**< 最近一次周期 tick，单位 ms。 */
    uint32_t last_time_us;           /**< 最近一次周期时间戳低 32 位，单位 us。 */
    RefereeStatus init_status;       /**< 裁判设备初始化状态。 */
    RefereeStatus state_status;      /**< 最近一次裁判状态读取状态。 */
    RefereeStatus diag_status;       /**< 最近一次裁判诊断读取状态。 */
    RefereeStatus heat_status;       /**< 最近一次发射热量读取状态。 */
    RefereeStatus power_status;      /**< 最近一次底盘功率读取状态。 */
    UiStatus ui_init_status;         /**< 最近一次 UI 初始化状态。 */
    UiStatus ui_status;              /**< 最近一次 UI step 状态。 */
    uint8_t ready;                   /**< 非 0 表示 app 已初始化。 */
    uint8_t ui_ready;                /**< 非 0 表示 UI 已按当前 robot_id 初始化。 */
} AppRefereeDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Referee 应用任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_referee_init(void);

/**
 * @brief 获取 Referee 应用层状态快照。
 *
 * @param out 状态快照输出。
 * @return 成功返回 1，参数非法返回 0。
 */
uint8_t app_referee_get_state(AppRefereeState *out);

/**
 * @brief 获取 Referee 应用层诊断快照。
 *
 * @param out 诊断快照输出。
 * @return 成功返回 1，参数非法返回 0。
 */
uint8_t app_referee_get_diag(AppRefereeDiag *out);

/**
 * @brief 获取最近一次裁判系统原始状态缓存。
 *
 * @param out 原始状态输出。
 * @return 成功返回 1，参数非法或尚未取得有效缓存返回 0。
 */
uint8_t app_referee_get_raw(RefereeState *out);

/**
 * @brief 获取 UI 组件诊断快照。
 *
 * @param out UI 诊断输出。
 * @return 成功返回 1，参数非法或 UI 尚未初始化返回 0。
 */
uint8_t app_referee_get_ui_diag(UiDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_REFEREE_H */
