/**
 ******************************************************************************
 * @file    app_observe.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Observe 应用层接口。
 *
 * @note    本模块负责把 IMU 设备采样和外部注入测量送入 Observer，并缓存
 *          ObserverState 供控制 app 使用。
 * @note    本模块不保留旧 INS 指针式接口，不读取旧 INS/chassis 全局。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_OBSERVE_H
#define PRINTK_APPS_OBSERVE_H

#include <stdint.h>

#include "observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief Observe 应用层诊断信息。
 */
typedef struct
{
    uint32_t loop_count;        /**< 周期执行次数。 */
    uint32_t stale_count;       /**< dt 非法或调度断更计数。 */
    uint32_t imu_lost_count;    /**< IMU 读取失败计数。 */
    uint32_t observe_fail_count;/**< Observer 返回错误计数。 */
    uint32_t heat_off_count;    /**< 温控安全关闭计数。 */
    uint32_t last_time_us;      /**< 最近一次周期时间戳低 32 位，单位 us。 */
    float32_t last_dt;          /**< 最近一次有效 dt，单位 s。 */
    float32_t temperature;      /**< 最近一次 IMU 温度，单位摄氏度。 */
    float32_t heat_target;      /**< 当前温控目标温度，单位摄氏度。 */
    float32_t heat_compare;     /**< 最近一次温控 compare 浮点输出。 */
    float32_t yaw;              /**< 最近一次 yaw 角，单位 rad。 */
    float32_t pitch;            /**< 最近一次 pitch 角，单位 rad。 */
    float32_t roll;             /**< 最近一次 roll 角，单位 rad。 */
    float32_t yaw_total;        /**< 最近一次 yaw 连续角，单位 rad。 */
    ObserverStatus status;      /**< 最近一次 Observer 状态。 */
    uint32_t thermal_status;    /**< 最近一次 ThermalStatus，避免 app 头暴露温控实现。 */
    uint8_t imu_online;         /**< 非 0 表示最近一次 IMU 输入有效。 */
    uint8_t ins_ready;          /**< 非 0 表示 INS 已 ready。 */
    uint8_t heat_enable;        /**< 非 0 表示最近一次允许加热。 */
    uint8_t chassis_valid_mask; /**< 最近一次底盘速度观测有效轴掩码。 */
} AppObserveDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Observe 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_observe_init(void);

/**
 * @brief 获取当前 Observer 状态快照。
 *
 * @param out 状态输出指针。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_observe_get_state(ObserverState *out);

/**
 * @brief 获取 Observe 应用层诊断快照。
 *
 * @param out 诊断输出指针。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_observe_get_diag(AppObserveDiag *out);

/**
 * @brief 注入底盘速度和加速度测量。
 *
 * @note vel_mask 和 acc_mask 使用 OBSERVER_AXIS_X/Y/Z 组合。某一类 mask
 *       为 0 时，对应指针允许为空并会清除该类测量。
 *
 * @param vel 底盘速度测量，单位由调用方和 Controller 约定。
 * @param vel_mask 速度测量有效轴掩码。
 * @param acc 底盘加速度测量，单位由调用方和 Controller 约定。
 * @param acc_mask 加速度测量有效轴掩码。
 * @param timestamp_us 测量时间戳低 32 位，单位 us；0 表示使用 Observe 周期时间戳。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_observe_set_chassis_measure(const Vec3 *vel,
                                        uint8_t vel_mask,
                                        const Vec3 *acc,
                                        uint8_t acc_mask,
                                        uint32_t timestamp_us);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_OBSERVE_H */
