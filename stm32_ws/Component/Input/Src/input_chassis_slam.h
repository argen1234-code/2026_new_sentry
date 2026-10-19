/**
 ******************************************************************************
 * @file    input_chassis_slam.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   SLAM 到底盘参考的映射接口。
 *
 * @note    本文件只提供 SLAM 输入源注入接口，SLAM 数据只覆盖底盘相关参考。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_CHASSIS_SLAM_H
#define PRINTK_COMPONENT_INPUT_CHASSIS_SLAM_H

#include "input_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief SLAM 底盘输入状态。
 */
typedef struct
{
    float32_t vx;          /**< 纵向速度参考。 */
    float32_t vy;          /**< 横向速度参考。 */
    float32_t wz;          /**< 角速度参考。 */
    float32_t yaw_ref;     /**< yaw 角参考。 */
    float32_t frame_angle; /**< 速度坐标系旋转角参考。 */
    float32_t leg_length;  /**< 统一腿长参考。 */
    float32_t leg_left;    /**< 左腿参考。 */
    float32_t leg_right;   /**< 右腿参考。 */
    ChassisMode mode;      /**< 底盘模式参考。 */
    uint32_t valid_mask;   /**< INPUT_CHASSIS_* 有效位。 */
    uint8_t mode_valid;    /**< 非 0 表示 mode 有效。 */
    uint8_t online;        /**< 非 0 表示 SLAM 输入在线。 */
    uint8_t force_stop;    /**< 非 0 表示强制停止底盘命令输出。 */
    uint8_t jump;          /**< 非 0 表示跳跃请求。 */
    uint8_t priority;      /**< 输出参考优先级。 */
    uint32_t tick_ms;      /**< 输入时间戳，单位 ms。 */
} InputSlamState;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 将 SLAM 输入映射为底盘参考。
 *
 * @param state SLAM 输入状态。
 * @param out 底盘参考输出。
 * @return 执行状态。
 */
InputStatus input_chassis_slam_map(const InputSlamState *state, InputChassisRef *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_CHASSIS_SLAM_H
