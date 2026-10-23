/**
 ******************************************************************************
 * @file    input_gimbal_vision.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   视觉到云台参考的映射接口。
 *
 * @note    本文件只提供视觉输入源注入接口，视觉数据只覆盖云台相关参考。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_GIMBAL_VISION_H
#define PRINTK_COMPONENT_INPUT_GIMBAL_VISION_H

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
 * @brief 视觉云台输入状态。
 */
typedef struct
{
    float32_t yaw;       /**< yaw 修正或目标参考。 */
    float32_t pitch;     /**< pitch 修正或目标参考。 */
    GimbalMode mode;     /**< 云台模式参考。 */
    uint32_t valid_mask; /**< INPUT_GIMBAL_* 有效位。 */
    uint8_t mode_valid;  /**< 非 0 表示 mode 有效。 */
    uint8_t online;      /**< 非 0 表示视觉输入在线。 */
    uint8_t force_stop;  /**< 非 0 表示强制停止云台命令输出。 */
    uint8_t priority;    /**< 输出参考优先级。 */
    uint32_t tick_ms;    /**< 输入时间戳，单位 ms。 */
} InputVisionState;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 将视觉输入映射为云台参考。
 *
 * @param state 视觉输入状态。
 * @param out 云台参考输出。
 * @return 执行状态。
 */
InputStatus input_gimbal_vision_map(const InputVisionState *state, InputGimbalRef *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_GIMBAL_VISION_H
