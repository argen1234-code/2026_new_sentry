/**
 ******************************************************************************
 * @file    input_cali_rc.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   RC 到校准命令的映射接口。
 *
 * @note    本文件只把遥控器物理输入解释为校准请求，不执行校准流程、
 *          Flash 写入或蜂鸣器提示。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_CALI_RC_H
#define PRINTK_COMPONENT_INPUT_CALI_RC_H

#include "input_common.h"
#include "rc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief RC 校准映射配置。
 */
typedef struct
{
    uint16_t begin_count;      /**< 进入校准手势需要连续保持的周期数。 */
    uint16_t action_count;     /**< 触发具体校准动作需要连续保持的周期数。 */
    uint16_t gimbal_key;       /**< 云台校准快捷键，0 表示禁用。 */
    uint16_t imu_key;          /**< IMU 校准快捷键，0 表示禁用。 */
    uint8_t enable_old_gesture;/**< 非 0 表示启用旧摇杆组合手势。 */
    uint8_t priority;          /**< 输出参考优先级。 */
} InputCaliRcMap;

/**
 * @brief RC 校准映射上下文。
 */
typedef struct
{
    uint16_t begin_hold; /**< 进入手势保持计数。 */
    uint16_t action_hold;/**< 动作手势保持计数。 */
    uint16_t last_key;   /**< 上一次键盘位图。 */
    uint8_t armed;       /**< 非 0 表示已经进入校准手势。 */
    uint8_t last_action; /**< 上一次动作是否有效，用于边沿触发。 */
} InputCaliRcCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 RC 校准映射上下文。
 *
 * @param ctx 映射上下文。
 */
void input_cali_rc_init(InputCaliRcCtx *ctx);

/**
 * @brief 将 RC 物理输入映射为校准参考。
 *
 * @param ctx 映射上下文。
 * @param rc DBUS 风格物理输入状态。
 * @param map 映射配置。
 * @param out 校准参考输出。
 * @return 执行状态。
 */
InputStatus input_cali_rc_map(InputCaliRcCtx *ctx,
                              const RcDbusState *rc,
                              const InputCaliRcMap *map,
                              InputCaliRef *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_CALI_RC_H
