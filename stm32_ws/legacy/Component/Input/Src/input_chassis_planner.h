/**
 ******************************************************************************
 * @file    input_chassis_planner.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   底盘 planner 输入占位接口。
 *
 * @note    本轮只保留 planner 接口占位，不实现轨迹规划、限速或参考整形。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_CHASSIS_PLANNER_H
#define PRINTK_COMPONENT_INPUT_CHASSIS_PLANNER_H

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
 * @brief 底盘 planner 占位输入。
 */
typedef struct
{
    uint32_t tick_ms; /**< 输入时间戳，单位 ms。 */
    uint8_t priority; /**< 预留优先级。 */
} InputChassisPlannerState;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 将底盘 planner 输入映射为底盘参考。
 *
 * @param state planner 输入。
 * @param out 底盘参考输出。
 * @return 执行状态。
 */
InputStatus input_chassis_planner_map(const InputChassisPlannerState *state, InputChassisRef *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_CHASSIS_PLANNER_H
