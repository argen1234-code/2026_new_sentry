/**
 ******************************************************************************
 * @file    debug_platform.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug 平台临界区与时间接口。
 *
 * @note    本文件只声明平台抽象，STM32 实现位于 debug_platform_stm32.c。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_DEBUG_PLATFORM_H
#define PRINTK_COMPONENT_DEBUG_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 进入临界区前保存的平台中断状态。 */
typedef uint32_t DebugIrqState;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 进入 Debug 临界区。
 *
 * @return 进入前的中断状态。
 */
DebugIrqState debug_lock(void);

/**
 * @brief 退出 Debug 临界区。
 *
 * @param state 由 debug_lock 返回的中断状态。
 */
void debug_unlock(DebugIrqState state);

/**
 * @brief 获取平台微秒时间戳。
 *
 * @return 当前单调时间戳，单位 us。
 */
uint64_t debug_time_us(void);

/**
 * @brief 获取平台周期计数。
 *
 * @return 当前 DWT CYCCNT；平台不可用时返回 0。
 */
uint32_t debug_cycles(void);

/**
 * @brief 判断当前是否处于中断上下文。
 *
 * @return 中断上下文返回 true，否则返回 false。
 */
bool debug_in_isr(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_DEBUG_PLATFORM_H
