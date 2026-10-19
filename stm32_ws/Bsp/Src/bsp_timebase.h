/**
 ******************************************************************************
 * @file    bsp_timebase.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   微秒级系统时间基接口。
 *
 * @note    底层定时器由 Robot/robot_board.h 配置，对外不暴露具体 TIM。
 * @note    CubeMX/HAL 必须保证该 timer counter clock 为 1 MHz，即计数器
 *          每增加 1 表示 1 us。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_TIMEBASE_H
#define PRINTK_BSP_TIMEBASE_H

#include "tim.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief 初始化微秒级系统时间基。
 *
 * 当前实现会清零配置的 timebase 定时器计数器和溢出高位，并启动基本定时器中断。
 */
void bsp_timebase_init(void);

/**
 * @brief 获取当前微秒时间戳。
 *
 * @return 当前 64 位时间戳，单位为微秒。
 */
uint64_t bsp_timebase_now_us(void);

/**
 * @brief 处理当前 timebase 定时器溢出。
 *
 * 该接口只累加 timebase 高位，通常由 bsp_timebase_on_timer_overflow() 调用。
 */
void bsp_timebase_on_overflow(void);

/**
 * @brief 在 TIM 周期回调中转发 timebase 溢出事件。
 *
 * @param htim 触发周期更新事件的 TIM 句柄。
 */
void bsp_timebase_on_timer_overflow(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_TIMEBASE_H
