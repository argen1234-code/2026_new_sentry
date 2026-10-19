/**
 ******************************************************************************
 * @file    bsp_timebase.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   微秒级系统时间基实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_timebase.h"

#include "bsp_tim.h"
#include "robot_board.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief 获取当前 timebase 使用的硬件 TIM。
 *
 * timebase 具体定时器由 Robot/robot_board.h 配置，本文件只通过 BSP TIM 表解析句柄。
 *
 * @return 找到则返回 TIM 句柄，否则返回 NULL。
 */
static TIM_HandleTypeDef *timebase_timer(void)
{
    return bsp_tim_handle(BSP_TIMEBASE_TIM_ID);
}

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * BSP 层只缓存外设句柄、DMA 缓冲和硬件状态，具体设备用途由 Robot 配置决定。
 * ========================================================================== */

/**
 * @brief timebase 溢出累计高位计数。
 *
 * 与底层 32 位计数器共同组成 64 位微秒时间戳。该值会在 TIM 更新中断
 * 和 bsp_timebase_now_us() 的补偿路径中更新，因此保持 volatile。
 */
static volatile uint32_t bsp_timebase_high_word;

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_timebase_init(void)
{
    TIM_HandleTypeDef *timebase_htim = timebase_timer();

    bsp_timebase_high_word = 0U;
    if (timebase_htim == NULL) {
        return;
    }

    bsp_tim_set_counter(timebase_htim, 0U);
    __HAL_TIM_CLEAR_FLAG(timebase_htim, TIM_FLAG_UPDATE);
    (void)bsp_tim_start(timebase_htim, BSP_TIM_MODE_BASE_IT);
}

void bsp_timebase_on_overflow(void)
{
    bsp_timebase_high_word++;
}

void bsp_timebase_on_timer_overflow(TIM_HandleTypeDef *htim)
{
    TIM_HandleTypeDef *timebase_htim = timebase_timer();

    if ((htim == NULL) || (timebase_htim == NULL) || (htim->Instance != timebase_htim->Instance)) {
        return;
    }

    bsp_timebase_on_overflow();
}

uint64_t bsp_timebase_now_us(void)
{
    TIM_HandleTypeDef *timebase_htim = timebase_timer();
    uint32_t high_word;
    uint32_t low_word;
    uint32_t primask = __get_PRIMASK();

    if (timebase_htim == NULL) {
        return 0ULL;
    }

    __disable_irq();

    high_word = bsp_timebase_high_word;
    low_word = bsp_tim_get_counter(timebase_htim);
    if (__HAL_TIM_GET_FLAG(timebase_htim, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(timebase_htim, TIM_FLAG_UPDATE);
        high_word++;
        bsp_timebase_high_word = high_word;
        low_word = bsp_tim_get_counter(timebase_htim);
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return ((uint64_t)high_word << 32) | (uint64_t)low_word;
}
