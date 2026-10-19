/**
 ******************************************************************************
 * @file    bsp_timebase.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   Timebase BSP 的 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/bsp_timebase.c。
 * @note    C 文件应包含 Src/bsp_timebase.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_TIMEBASE_HPP
#define PRINTK_BSP_TIMEBASE_HPP

#include "Src/bsp_timebase.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Timebase {

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化板级时间基。 */
inline void init()
{
    bsp_timebase_init();
}

/** @brief 获取当前微秒时间。 */
inline uint64_t now_us()
{
    return bsp_timebase_now_us();
}

/** @brief 通知时间基溢出。 */
inline void overflow()
{
    bsp_timebase_on_overflow();
}

/** @brief 根据 TIM 句柄通知时间基定时器溢出。 */
inline void tim_overflow(TIM_HandleTypeDef *htim)
{
    bsp_timebase_on_timer_overflow(htim);
}

} // namespace Bsp::Timebase

#endif // PRINTK_BSP_TIMEBASE_HPP
