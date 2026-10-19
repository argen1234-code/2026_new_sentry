/**
 ******************************************************************************
 * @file    bsp_dwt.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   DWT BSP 的 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/bsp_dwt.c。
 * @note    C 文件应包含 Src/bsp_dwt.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_DWT_HPP
#define PRINTK_BSP_DWT_HPP

#include "Src/bsp_dwt.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Dwt {

/** @brief DWT 拆分时间。 */
using Time = dwt_time_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 DWT 时间基。 */
inline void init(uint32_t cpu_freq_mhz)
{
    bsp_dwt_init(cpu_freq_mhz);
}

/** @brief 获取单精度采样间隔。 */
inline float delta(uint32_t *cnt_last)
{
    return bsp_dwt_get_delta_t(cnt_last);
}

/** @brief 获取双精度采样间隔。 */
inline double delta64(uint32_t *cnt_last)
{
    return bsp_dwt_get_delta_t64(cnt_last);
}

/** @brief 获取秒级时间线。 */
inline float timeline_s()
{
    return bsp_dwt_get_timeline_s();
}

/** @brief 获取毫秒级时间线。 */
inline float timeline_ms()
{
    return bsp_dwt_get_timeline_ms();
}

/** @brief 获取微秒级时间线。 */
inline uint64_t timeline_us()
{
    return bsp_dwt_get_timeline_us();
}

/** @brief 获取拆分时间。 */
inline void time(Time *out)
{
    bsp_dwt_get_time(out);
}

/** @brief DWT 阻塞延时。 */
inline void delay(float delay_s)
{
    bsp_dwt_delay(delay_s);
}

} // namespace Bsp::Dwt

#endif // PRINTK_BSP_DWT_HPP
