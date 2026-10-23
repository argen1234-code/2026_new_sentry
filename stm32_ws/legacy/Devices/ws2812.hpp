/**
 ******************************************************************************
 * @file    ws2812.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   WS2812 设备 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/ws2812.c。
 * @note    C 文件应包含 Src/ws2812.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_WS2812_HPP
#define PRINTK_DEVICE_WS2812_HPP

#include "Src/ws2812.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Ws2812 {

/** @brief WS2812 设备状态码。 */
using Status = Ws2812Status;

/** @brief WS2812 颜色顺序。 */
using Order = Ws2812Order;

/** @brief WS2812 RGB 颜色。 */
using Color = Ws2812Color;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 默认最大灯珠数量。 */
inline constexpr uint16_t LedMax = WS2812_LED_MAX;

/** @brief 初始化 WS2812。 */
inline Status init(uint16_t led_count)
{
    return ws2812_init(led_count);
}

/** @brief 判断 WS2812 是否已初始化。 */
inline bool ready()
{
    return ws2812_ready() != 0U;
}

/** @brief 获取当前灯珠数量。 */
inline uint16_t count()
{
    return ws2812_count();
}

/** @brief 设置颜色顺序。 */
inline Status order(Order order)
{
    return ws2812_order(order);
}

/** @brief 设置单个灯珠颜色。 */
inline Status set(uint16_t index, Color color)
{
    return ws2812_set(index, color);
}

/** @brief 按 RGB 分量设置单个灯珠颜色。 */
inline Status set(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    return ws2812_set_rgb(index, r, g, b);
}

/** @brief 填充所有灯珠颜色。 */
inline Status fill(Color color)
{
    return ws2812_fill(color);
}

/** @brief 清空所有灯珠颜色缓存。 */
inline Status clear()
{
    return ws2812_clear();
}

/** @brief 发送当前颜色缓存。 */
inline Status show()
{
    return ws2812_show();
}

/** @brief 获取最近一次操作状态。 */
inline Status status()
{
    return ws2812_status();
}

} // namespace Device::Ws2812

#endif // PRINTK_DEVICE_WS2812_HPP
