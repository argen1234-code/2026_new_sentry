/**
 ******************************************************************************
 * @file    power.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   可控电源设备 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/power.c。
 * @note    C 文件应包含 Src/power.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_POWER_HPP
#define PRINTK_DEVICE_POWER_HPP

#include "Src/power.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Power {

/** @brief 电源设备状态码。 */
using Status = PowerStatus;

/** @brief 可控电源通道。 */
using Channel = PowerChannel;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化电源设备。 */
inline Status init()
{
    return power_init();
}

/** @brief 判断电源设备是否已初始化。 */
inline bool ready()
{
    return power_ready() != 0U;
}

/** @brief 设置指定电源通道状态。 */
inline Status set(Channel channel, bool enable)
{
    return power_set(channel, enable ? 1U : 0U);
}

/** @brief 打开指定电源通道。 */
inline Status on(Channel channel)
{
    return power_on(channel);
}

/** @brief 关闭指定电源通道。 */
inline Status off(Channel channel)
{
    return power_off(channel);
}

/** @brief 翻转指定电源通道状态。 */
inline Status toggle(Channel channel)
{
    return power_toggle(channel);
}

/** @brief 读取指定电源通道状态。 */
inline bool get(Channel channel)
{
    return power_get(channel) != 0U;
}

/** @brief 同时设置三路电源状态。 */
inline Status set_all(bool out1_enable, bool out2_enable, bool v5_enable)
{
    return power_set_all(out1_enable ? 1U : 0U, out2_enable ? 1U : 0U, v5_enable ? 1U : 0U);
}

/** @brief 关闭全部可控电源输出。 */
inline Status all_off()
{
    return power_all_off();
}

} // namespace Device::Power

#endif // PRINTK_DEVICE_POWER_HPP
