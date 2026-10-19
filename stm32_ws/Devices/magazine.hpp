/**
 ******************************************************************************
 * @file    magazine.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   弹仓设备 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/magazine.c。
 * @note    C 文件应包含 Src/magazine.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_MAGAZINE_HPP
#define PRINTK_DEVICE_MAGAZINE_HPP

#include "Src/magazine.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Magazine {

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 弹仓关闭位置比较值。 */
inline constexpr uint32_t ClosedCompare = MAGAZINE_COMPARE_CLOSED;

/** @brief 弹仓打开位置比较值。 */
inline constexpr uint32_t OpenCompare = MAGAZINE_COMPARE_OPEN;

/** @brief 初始化弹仓舵机。 */
inline HAL_StatusTypeDef init()
{
    return magazine_init();
}

/** @brief 打开弹仓。 */
inline HAL_StatusTypeDef open()
{
    return magazine_open();
}

/** @brief 关闭弹仓。 */
inline HAL_StatusTypeDef close()
{
    return magazine_close();
}

/** @brief 设置弹仓舵机比较值。 */
inline HAL_StatusTypeDef set(uint32_t value)
{
    return magazine_set(value);
}

} // namespace Device::Magazine

#endif // PRINTK_DEVICE_MAGAZINE_HPP
