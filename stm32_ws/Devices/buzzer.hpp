/**
 ******************************************************************************
 * @file    buzzer.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   蜂鸣器设备 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/buzzer.c。
 * @note    C 文件应包含 Src/buzzer.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_BUZZER_HPP
#define PRINTK_DEVICE_BUZZER_HPP

#include "Src/buzzer.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Buzzer {

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 默认蜂鸣器占空比，单位为千分比。 */
inline constexpr uint16_t DefaultDuty = BUZZER_DUTY_DEFAULT;

/** @brief 初始化蜂鸣器。 */
inline HAL_StatusTypeDef init()
{
    return buzzer_init();
}

/** @brief 按默认占空比启动蜂鸣器。 */
inline HAL_StatusTypeDef start(uint32_t freq_hz)
{
    return buzzer_start(freq_hz);
}

/** @brief 按指定占空比启动蜂鸣器。 */
inline HAL_StatusTypeDef start(uint32_t freq_hz, uint16_t duty_permille)
{
    return buzzer_start_duty(freq_hz, duty_permille);
}

/** @brief 关闭蜂鸣器。 */
inline HAL_StatusTypeDef off()
{
    return buzzer_off();
}

} // namespace Device::Buzzer

#endif // PRINTK_DEVICE_BUZZER_HPP
