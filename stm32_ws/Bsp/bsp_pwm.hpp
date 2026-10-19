/**
 ******************************************************************************
 * @file    bsp_pwm.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   PWM BSP 的 C++ 编译期包装。
 *
 * @note    PWM 的 C 接口为头文件内联实现，本文件只做语义包装和模板绑定。
 * @note    C 文件应包含 Src/bsp_pwm.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_PWM_HPP
#define PRINTK_BSP_PWM_HPP

#include "Src/bsp_pwm.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Pwm {

/** @brief PWM 通道描述。 */
using Channel = bsp_pwm_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 启动 PWM 输出。 */
inline HAL_StatusTypeDef start(const Channel &pwm)
{
    return bsp_pwm_start(&pwm);
}

/** @brief 停止 PWM 输出。 */
inline HAL_StatusTypeDef stop(const Channel &pwm)
{
    return bsp_pwm_stop(&pwm);
}

/** @brief 设置 PWM 比较值。 */
inline void compare(const Channel &pwm, uint32_t value)
{
    bsp_pwm_set_compare_value(&pwm, value);
}

/** @brief 设置 PWM 占空比千分比。 */
inline void duty(const Channel &pwm, uint16_t permille)
{
    bsp_pwm_set_duty_permille(&pwm, permille);
}

/** @brief 通过 TIM 句柄和通道设置比较值。 */
inline void compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t value)
{
    bsp_pwm_set_compare(htim, channel, value);
}

/**
 * @brief PWM 编译期端口。
 *
 * @tparam Pwm 具有外部链接的 PWM 通道描述对象。
 */
template<const Channel &Pwm>
struct Port {
    /** @brief 启动 PWM 输出。 */
    static HAL_StatusTypeDef start()
    {
        return Bsp::Pwm::start(Pwm);
    }

    /** @brief 停止 PWM 输出。 */
    static HAL_StatusTypeDef stop()
    {
        return Bsp::Pwm::stop(Pwm);
    }

    /** @brief 设置 PWM 比较值。 */
    static void compare(uint32_t value)
    {
        Bsp::Pwm::compare(Pwm, value);
    }

    /** @brief 设置 PWM 占空比千分比。 */
    static void duty(uint16_t permille)
    {
        Bsp::Pwm::duty(Pwm, permille);
    }
};

} // namespace Bsp::Pwm

#endif // PRINTK_BSP_PWM_HPP
