/**
 ******************************************************************************
 * @file    bsp_tim.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   TIM BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_tim.c。
 * @note    C 文件应包含 Src/bsp_tim.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_TIM_HPP
#define PRINTK_BSP_TIM_HPP

#include "Src/bsp_tim.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Tim {

/** @brief TIM 标识。 */
enum class Id {
    Tim1 = BSP_TIM_ID_TIM1,      /**< TIM1。 */
    Tim2 = BSP_TIM_ID_TIM2,      /**< TIM2。 */
    Tim3 = BSP_TIM_ID_TIM3,      /**< TIM3。 */
    Tim12 = BSP_TIM_ID_TIM12,    /**< TIM12。 */
    Tim24 = BSP_TIM_ID_TIM24,    /**< TIM24。 */
};

/** @brief TIM 启停模式。 */
using Mode = bsp_tim_mode_t;

/** @brief TIM 事件类型。 */
using Event = bsp_tim_event_t;

/** @brief TIM 事件回调。 */
using Callback = bsp_tim_event_callback_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 TIM BSP。 */
inline void init()
{
    bsp_tim_init();
}

/** @brief 获取 TIM HAL 句柄。 */
inline TIM_HandleTypeDef *handle(Id id)
{
    return bsp_tim_handle(static_cast<bsp_tim_id_t>(id));
}

/** @brief 根据外设实例反查 TIM HAL 句柄。 */
inline TIM_HandleTypeDef *inst(TIM_TypeDef *instance)
{
    return bsp_tim_from_instance(instance);
}

/** @brief 判断 TIM 句柄是否由 BSP 管理。 */
inline bool supported(TIM_HandleTypeDef *htim)
{
    return bsp_tim_is_supported(htim) != 0U;
}

/** @brief 启动 TIM。 */
inline HAL_StatusTypeDef start(TIM_HandleTypeDef *htim, Mode mode)
{
    return bsp_tim_start(htim, mode);
}

/** @brief 停止 TIM。 */
inline HAL_StatusTypeDef stop(TIM_HandleTypeDef *htim, Mode mode)
{
    return bsp_tim_stop(htim, mode);
}

/** @brief 获取 TIM 计数值。 */
inline uint32_t counter(TIM_HandleTypeDef *htim)
{
    return bsp_tim_get_counter(htim);
}

/** @brief 设置 TIM 计数值。 */
inline void counter(TIM_HandleTypeDef *htim, uint32_t value)
{
    bsp_tim_set_counter(htim, value);
}

/** @brief 获取 TIM 自动重装值。 */
inline uint32_t autoreload(TIM_HandleTypeDef *htim)
{
    return bsp_tim_get_autoreload(htim);
}

/** @brief 设置 TIM 自动重装值。 */
inline void autoreload(TIM_HandleTypeDef *htim, uint32_t value)
{
    bsp_tim_set_autoreload(htim, value);
}

/** @brief 获取 TIM 比较值。 */
inline uint32_t compare(TIM_HandleTypeDef *htim, uint32_t channel)
{
    return bsp_tim_get_compare(htim, channel);
}

/** @brief 设置 TIM 比较值。 */
inline void compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t value)
{
    bsp_tim_set_compare(htim, channel, value);
}

/** @brief 启动 TIM PWM 输出。 */
inline HAL_StatusTypeDef start_pwm(TIM_HandleTypeDef *htim, uint32_t channel)
{
    return bsp_tim_start_pwm(htim, channel);
}

/** @brief 停止 TIM PWM 输出。 */
inline HAL_StatusTypeDef stop_pwm(TIM_HandleTypeDef *htim, uint32_t channel)
{
    return bsp_tim_stop_pwm(htim, channel);
}

/** @brief 绑定 TIM 事件回调。 */
inline HAL_StatusTypeDef on(TIM_HandleTypeDef *htim, Event event, Callback callback, void *user)
{
    return bsp_tim_on(htim, event, callback, user);
}

/** @brief 分发 TIM 周期中断事件。 */
inline void dispatch(TIM_HandleTypeDef *htim)
{
    bsp_tim_dispatch_period_elapsed(htim);
}

/**
 * @brief TIM 编译期端口。
 *
 * @tparam Timer TIM 标识。
 */
template<Id Timer>
struct Port {
    /** @brief 获取 TIM HAL 句柄。 */
    static TIM_HandleTypeDef *handle()
    {
        return Bsp::Tim::handle(Timer);
    }

    /** @brief 启动 TIM。 */
    static HAL_StatusTypeDef start(Mode mode)
    {
        return Bsp::Tim::start(handle(), mode);
    }

    /** @brief 停止 TIM。 */
    static HAL_StatusTypeDef stop(Mode mode)
    {
        return Bsp::Tim::stop(handle(), mode);
    }

    /** @brief 获取 TIM 计数值。 */
    static uint32_t counter()
    {
        return Bsp::Tim::counter(handle());
    }

    /** @brief 设置 TIM 计数值。 */
    static void counter(uint32_t value)
    {
        Bsp::Tim::counter(handle(), value);
    }

    /** @brief 获取 TIM 自动重装值。 */
    static uint32_t autoreload()
    {
        return Bsp::Tim::autoreload(handle());
    }

    /** @brief 设置 TIM 自动重装值。 */
    static void autoreload(uint32_t value)
    {
        Bsp::Tim::autoreload(handle(), value);
    }

    /** @brief 获取 TIM 比较值。 */
    static uint32_t compare(uint32_t channel)
    {
        return Bsp::Tim::compare(handle(), channel);
    }

    /** @brief 设置 TIM 比较值。 */
    static void compare(uint32_t channel, uint32_t value)
    {
        Bsp::Tim::compare(handle(), channel, value);
    }

    /** @brief 启动 TIM PWM 输出。 */
    static HAL_StatusTypeDef start_pwm(uint32_t channel)
    {
        return Bsp::Tim::start_pwm(handle(), channel);
    }

    /** @brief 停止 TIM PWM 输出。 */
    static HAL_StatusTypeDef stop_pwm(uint32_t channel)
    {
        return Bsp::Tim::stop_pwm(handle(), channel);
    }

    /** @brief 绑定 TIM 事件回调。 */
    static HAL_StatusTypeDef on(Event event, Callback callback, void *user)
    {
        return Bsp::Tim::on(handle(), event, callback, user);
    }
};

} // namespace Bsp::Tim

#endif // PRINTK_BSP_TIM_HPP
