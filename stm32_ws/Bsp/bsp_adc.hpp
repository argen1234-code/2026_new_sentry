/**
 ******************************************************************************
 * @file    bsp_adc.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   ADC BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_adc.c。
 * @note    C 文件应包含 Src/bsp_adc.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_ADC_HPP
#define PRINTK_BSP_ADC_HPP

#include "Src/bsp_adc.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Adc {

/**
 * @brief ADC 标识。
 */
enum class Id {
    Adc1 = BSP_ADC_ID_ADC1,    /**< ADC1。 */
    Adc3 = BSP_ADC_ID_ADC3,    /**< ADC3。 */
};

/** @brief ADC 启停模式。 */
using Mode = bsp_adc_mode_t;

/** @brief ADC 事件类型。 */
using Event = bsp_adc_event_t;

/** @brief ADC 事件回调。 */
using Callback = bsp_adc_event_callback_t;

/** @brief ADC 启动事务配置。 */
using Start = bsp_adc_start_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 ADC BSP。 */
inline void init()
{
    bsp_adc_init();
}

/** @brief 获取 ADC HAL 句柄。 */
inline ADC_HandleTypeDef *handle(Id id)
{
    return bsp_adc_handle(static_cast<bsp_adc_id_t>(id));
}

/** @brief 根据外设实例反查 ADC HAL 句柄。 */
inline ADC_HandleTypeDef *inst(ADC_TypeDef *instance)
{
    return bsp_adc_from_instance(instance);
}

/** @brief 判断 ADC 句柄是否由 BSP 管理。 */
inline bool supported(ADC_HandleTypeDef *hadc)
{
    return bsp_adc_is_supported(hadc) != 0U;
}

/** @brief 启动 ADC 自校准。 */
inline HAL_StatusTypeDef calibrate(ADC_HandleTypeDef *hadc, uint32_t mode, uint32_t single_diff)
{
    return bsp_adc_calibration_start(hadc, mode, single_diff);
}

/** @brief 启动 ADC 转换。 */
inline HAL_StatusTypeDef start(ADC_HandleTypeDef *hadc, const Start &cfg)
{
    return bsp_adc_start(hadc, &cfg);
}

/** @brief 停止 ADC 转换。 */
inline HAL_StatusTypeDef stop(ADC_HandleTypeDef *hadc, Mode mode)
{
    return bsp_adc_stop(hadc, mode);
}

/** @brief 等待 ADC 转换完成。 */
inline HAL_StatusTypeDef poll(ADC_HandleTypeDef *hadc, uint32_t timeout)
{
    return bsp_adc_poll_for_conversion(hadc, timeout);
}

/** @brief 获取 ADC 当前转换值。 */
inline uint32_t value(ADC_HandleTypeDef *hadc)
{
    return bsp_adc_get_value(hadc);
}

/** @brief 配置 ADC 普通通道。 */
inline HAL_StatusTypeDef channel(ADC_HandleTypeDef *hadc, const ADC_ChannelConfTypeDef &cfg)
{
    return bsp_adc_config_channel(hadc, &cfg);
}

/** @brief 绑定 ADC 事件回调。 */
inline HAL_StatusTypeDef on(ADC_HandleTypeDef *hadc, Event event, Callback callback, void *user)
{
    return bsp_adc_on(hadc, event, callback, user);
}

/**
 * @brief ADC 编译期端口。
 *
 * @tparam Bus ADC 标识。
 */
template<Id Bus>
struct Port;

/** @brief ADC1 编译期端口。 */
template<>
struct Port<Id::Adc1> {
    /** @brief 获取 ADC HAL 句柄。 */
    static ADC_HandleTypeDef *handle()
    {
        return Bsp::Adc::handle(Id::Adc1);
    }
};

/** @brief ADC3 编译期端口。 */
template<>
struct Port<Id::Adc3> {
    /** @brief 获取 ADC HAL 句柄。 */
    static ADC_HandleTypeDef *handle()
    {
        return Bsp::Adc::handle(Id::Adc3);
    }
};

/** @brief 通过编译期端口启动转换。 */
template<Id Bus>
inline HAL_StatusTypeDef start(const Start &cfg)
{
    return start(Port<Bus>::handle(), cfg);
}

/** @brief 通过编译期端口停止转换。 */
template<Id Bus>
inline HAL_StatusTypeDef stop(Mode mode)
{
    return stop(Port<Bus>::handle(), mode);
}

/** @brief 通过编译期端口等待转换完成。 */
template<Id Bus>
inline HAL_StatusTypeDef poll(uint32_t timeout)
{
    return poll(Port<Bus>::handle(), timeout);
}

/** @brief 通过编译期端口读取转换值。 */
template<Id Bus>
inline uint32_t value()
{
    return value(Port<Bus>::handle());
}

/** @brief 通过编译期端口绑定事件回调。 */
template<Id Bus>
inline HAL_StatusTypeDef on(Event event, Callback callback, void *user)
{
    return on(Port<Bus>::handle(), event, callback, user);
}

} // namespace Bsp::Adc

#endif // PRINTK_BSP_ADC_HPP
