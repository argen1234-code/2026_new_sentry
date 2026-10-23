/**
 ******************************************************************************
 * @file    bsp_spi.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   SPI BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_spi.c。
 * @note    C 文件应包含 Src/bsp_spi.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_SPI_HPP
#define PRINTK_BSP_SPI_HPP

#include "Src/bsp_spi.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Spi {

/** @brief SPI 标识。 */
enum class Id {
    Spi2 = BSP_SPI_ID_SPI2,    /**< SPI2。 */
    Spi6 = BSP_SPI_ID_SPI6,    /**< SPI6。 */
};

/** @brief SPI 传输模式。 */
using Mode = bsp_spi_mode_t;

/** @brief SPI 事件类型。 */
using Event = bsp_spi_event_t;

/** @brief SPI 事件回调。 */
using Callback = bsp_spi_event_callback_t;

/** @brief SPI 传输事务。 */
using Transfer = bsp_spi_transfer_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 SPI BSP。 */
inline void init()
{
    bsp_spi_init();
}

/** @brief 获取 SPI HAL 句柄。 */
inline SPI_HandleTypeDef *handle(Id id)
{
    return bsp_spi_handle(static_cast<bsp_spi_id_t>(id));
}

/** @brief 根据外设实例反查 SPI HAL 句柄。 */
inline SPI_HandleTypeDef *inst(SPI_TypeDef *instance)
{
    return bsp_spi_from_instance(instance);
}

/** @brief 判断 SPI 句柄是否由 BSP 管理。 */
inline bool supported(SPI_HandleTypeDef *hspi)
{
    return bsp_spi_is_supported(hspi) != 0U;
}

/** @brief 执行 SPI 传输事务。 */
inline HAL_StatusTypeDef transfer(SPI_HandleTypeDef *hspi, const Transfer &xfer)
{
    return bsp_spi_transfer(hspi, &xfer);
}

/** @brief 中止 SPI 传输。 */
inline HAL_StatusTypeDef abort(SPI_HandleTypeDef *hspi)
{
    return bsp_spi_abort(hspi);
}

/** @brief 以中断方式中止 SPI 传输。 */
inline HAL_StatusTypeDef abort_it(SPI_HandleTypeDef *hspi)
{
    return bsp_spi_abort_it(hspi);
}

/** @brief 获取 SPI HAL 状态。 */
inline HAL_SPI_StateTypeDef state(SPI_HandleTypeDef *hspi)
{
    return bsp_spi_get_state(hspi);
}

/** @brief 获取 SPI 错误码。 */
inline uint32_t error(SPI_HandleTypeDef *hspi)
{
    return bsp_spi_get_error(hspi);
}

/** @brief 绑定 SPI 事件回调。 */
inline HAL_StatusTypeDef on(SPI_HandleTypeDef *hspi, Event event, Callback callback, void *user)
{
    return bsp_spi_on(hspi, event, callback, user);
}

/**
 * @brief SPI 编译期端口。
 *
 * @tparam Bus SPI 标识。
 */
template<Id Bus>
struct Port;

/** @brief SPI2 编译期端口。 */
template<>
struct Port<Id::Spi2> {
    /** @brief 获取 SPI HAL 句柄。 */
    static SPI_HandleTypeDef *handle()
    {
        return Bsp::Spi::handle(Id::Spi2);
    }
};

/** @brief SPI6 编译期端口。 */
template<>
struct Port<Id::Spi6> {
    /** @brief 获取 SPI HAL 句柄。 */
    static SPI_HandleTypeDef *handle()
    {
        return Bsp::Spi::handle(Id::Spi6);
    }
};

/** @brief 通过编译期端口执行 SPI 传输事务。 */
template<Id Bus>
inline HAL_StatusTypeDef transfer(const Transfer &xfer)
{
    return transfer(Port<Bus>::handle(), xfer);
}

/** @brief 通过编译期端口中止 SPI 传输。 */
template<Id Bus>
inline HAL_StatusTypeDef abort()
{
    return abort(Port<Bus>::handle());
}

/** @brief 通过编译期端口获取 SPI 状态。 */
template<Id Bus>
inline HAL_SPI_StateTypeDef state()
{
    return state(Port<Bus>::handle());
}

/** @brief 通过编译期端口绑定 SPI 事件回调。 */
template<Id Bus>
inline HAL_StatusTypeDef on(Event event, Callback callback, void *user)
{
    return on(Port<Bus>::handle(), event, callback, user);
}

} // namespace Bsp::Spi

#endif // PRINTK_BSP_SPI_HPP
