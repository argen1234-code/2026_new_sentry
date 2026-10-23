/**
 ******************************************************************************
 * @file    bsp_uart.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   UART BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_uart.c。
 * @note    C 文件应包含 Src/bsp_uart.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_UART_HPP
#define PRINTK_BSP_UART_HPP

#include "Src/bsp_uart.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Uart {

/** @brief UART 标识。 */
enum class Id {
    Usart2 = BSP_UART_ID_USART2,      /**< USART2。 */
    Usart1 = BSP_UART_ID_USART1,      /**< USART1。 */
    Usart3 = BSP_UART_ID_USART3,      /**< USART3。 */
    Uart5 = BSP_UART_ID_UART5,        /**< UART5。 */
    Uart7 = BSP_UART_ID_UART7,        /**< UART7。 */
    Uart8 = BSP_UART_ID_UART8,        /**< UART8。 */
    Uart9 = BSP_UART_ID_UART9,        /**< UART9。 */
    Usart10 = BSP_UART_ID_USART10,    /**< USART10。 */
};

/** @brief UART 传输模式。 */
using Mode = bsp_uart_mode_t;

/** @brief UART 事件类型。 */
using Event = bsp_uart_event_t;

/** @brief UART 普通事件回调。 */
using Callback = bsp_uart_event_callback_t;

/** @brief UART idle 接收回调。 */
using IdleCallback = bsp_uart_rx_idle_callback_t;

/** @brief UART 发送事务。 */
using Tx = bsp_uart_tx_t;

/** @brief UART 接收事务。 */
using Rx = bsp_uart_rx_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 UART BSP。 */
inline void init()
{
    bsp_uart_init();
}

/** @brief 获取 UART HAL 句柄。 */
inline UART_HandleTypeDef *handle(Id id)
{
    return bsp_uart_handle(static_cast<bsp_uart_id_t>(id));
}

/** @brief 根据外设实例反查 UART HAL 句柄。 */
inline UART_HandleTypeDef *inst(USART_TypeDef *instance)
{
    return bsp_uart_from_instance(instance);
}

/** @brief 判断 UART 句柄是否由 BSP 管理。 */
inline bool supported(UART_HandleTypeDef *huart)
{
    return bsp_uart_is_supported(huart) != 0U;
}

/** @brief 启动 UART 发送事务。 */
inline HAL_StatusTypeDef tx(UART_HandleTypeDef *huart, const Tx &tx)
{
    return bsp_uart_tx(huart, &tx);
}

/** @brief 启动 UART 接收事务。 */
inline HAL_StatusTypeDef rx(UART_HandleTypeDef *huart, const Rx &rx)
{
    return bsp_uart_rx(huart, &rx);
}

/** @brief 中止 UART 接收。 */
inline HAL_StatusTypeDef abort_rx(UART_HandleTypeDef *huart)
{
    return bsp_uart_abort_rx(huart);
}

/** @brief 中止 UART 发送。 */
inline HAL_StatusTypeDef abort_tx(UART_HandleTypeDef *huart)
{
    return bsp_uart_abort_tx(huart);
}

/** @brief 中止 UART 全部传输。 */
inline HAL_StatusTypeDef abort(UART_HandleTypeDef *huart)
{
    return bsp_uart_abort(huart);
}

/** @brief 绑定 UART 普通事件回调。 */
inline HAL_StatusTypeDef on(UART_HandleTypeDef *huart, Event event, Callback callback, void *user)
{
    return bsp_uart_on(huart, event, callback, user);
}

/** @brief 绑定 UART idle 接收回调。 */
inline HAL_StatusTypeDef on_idle(UART_HandleTypeDef *huart, IdleCallback callback, void *user)
{
    return bsp_uart_on_rx_idle(huart, callback, user);
}

/**
 * @brief UART 编译期端口。
 *
 * @tparam Bus UART 标识。
 */
template<Id Bus>
struct Port {
    /** @brief 获取 UART HAL 句柄。 */
    static UART_HandleTypeDef *handle()
    {
        return Bsp::Uart::handle(Bus);
    }

    /** @brief 启动 UART 发送事务。 */
    static HAL_StatusTypeDef tx(const Tx &tx)
    {
        return Bsp::Uart::tx(handle(), tx);
    }

    /** @brief 启动 UART 接收事务。 */
    static HAL_StatusTypeDef rx(const Rx &rx)
    {
        return Bsp::Uart::rx(handle(), rx);
    }

    /** @brief 中止 UART 接收。 */
    static HAL_StatusTypeDef abort_rx()
    {
        return Bsp::Uart::abort_rx(handle());
    }

    /** @brief 中止 UART 发送。 */
    static HAL_StatusTypeDef abort_tx()
    {
        return Bsp::Uart::abort_tx(handle());
    }

    /** @brief 中止 UART 全部传输。 */
    static HAL_StatusTypeDef abort()
    {
        return Bsp::Uart::abort(handle());
    }

    /** @brief 绑定 UART 普通事件回调。 */
    static HAL_StatusTypeDef on(Event event, Callback callback, void *user)
    {
        return Bsp::Uart::on(handle(), event, callback, user);
    }

    /** @brief 绑定 UART idle 接收回调。 */
    static HAL_StatusTypeDef on_idle(IdleCallback callback, void *user)
    {
        return Bsp::Uart::on_idle(handle(), callback, user);
    }
};

} // namespace Bsp::Uart

#endif // PRINTK_BSP_UART_HPP
