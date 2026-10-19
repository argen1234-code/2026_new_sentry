/**
 ******************************************************************************
 * @file    bsp_cdc.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   CDC BSP 的 C++ 包装。
 *
 * @note    本文件只做 C++ 包装，运行时实现全部位于 Src/bsp_cdc.c。
 * @note    C 文件应包含 Src/bsp_cdc.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_CDC_HPP
#define PRINTK_BSP_CDC_HPP

#include "Src/bsp_cdc.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Cdc {

/** @brief CDC 接收回调。 */
using RxCallback = bsp_cdc_rx_fn_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 CDC BSP。 */
inline HAL_StatusTypeDef init()
{
    return bsp_cdc_init();
}

/** @brief 写入 CDC 调试数据。 */
inline HAL_StatusTypeDef write(const uint8_t *data, uint16_t length)
{
    return bsp_cdc_write(data, length);
}

/** @brief 写入 CDC 原始二进制数据。 */
inline HAL_StatusTypeDef write_raw(const uint8_t *data, uint16_t length)
{
    return bsp_cdc_write_raw(data, length);
}

/** @brief 注册 CDC 接收回调。 */
inline void on_rx(RxCallback callback, void *user)
{
    bsp_cdc_on_rx(callback, user);
}

/** @brief 判断 CDC 是否已可用。 */
inline bool ready()
{
    return bsp_cdc_is_ready() != 0U;
}

/** @brief 判断 CDC 发送端点是否空闲。 */
inline bool tx_idle()
{
    return bsp_cdc_tx_idle() != 0U;
}

} // namespace Bsp::Cdc

#endif // PRINTK_BSP_CDC_HPP
