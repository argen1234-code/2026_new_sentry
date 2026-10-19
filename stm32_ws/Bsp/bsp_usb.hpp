/**
 ******************************************************************************
 * @file    bsp_usb.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   USB BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_usb.c。
 * @note    C 文件应包含 Src/bsp_usb.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_USB_HPP
#define PRINTK_BSP_USB_HPP

#include "Src/bsp_usb.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Usb {

/** @brief USB PCD 标识。 */
enum class Id {
    Hs = BSP_USB_ID_HS,    /**< USB HS。 */
};

/** @brief USB 事件类型。 */
using Event = bsp_usb_event_t;

/** @brief USB 事件回调。 */
using Callback = bsp_usb_event_callback_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 USB BSP。 */
inline void init()
{
    bsp_usb_init();
}

/** @brief 获取 USB PCD HAL 句柄。 */
inline PCD_HandleTypeDef *handle(Id id)
{
    return bsp_usb_handle(static_cast<bsp_usb_id_t>(id));
}

/** @brief 根据外设实例反查 USB PCD HAL 句柄。 */
inline PCD_HandleTypeDef *inst(USB_OTG_GlobalTypeDef *instance)
{
    return bsp_usb_from_instance(instance);
}

/** @brief 判断 USB PCD 句柄是否由 BSP 管理。 */
inline bool supported(PCD_HandleTypeDef *hpcd)
{
    return bsp_usb_is_supported(hpcd) != 0U;
}

/** @brief 启动 USB PCD。 */
inline HAL_StatusTypeDef start(PCD_HandleTypeDef *hpcd)
{
    return bsp_usb_start(hpcd);
}

/** @brief 停止 USB PCD。 */
inline HAL_StatusTypeDef stop(PCD_HandleTypeDef *hpcd)
{
    return bsp_usb_stop(hpcd);
}

/** @brief 连接 USB 设备。 */
inline HAL_StatusTypeDef connect(PCD_HandleTypeDef *hpcd)
{
    return bsp_usb_dev_connect(hpcd);
}

/** @brief 断开 USB 设备。 */
inline HAL_StatusTypeDef disconnect(PCD_HandleTypeDef *hpcd)
{
    return bsp_usb_dev_disconnect(hpcd);
}

/** @brief 绑定 USB 事件回调。 */
inline HAL_StatusTypeDef on(PCD_HandleTypeDef *hpcd, Event event, Callback callback, void *user)
{
    return bsp_usb_on(hpcd, event, callback, user);
}

/**
 * @brief USB 编译期端口。
 *
 * @tparam Unit USB PCD 标识。
 */
template<Id Unit>
struct Port;

/** @brief USB HS 编译期端口。 */
template<>
struct Port<Id::Hs> {
    /** @brief 获取 USB PCD HAL 句柄。 */
    static PCD_HandleTypeDef *handle()
    {
        return Bsp::Usb::handle(Id::Hs);
    }
};

/** @brief 通过编译期端口启动 USB PCD。 */
template<Id Unit>
inline HAL_StatusTypeDef start()
{
    return start(Port<Unit>::handle());
}

/** @brief 通过编译期端口停止 USB PCD。 */
template<Id Unit>
inline HAL_StatusTypeDef stop()
{
    return stop(Port<Unit>::handle());
}

/** @brief 通过编译期端口连接 USB 设备。 */
template<Id Unit>
inline HAL_StatusTypeDef connect()
{
    return connect(Port<Unit>::handle());
}

/** @brief 通过编译期端口断开 USB 设备。 */
template<Id Unit>
inline HAL_StatusTypeDef disconnect()
{
    return disconnect(Port<Unit>::handle());
}

/** @brief 通过编译期端口绑定 USB 事件回调。 */
template<Id Unit>
inline HAL_StatusTypeDef on(Event event, Callback callback, void *user)
{
    return on(Port<Unit>::handle(), event, callback, user);
}

} // namespace Bsp::Usb

#endif // PRINTK_BSP_USB_HPP
