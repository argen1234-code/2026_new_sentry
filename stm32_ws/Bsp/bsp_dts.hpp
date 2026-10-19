/**
 ******************************************************************************
 * @file    bsp_dts.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   DTS BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_dts.c。
 * @note    C 文件应包含 Src/bsp_dts.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_DTS_HPP
#define PRINTK_BSP_DTS_HPP

#include "Src/bsp_dts.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Dts {

/**
 * @brief DTS 标识。
 */
enum class Id {
    Dts = BSP_DTS_ID_DTS,    /**< DTS。 */
};

/** @brief DTS 事件类型。 */
using Event = bsp_dts_event_t;

/** @brief DTS 事件回调。 */
using Callback = bsp_dts_event_callback_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 DTS BSP。 */
inline void init()
{
    bsp_dts_init();
}

/** @brief 获取 DTS HAL 句柄。 */
inline DTS_HandleTypeDef *handle(Id id)
{
    return bsp_dts_handle(static_cast<bsp_dts_id_t>(id));
}

/** @brief 根据外设实例反查 DTS HAL 句柄。 */
inline DTS_HandleTypeDef *inst(DTS_TypeDef *instance)
{
    return bsp_dts_from_instance(instance);
}

/** @brief 判断 DTS 句柄是否由 BSP 管理。 */
inline bool supported(DTS_HandleTypeDef *hdts)
{
    return bsp_dts_is_supported(hdts) != 0U;
}

/** @brief 启动 DTS 同步测量。 */
inline HAL_StatusTypeDef start(DTS_HandleTypeDef *hdts)
{
    return bsp_dts_start(hdts);
}

/** @brief 停止 DTS 同步测量。 */
inline HAL_StatusTypeDef stop(DTS_HandleTypeDef *hdts)
{
    return bsp_dts_stop(hdts);
}

/** @brief 读取 DTS 原始温度。 */
inline HAL_StatusTypeDef temp(DTS_HandleTypeDef *hdts, int32_t *out)
{
    return bsp_dts_get_temperature(hdts, out);
}

/** @brief 以中断方式启动 DTS 测量。 */
inline HAL_StatusTypeDef start_it(DTS_HandleTypeDef *hdts)
{
    return bsp_dts_start_it(hdts);
}

/** @brief 停止 DTS 中断测量。 */
inline HAL_StatusTypeDef stop_it(DTS_HandleTypeDef *hdts)
{
    return bsp_dts_stop_it(hdts);
}

/** @brief 转发 DTS 中断处理。 */
inline void irq(DTS_HandleTypeDef *hdts)
{
    bsp_dts_irq_handler(hdts);
}

/** @brief 获取 DTS HAL 状态。 */
inline HAL_DTS_StateTypeDef state(DTS_HandleTypeDef *hdts)
{
    return bsp_dts_get_state(hdts);
}

/** @brief 绑定 DTS 事件回调。 */
inline HAL_StatusTypeDef on(DTS_HandleTypeDef *hdts, Event event, Callback callback, void *user)
{
    return bsp_dts_on(hdts, event, callback, user);
}

/**
 * @brief DTS 编译期端口。
 *
 * @tparam Unit DTS 标识。
 */
template<Id Unit>
struct Port;

/** @brief DTS 编译期端口。 */
template<>
struct Port<Id::Dts> {
    /** @brief 获取 DTS HAL 句柄。 */
    static DTS_HandleTypeDef *handle()
    {
        return Bsp::Dts::handle(Id::Dts);
    }
};

/** @brief 通过编译期端口启动同步测量。 */
template<Id Unit>
inline HAL_StatusTypeDef start()
{
    return start(Port<Unit>::handle());
}

/** @brief 通过编译期端口停止同步测量。 */
template<Id Unit>
inline HAL_StatusTypeDef stop()
{
    return stop(Port<Unit>::handle());
}

/** @brief 通过编译期端口读取温度。 */
template<Id Unit>
inline HAL_StatusTypeDef temp(int32_t *out)
{
    return temp(Port<Unit>::handle(), out);
}

/** @brief 通过编译期端口绑定事件回调。 */
template<Id Unit>
inline HAL_StatusTypeDef on(Event event, Callback callback, void *user)
{
    return on(Port<Unit>::handle(), event, callback, user);
}

} // namespace Bsp::Dts

#endif // PRINTK_BSP_DTS_HPP
