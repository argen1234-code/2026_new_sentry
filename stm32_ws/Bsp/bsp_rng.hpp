/**
 ******************************************************************************
 * @file    bsp_rng.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   RNG BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_rng.c。
 * @note    C 文件应包含 Src/bsp_rng.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_RNG_HPP
#define PRINTK_BSP_RNG_HPP

#include "Src/bsp_rng.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Rng {

/** @brief RNG 标识。 */
enum class Id {
    Rng = BSP_RNG_ID_RNG,    /**< 板级 RNG。 */
};

/** @brief RNG 事件类型。 */
using Event = bsp_rng_event_t;

/** @brief RNG 普通事件回调。 */
using EventCallback = bsp_rng_event_callback_t;

/** @brief RNG 随机数就绪回调。 */
using ReadyCallback = bsp_rng_ready_callback_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 RNG BSP。 */
inline void init()
{
    bsp_rng_init();
}

/** @brief 获取 RNG HAL 句柄。 */
inline RNG_HandleTypeDef *handle(Id id)
{
    return bsp_rng_handle(static_cast<bsp_rng_id_t>(id));
}

/** @brief 根据外设实例反查 RNG HAL 句柄。 */
inline RNG_HandleTypeDef *inst(RNG_TypeDef *instance)
{
    return bsp_rng_from_instance(instance);
}

/** @brief 判断 RNG 句柄是否由 BSP 管理。 */
inline bool supported(RNG_HandleTypeDef *hrng)
{
    return bsp_rng_is_supported(hrng) != 0U;
}

/** @brief 阻塞生成随机数。 */
inline HAL_StatusTypeDef generate(RNG_HandleTypeDef *hrng, uint32_t *out)
{
    return bsp_rng_generate(hrng, out);
}

/** @brief 以中断方式生成随机数。 */
inline HAL_StatusTypeDef generate_it(RNG_HandleTypeDef *hrng)
{
    return bsp_rng_generate_it(hrng);
}

/** @brief 读取最近一次随机数。 */
inline uint32_t last(RNG_HandleTypeDef *hrng)
{
    return bsp_rng_read_last(hrng);
}

/** @brief 转发 RNG 中断处理。 */
inline void irq(RNG_HandleTypeDef *hrng)
{
    bsp_rng_irq_handler(hrng);
}

/** @brief 获取 RNG HAL 状态。 */
inline HAL_RNG_StateTypeDef state(RNG_HandleTypeDef *hrng)
{
    return bsp_rng_get_state(hrng);
}

/** @brief 获取 RNG 错误码。 */
inline uint32_t error(RNG_HandleTypeDef *hrng)
{
    return bsp_rng_get_error(hrng);
}

/** @brief 恢复 RNG seed error。 */
inline HAL_StatusTypeDef recover_seed(RNG_HandleTypeDef *hrng)
{
    return bsp_rng_recover_seed_error(hrng);
}

/** @brief 设置 RNG 配置。 */
inline HAL_StatusTypeDef cfg(RNG_HandleTypeDef *hrng, const RNG_ConfigTypeDef &cfg)
{
    return bsp_rng_set_config(hrng, &cfg);
}

/** @brief 读取 RNG 配置。 */
inline HAL_StatusTypeDef cfg(RNG_HandleTypeDef *hrng, RNG_ConfigTypeDef *out)
{
    return bsp_rng_get_config(hrng, out);
}

/** @brief 锁定 RNG 配置。 */
inline HAL_StatusTypeDef lock_cfg(RNG_HandleTypeDef *hrng)
{
    return bsp_rng_lock_config(hrng);
}

/** @brief 绑定 RNG 事件回调。 */
inline HAL_StatusTypeDef on(RNG_HandleTypeDef *hrng, Event event, void *callback, void *user)
{
    return bsp_rng_on(hrng, event, callback, user);
}

/**
 * @brief RNG 编译期端口。
 *
 * @tparam Unit RNG 标识。
 */
template<Id Unit>
struct Port;

/** @brief 板级 RNG 编译期端口。 */
template<>
struct Port<Id::Rng> {
    /** @brief 获取 RNG HAL 句柄。 */
    static RNG_HandleTypeDef *handle()
    {
        return Bsp::Rng::handle(Id::Rng);
    }
};

/** @brief 通过编译期端口阻塞生成随机数。 */
template<Id Unit>
inline HAL_StatusTypeDef generate(uint32_t *out)
{
    return generate(Port<Unit>::handle(), out);
}

/** @brief 通过编译期端口以中断方式生成随机数。 */
template<Id Unit>
inline HAL_StatusTypeDef generate_it()
{
    return generate_it(Port<Unit>::handle());
}

/** @brief 通过编译期端口读取最近一次随机数。 */
template<Id Unit>
inline uint32_t last()
{
    return last(Port<Unit>::handle());
}

/** @brief 通过编译期端口获取 RNG 状态。 */
template<Id Unit>
inline HAL_RNG_StateTypeDef state()
{
    return state(Port<Unit>::handle());
}

/** @brief 通过编译期端口绑定 RNG 事件回调。 */
template<Id Unit>
inline HAL_StatusTypeDef on(Event event, void *callback, void *user)
{
    return on(Port<Unit>::handle(), event, callback, user);
}

} // namespace Bsp::Rng

#endif // PRINTK_BSP_RNG_HPP
