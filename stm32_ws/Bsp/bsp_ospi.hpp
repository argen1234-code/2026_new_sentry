/**
 ******************************************************************************
 * @file    bsp_ospi.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   OSPI BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_ospi.c。
 * @note    C 文件应包含 Src/bsp_ospi.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_OSPI_HPP
#define PRINTK_BSP_OSPI_HPP

#include "Src/bsp_ospi.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Ospi {

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 默认 OSPI 超时时间，单位为毫秒。 */
inline constexpr uint32_t DefaultTimeout = BSP_OSPI_TIMEOUT_DEFAULT_MS;

/** @brief OSPI bus 标识。 */
enum class Id {
    Flash = BSP_OSPI_ID_FLASH,    /**< 板级外部 Flash bus。 */
};

/** @brief OSPI BSP 状态。 */
using Status = bsp_ospi_status_t;

/** @brief OSPI 命令操作类型。 */
using Operation = bsp_ospi_operation_type_t;

/** @brief OSPI Flash 标识。 */
using FlashId = bsp_ospi_flash_id_t;

/** @brief OSPI 线宽模式。 */
using Line = bsp_ospi_line_mode_t;

/** @brief OSPI 字段宽度。 */
using Width = bsp_ospi_bus_width_t;

/** @brief OSPI DTR 模式。 */
using Dtr = bsp_ospi_dtr_mode_t;

/** @brief OSPI DQS 模式。 */
using Dqs = bsp_ospi_dqs_mode_t;

/** @brief OSPI SIOO 模式。 */
using Sioo = bsp_ospi_sioo_mode_t;

/** @brief OSPI 自动轮询匹配模式。 */
using Match = bsp_ospi_match_mode_t;

/** @brief OSPI 自动停止模式。 */
using AutoStop = bsp_ospi_automatic_stop_t;

/** @brief OSPI timeout counter 模式。 */
using Timeout = bsp_ospi_timeout_activation_t;

/** @brief OSPI bus 上下文。 */
using Bus = bsp_ospi_bus_t;

/** @brief OSPI regular command。 */
using Cmd = bsp_ospi_cmd_t;

/** @brief OSPI 自动轮询配置。 */
using Polling = bsp_ospi_polling_t;

/** @brief OSPI memory mapped 配置。 */
using Mmap = bsp_ospi_memory_mapped_cfg_t;

/** @brief 获取 OSPI bus。 */
inline Bus *bus(Id id)
{
    return bsp_ospi_bus(static_cast<bsp_ospi_id_t>(id));
}

/** @brief 发送 OSPI regular command。 */
inline Status command(Bus *bus, const Cmd &cmd, uint32_t timeout_ms = DefaultTimeout)
{
    return bsp_ospi_command(bus, &cmd, timeout_ms);
}

/** @brief 发送 OSPI 数据。 */
inline Status tx(Bus *bus, const uint8_t *data, size_t length, uint32_t timeout_ms = DefaultTimeout)
{
    return bsp_ospi_transmit(bus, data, length, timeout_ms);
}

/** @brief 接收 OSPI 数据。 */
inline Status rx(Bus *bus, uint8_t *data, size_t length, uint32_t timeout_ms = DefaultTimeout)
{
    return bsp_ospi_receive(bus, data, length, timeout_ms);
}

/** @brief 执行 OSPI 自动轮询。 */
inline Status poll(Bus *bus, const Polling &polling, uint32_t timeout_ms = DefaultTimeout)
{
    return bsp_ospi_auto_polling(bus, &polling, timeout_ms);
}

/** @brief 进入 OSPI memory mapped 模式。 */
inline Status mmap(Bus *bus, const Mmap &cfg)
{
    return bsp_ospi_memory_mapped(bus, &cfg);
}

/** @brief 发送命令并执行 OSPI 自动轮询。 */
inline Status poll_cmd(Bus *bus,
                       const Cmd &cmd,
                       const Polling &polling,
                       uint32_t timeout_ms = DefaultTimeout)
{
    return bsp_ospi_poll_command(bus, &cmd, &polling, timeout_ms);
}

/** @brief 发送命令并进入 OSPI memory mapped 模式。 */
inline Status enter_mmap(Bus *bus,
                         const Cmd &cmd,
                         const Mmap &cfg,
                         uint32_t timeout_ms = DefaultTimeout)
{
    return bsp_ospi_enter_memory_mapped(bus, &cmd, &cfg, timeout_ms);
}

/**
 * @brief OSPI 编译期端口。
 *
 * @tparam BusId OSPI bus 标识。
 */
template<Id BusId>
struct Port {
    /** @brief 获取 OSPI bus。 */
    static Bus *bus()
    {
        return Bsp::Ospi::bus(BusId);
    }

    /** @brief 发送 OSPI regular command。 */
    static Status command(const Cmd &cmd, uint32_t timeout_ms = DefaultTimeout)
    {
        return Bsp::Ospi::command(bus(), cmd, timeout_ms);
    }

    /** @brief 发送 OSPI 数据。 */
    static Status tx(const uint8_t *data, size_t length, uint32_t timeout_ms = DefaultTimeout)
    {
        return Bsp::Ospi::tx(bus(), data, length, timeout_ms);
    }

    /** @brief 接收 OSPI 数据。 */
    static Status rx(uint8_t *data, size_t length, uint32_t timeout_ms = DefaultTimeout)
    {
        return Bsp::Ospi::rx(bus(), data, length, timeout_ms);
    }

    /** @brief 执行 OSPI 自动轮询。 */
    static Status poll(const Polling &polling, uint32_t timeout_ms = DefaultTimeout)
    {
        return Bsp::Ospi::poll(bus(), polling, timeout_ms);
    }

    /** @brief 进入 OSPI memory mapped 模式。 */
    static Status mmap(const Mmap &cfg)
    {
        return Bsp::Ospi::mmap(bus(), cfg);
    }

    /** @brief 发送命令并执行 OSPI 自动轮询。 */
    static Status poll_cmd(const Cmd &cmd,
                           const Polling &polling,
                           uint32_t timeout_ms = DefaultTimeout)
    {
        return Bsp::Ospi::poll_cmd(bus(), cmd, polling, timeout_ms);
    }

    /** @brief 发送命令并进入 OSPI memory mapped 模式。 */
    static Status enter_mmap(const Cmd &cmd,
                             const Mmap &cfg,
                             uint32_t timeout_ms = DefaultTimeout)
    {
        return Bsp::Ospi::enter_mmap(bus(), cmd, cfg, timeout_ms);
    }
};

} // namespace Bsp::Ospi

#endif // PRINTK_BSP_OSPI_HPP
