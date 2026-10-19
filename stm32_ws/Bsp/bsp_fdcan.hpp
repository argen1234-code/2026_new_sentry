/**
 ******************************************************************************
 * @file    bsp_fdcan.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   FDCAN BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_fdcan.c。
 * @note    C 文件应包含 Src/bsp_fdcan.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_FDCAN_HPP
#define PRINTK_BSP_FDCAN_HPP

#include "Src/bsp_fdcan.h"
#include "robot_board.h"

/**
 * @brief 初始化 FDCAN BSP。
 */
inline void init()
{
    bsp_fdcan_init();
}

/**
 * @brief 获取 FDCAN HAL 句柄。
 *
 * @param id FDCAN 总线标识。
 * @return HAL FDCAN 句柄。
 */
inline FDCAN_HandleTypeDef *handle(Id id)
{
    return bsp_fdcan_handle(static_cast<bsp_fdcan_id_t>(id));
}

/**
 * @brief 配置标准帧过滤器。
 */
inline HAL_StatusTypeDef filter(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t index,
                                uint32_t fifo,
                                uint16_t id,
                                uint16_t mask)
{
    return bsp_fdcan_config_std_filter(hfdcan, index, fifo, id, mask);
}

/**
 * @brief 接收所有标准帧。
 */
inline HAL_StatusTypeDef accept(FDCAN_HandleTypeDef *hfdcan, uint32_t index, uint32_t fifo)
{
    return bsp_fdcan_accept_std(hfdcan, index, fifo);
}

/**
 * @brief 拒收剩余帧和远程帧。
 */
inline HAL_StatusTypeDef reject(FDCAN_HandleTypeDef *hfdcan)
{
    return bsp_fdcan_reject_rest(hfdcan);
}

/**
 * @brief 启动 FDCAN。
 */
inline HAL_StatusTypeDef start(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    return bsp_fdcan_start(hfdcan, fifo);
}

/**
 * @brief 启动板级默认 FDCAN 配置。
 */
inline HAL_StatusTypeDef start_default()
{
    return bsp_fdcan_start_default();
}

/**
 * @brief 发送标准帧。
 */
inline HAL_StatusTypeDef send(FDCAN_HandleTypeDef *hfdcan,
                              uint16_t id,
                              const uint8_t *data,
                              uint8_t len)
{
    return bsp_fdcan_send_std(hfdcan, id, data, len);
}

/**
 * @brief 将 DLC 转换为字节长度。
 */
inline uint8_t dlc_len(uint32_t dlc)
{
    return bsp_fdcan_dlc_to_length(dlc);
}

/**
 * @brief 绑定 RX 回调。
 */
inline HAL_StatusTypeDef bind(FDCAN_HandleTypeDef *hfdcan,
                              uint32_t fifo,
                              RxCallback callback,
                              void *user)
{
    return bsp_fdcan_rx_bind(hfdcan, fifo, callback, user);
}

/**
 * @brief FDCAN 总线编译期特性。
 *
 * @tparam Bus FDCAN 总线标识。
 */
template<Id Bus>
struct Port;

/**
 * @brief FDCAN1 编译期端口。
 */
template<>
struct Port<Id::Fdcan1> {
    /** @brief 默认 RX FIFO。 */
    static constexpr uint32_t RxFifo = BSP_FDCAN_DEFAULT0_RX_FIFO;
    /** @brief 默认过滤器索引。 */
    static constexpr uint32_t Filter = BSP_FDCAN_DEFAULT0_FILTER;

    /** @brief 获取 HAL 句柄。 */
    static FDCAN_HandleTypeDef *handle()
    {
        return Bsp::Fdcan::handle(Id::Fdcan1);
    }
};

/**
 * @brief FDCAN2 编译期端口。
 */
template<>
struct Port<Id::Fdcan2> {
    /** @brief 默认 RX FIFO。 */
    static constexpr uint32_t RxFifo = BSP_FDCAN_DEFAULT1_RX_FIFO;
    /** @brief 默认过滤器索引。 */
    static constexpr uint32_t Filter = BSP_FDCAN_DEFAULT1_FILTER;

    /** @brief 获取 HAL 句柄。 */
    static FDCAN_HandleTypeDef *handle()
    {
        return Bsp::Fdcan::handle(Id::Fdcan2);
    }
};

/**
 * @brief FDCAN3 编译期端口。
 */
template<>
struct Port<Id::Fdcan3> {
    /** @brief 默认 RX FIFO。 */
    static constexpr uint32_t RxFifo = BSP_FDCAN_DEFAULT2_RX_FIFO;
    /** @brief 默认过滤器索引。 */
    static constexpr uint32_t Filter = BSP_FDCAN_DEFAULT2_FILTER;

    /** @brief 获取 HAL 句柄。 */
    static FDCAN_HandleTypeDef *handle()
    {
        return Bsp::Fdcan::handle(Id::Fdcan3);
    }
};

/**
 * @brief 默认配置并启动指定 FDCAN 端口。
 *
 * @tparam Bus FDCAN 总线标识。
 * @return HAL 执行状态。
 */
template<Id Bus>
inline HAL_StatusTypeDef start()
{
    FDCAN_HandleTypeDef *hfdcan = Port<Bus>::handle();

    if ((accept(hfdcan, Port<Bus>::Filter, Port<Bus>::RxFifo) != HAL_OK) ||
        (reject(hfdcan) != HAL_OK) ||
        (start(hfdcan, Port<Bus>::RxFifo) != HAL_OK)) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief 通过编译期端口发送标准帧。
 *
 * @tparam Bus FDCAN 总线标识。
 * @param id 标准 CAN ID。
 * @param data 数据指针。
 * @param len 数据长度。
 * @return HAL 执行状态。
 */
template<Id Bus>
inline HAL_StatusTypeDef send(uint16_t id, const uint8_t *data, uint8_t len)
{
    return send(Port<Bus>::handle(), id, data, len);
}

/**
 * @brief 通过编译期端口绑定默认 RX FIFO 回调。
 *
 * @tparam Bus FDCAN 总线标识。
 * @param callback 接收回调。
 * @param user 用户上下文。
 * @return HAL 执行状态。
 */
template<Id Bus>
inline HAL_StatusTypeDef bind(RxCallback callback, void *user)
{
    return bind(Port<Bus>::handle(), Port<Bus>::RxFifo, callback, user);
}

} // namespace Bsp::Fdcan

#endif // PRINTK_BSP_FDCAN_HPP
