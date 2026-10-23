/**
 ******************************************************************************
 * @file    bsp_spi.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   SPI 控制器 BSP 抽象接口。
 *
 * @note    本文件只提供 SPI 句柄获取、HAL SPI 事务包装、状态读取和 HAL SPI
 *          回调分发。
 * @note    片选 GPIO、寄存器协议、IMU/Flash/传感器解析和业务缓存应放在设备驱动层，
 *          不应放在 SPI BSP。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_SPI_H
#define PRINTK_BSP_SPI_H

#include "spi.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */

/**
 * @brief 板上可由 BSP 管理的 SPI 标识。
 *
 * 新增或删除 SPI 实例时，需要同步维护本枚举和 bsp_spi.c 中的 bsp_spi_slots[]。
 */
typedef enum
{
    BSP_SPI_ID_SPI2 = 0, /**< SPI2。 */
    BSP_SPI_ID_SPI6,     /**< SPI6。 */
    BSP_SPI_ID_COUNT     /**< SPI 标识数量。 */
} bsp_spi_id_t;

/**
 * @brief SPI 传输模式。
 */
typedef enum
{
    BSP_SPI_MODE_POLL = 0,    /**< 阻塞模式。 */
    BSP_SPI_MODE_IT,          /**< 中断模式。 */
    BSP_SPI_MODE_DMA          /**< DMA 模式。 */
} bsp_spi_mode_t;

/**
 * @brief SPI 事件类型。
 */
typedef enum
{
    BSP_SPI_EVENT_TX_CPLT = 0,
    BSP_SPI_EVENT_RX_CPLT,
    BSP_SPI_EVENT_TXRX_CPLT,
    BSP_SPI_EVENT_TX_HALF,
    BSP_SPI_EVENT_RX_HALF,
    BSP_SPI_EVENT_TXRX_HALF,
    BSP_SPI_EVENT_ERROR,
    BSP_SPI_EVENT_ABORT,
    BSP_SPI_EVENT_SUSPEND,
    BSP_SPI_EVENT_COUNT
} bsp_spi_event_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief SPI 事件回调函数类型。
 *
 * @param hspi 触发事件的 SPI 句柄。
 * @param user 注册回调时传入的用户上下文。
 */
typedef void (*bsp_spi_event_callback_t)(SPI_HandleTypeDef *hspi, void *user);

/**
 * @brief SPI 收发事务。
 *
 * tx_data 和 rx_data 均非 NULL 时执行全双工收发；仅 tx_data 非 NULL 时执行发送；
 * 仅 rx_data 非 NULL 时执行接收。
 */
typedef struct
{
    const uint8_t *tx_data;    /**< 待发送数据缓冲区，可为 NULL。 */
    uint8_t *rx_data;          /**< 接收缓冲区，可为 NULL。 */
    uint16_t size;             /**< 收发长度，单位为字节。 */
    uint32_t timeout;          /**< 阻塞模式超时时间，单位为毫秒。 */
    bsp_spi_mode_t mode;       /**< 传输模式。 */
} bsp_spi_transfer_t;

/**
 * @brief 初始化 SPI BSP 回调表。
 *
 * @note 差异: 新版 SPI BSP 只清空回调注册状态，不重新初始化 SPI，也不启动传输。
 */
void bsp_spi_init(void);

/**
 * @brief 根据 BSP SPI 标识获取 SPI 句柄。
 *
 * @param id BSP SPI 标识。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
SPI_HandleTypeDef *bsp_spi_handle(bsp_spi_id_t id);

/**
 * @brief 根据外设实例地址获取 SPI 句柄。
 *
 * @param instance SPI 外设实例，例如 `SPI2`。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
SPI_HandleTypeDef *bsp_spi_from_instance(SPI_TypeDef *instance);

/**
 * @brief 判断 SPI 句柄是否由本 BSP 管理。
 *
 * @param hspi 待检查的 SPI 句柄。
 * @return 受管理返回 1，否则返回 0。
 */
uint8_t bsp_spi_is_supported(SPI_HandleTypeDef *hspi);

/**
 * @brief 执行 SPI 收发事务。
 *
 * @param hspi 目标 SPI 句柄。
 * @param xfer SPI 收发事务配置。
 * @return 返回 HAL SPI 状态码。
 */
HAL_StatusTypeDef bsp_spi_transfer(SPI_HandleTypeDef *hspi, const bsp_spi_transfer_t *xfer);

/**
 * @brief 中止当前 SPI 传输。
 *
 * @param hspi 目标 SPI 句柄。
 * @return 返回 HAL SPI 状态码。
 */
HAL_StatusTypeDef bsp_spi_abort(SPI_HandleTypeDef *hspi);

/**
 * @brief 通过中断方式中止当前 SPI 传输。
 *
 * @param hspi 目标 SPI 句柄。
 * @return 返回 HAL SPI 状态码。
 */
HAL_StatusTypeDef bsp_spi_abort_it(SPI_HandleTypeDef *hspi);

/**
 * @brief 获取 SPI HAL 状态。
 *
 * @param hspi 目标 SPI 句柄。
 * @return 返回 HAL SPI 状态；入参非法时返回 `HAL_SPI_STATE_RESET`。
 */
HAL_SPI_StateTypeDef bsp_spi_get_state(SPI_HandleTypeDef *hspi);

/**
 * @brief 获取 SPI HAL 错误码。
 *
 * @param hspi 目标 SPI 句柄。
 * @return 返回 HAL SPI 错误码；入参非法时返回 `HAL_SPI_ERROR_NONE`。
 */
uint32_t bsp_spi_get_error(SPI_HandleTypeDef *hspi);

/**
 * @brief 绑定 SPI 事件回调。
 *
 * @param hspi 目标 SPI 句柄。
 * @param event SPI 事件类型。
 * @param callback 事件回调，传入 `NULL` 表示取消绑定。
 * @param user 用户上下文。
 * @return 返回 HAL SPI 状态码。
 */
HAL_StatusTypeDef bsp_spi_on(SPI_HandleTypeDef *hspi,
                             bsp_spi_event_t event,
                             bsp_spi_event_callback_t callback,
                             void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_SPI_H
