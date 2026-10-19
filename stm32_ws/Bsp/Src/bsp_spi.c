/**
 ******************************************************************************
 * @file    bsp_spi.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   SPI 控制器 BSP 抽象实现。
 *
 * @note    本文件只维护 SPI 句柄表、HAL SPI 事务包装和 HAL SPI 回调分发。
 * @note    与设备驱动混合写法相比，新版不在 BSP 中控制片选时序、不解析寄存器协议，
 *          也不维护传感器或外设业务状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_spi.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief SPI 事件绑定槽。
 */
typedef struct
{
    bsp_spi_event_callback_t callback;    /**< 事件回调，可为 NULL。 */
    void *user;                           /**< 用户上下文。 */
} bsp_spi_callback_slot_t;

/**
 * @brief SPI 分发槽位。
 *
 * 每个槽位对应一个板级 SPI 句柄及其事件回调注册信息。
 */
typedef struct
{
    bsp_spi_id_t id;                                        /**< BSP 层暴露给上层使用的 SPI 标识。 */
    SPI_HandleTypeDef *hspi;                                /**< CubeMX/HAL 生成的 SPI 句柄。 */
    SPI_TypeDef *instance;                                  /**< SPI 外设实例地址，例如 SPI2。 */
    bsp_spi_callback_slot_t events[BSP_SPI_EVENT_COUNT];    /**< SPI 事件绑定表。 */
} bsp_spi_slot_t;

/**
 * @brief 板级 SPI 回调注册表。
 *
 * 新增或删除 SPI 时，本表必须与 bsp_spi_id_t 和 CubeMX 生成的 hspix/SPIx 实例保持一致。
 */
static bsp_spi_slot_t bsp_spi_slots[] = {
    {.id = BSP_SPI_ID_SPI2, .hspi = &hspi2, .instance = SPI2},
    {.id = BSP_SPI_ID_SPI6, .hspi = &hspi6, .instance = SPI6},
};

#define BSP_SPI_SLOT_COUNT ((uint32_t)(sizeof(bsp_spi_slots) / sizeof(bsp_spi_slots[0])))    /**< bsp_spi_slots[] 中受 BSP 管理的 SPI 数量。 */

/**
 * @brief 根据 SPI 句柄查找回调槽位。
 *
 * @param hspi 目标 SPI 句柄。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_spi_slot_t *slot_by_handle(SPI_HandleTypeDef *hspi)
{
    uint32_t i;

    if (hspi == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_SPI_SLOT_COUNT; i++) {
        if (bsp_spi_slots[i].hspi == hspi) {
            return &bsp_spi_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP SPI 标识查找回调槽位。
 *
 * @param id BSP SPI 标识。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_spi_slot_t *slot_by_id(bsp_spi_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_SPI_SLOT_COUNT; i++) {
        if (bsp_spi_slots[i].id == id) {
            return &bsp_spi_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据外设实例查找回调槽位。
 *
 * @param instance SPI 外设实例。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_spi_slot_t *slot_by_instance(SPI_TypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_SPI_SLOT_COUNT; i++) {
        if (bsp_spi_slots[i].instance == instance) {
            return &bsp_spi_slots[i];
        }
    }

    return NULL;
}

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */

/**
 * @brief 清空单个 SPI 槽位的事件绑定。
 *
 * @param slot SPI 分发槽位。
 */
static void clear_callbacks(bsp_spi_slot_t *slot)
{
    uint32_t i;

    if (slot == NULL) {
        return;
    }

    for (i = 0U; i < BSP_SPI_EVENT_COUNT; i++) {
        slot->events[i].callback = NULL;
        slot->events[i].user = NULL;
    }
}

/**
 * @brief 分发 SPI 普通事件。
 *
 * @param hspi 触发事件的 SPI 句柄。
 * @param event SPI 事件类型。
 */
static void dispatch_event(SPI_HandleTypeDef *hspi, bsp_spi_event_t event)
{
    bsp_spi_slot_t *slot = slot_by_handle(hspi);

    if ((slot == NULL) || (event >= BSP_SPI_EVENT_COUNT) || (slot->events[event].callback == NULL)) {
        return;
    }

    slot->events[event].callback(hspi, slot->events[event].user);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_spi_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_SPI_SLOT_COUNT; i++) {
        /* @diff: 新版 SPI BSP 只清空事件回调，不启动传输，也不维护设备协议状态。 */
        clear_callbacks(&bsp_spi_slots[i]);
    }
}

SPI_HandleTypeDef *bsp_spi_handle(bsp_spi_id_t id)
{
    bsp_spi_slot_t *slot = slot_by_id(id);

    if (slot == NULL) {
        return NULL;
    }

    return slot->hspi;
}

SPI_HandleTypeDef *bsp_spi_from_instance(SPI_TypeDef *instance)
{
    bsp_spi_slot_t *slot = slot_by_instance(instance);

    if (slot == NULL) {
        return NULL;
    }

    return slot->hspi;
}

uint8_t bsp_spi_is_supported(SPI_HandleTypeDef *hspi)
{
    return (slot_by_handle(hspi) != NULL) ? 1U : 0U;
}

HAL_StatusTypeDef bsp_spi_transfer(SPI_HandleTypeDef *hspi, const bsp_spi_transfer_t *xfer)
{
    if ((slot_by_handle(hspi) == NULL) || (xfer == NULL) || (xfer->size == 0U)) {
        return HAL_ERROR;
    }

    if ((xfer->tx_data == NULL) && (xfer->rx_data == NULL)) {
        return HAL_ERROR;
    }

    switch (xfer->mode) {
    case BSP_SPI_MODE_POLL:
        if ((xfer->tx_data != NULL) && (xfer->rx_data != NULL)) {
            return HAL_SPI_TransmitReceive(hspi, xfer->tx_data, xfer->rx_data, xfer->size, xfer->timeout);
        }
        if (xfer->tx_data != NULL) {
            return HAL_SPI_Transmit(hspi, xfer->tx_data, xfer->size, xfer->timeout);
        }
        return HAL_SPI_Receive(hspi, xfer->rx_data, xfer->size, xfer->timeout);

    case BSP_SPI_MODE_IT:
        if ((xfer->tx_data != NULL) && (xfer->rx_data != NULL)) {
            return HAL_SPI_TransmitReceive_IT(hspi, xfer->tx_data, xfer->rx_data, xfer->size);
        }
        if (xfer->tx_data != NULL) {
            return HAL_SPI_Transmit_IT(hspi, xfer->tx_data, xfer->size);
        }
        return HAL_SPI_Receive_IT(hspi, xfer->rx_data, xfer->size);

    case BSP_SPI_MODE_DMA:
        if ((xfer->tx_data != NULL) && (xfer->rx_data != NULL)) {
            return HAL_SPI_TransmitReceive_DMA(hspi, xfer->tx_data, xfer->rx_data, xfer->size);
        }
        if (xfer->tx_data != NULL) {
            return HAL_SPI_Transmit_DMA(hspi, xfer->tx_data, xfer->size);
        }
        return HAL_SPI_Receive_DMA(hspi, xfer->rx_data, xfer->size);

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef bsp_spi_abort(SPI_HandleTypeDef *hspi)
{
    if (slot_by_handle(hspi) == NULL) {
        return HAL_ERROR;
    }

    return HAL_SPI_Abort(hspi);
}

HAL_StatusTypeDef bsp_spi_abort_it(SPI_HandleTypeDef *hspi)
{
    if (slot_by_handle(hspi) == NULL) {
        return HAL_ERROR;
    }

    return HAL_SPI_Abort_IT(hspi);
}

HAL_SPI_StateTypeDef bsp_spi_get_state(SPI_HandleTypeDef *hspi)
{
    if (slot_by_handle(hspi) == NULL) {
        return HAL_SPI_STATE_RESET;
    }

    return HAL_SPI_GetState(hspi);
}

uint32_t bsp_spi_get_error(SPI_HandleTypeDef *hspi)
{
    if (slot_by_handle(hspi) == NULL) {
        return HAL_SPI_ERROR_NONE;
    }

    return HAL_SPI_GetError(hspi);
}

HAL_StatusTypeDef bsp_spi_on(SPI_HandleTypeDef *hspi,
                             bsp_spi_event_t event,
                             bsp_spi_event_callback_t callback,
                             void *user)
{
    bsp_spi_slot_t *slot = slot_by_handle(hspi);

    if ((slot == NULL) || (event >= BSP_SPI_EVENT_COUNT)) {
        return HAL_ERROR;
    }

    slot->events[event].callback = callback;
    slot->events[event].user = user;
    return HAL_OK;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_TX_CPLT);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_RX_CPLT);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_TXRX_CPLT);
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_TX_HALF);
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_RX_HALF);
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_TXRX_HALF);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_ERROR);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_ABORT);
}

void HAL_SPI_SuspendCallback(SPI_HandleTypeDef *hspi)
{
    dispatch_event(hspi, BSP_SPI_EVENT_SUSPEND);
}
