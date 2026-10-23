/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   UART BSP 抽象实现。
 *
 * @note    本文件只维护 UART 句柄表、HAL 收发事务包装和 HAL 回调分发。
 *          协议 buffer、解析逻辑和设备状态应放在上层驱动或应用模块。
 * @note    与旧版 bsp_usart 相比，新版不再包含遥控器、裁判系统、
 *          板间通信、TFmini 或云台/底盘协议逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_uart.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief UART 分发槽位。
 *
 * 每个槽位对应一个板级 UART 句柄及其事件回调注册信息。
 */
typedef struct
{
    bsp_uart_id_t id;                                /**< BSP 层暴露给上层使用的 UART 标识。 */
    UART_HandleTypeDef *huart;                       /**< CubeMX/HAL 生成的 UART 句柄。 */
    USART_TypeDef *instance;                         /**< UART 外设实例地址。 */
    bsp_uart_event_callback_t rx_cplt_callback;      /**< 接收完成回调，可为 NULL。 */
    void *rx_cplt_user;                              /**< 接收完成用户上下文。 */
    bsp_uart_rx_idle_callback_t rx_idle_callback;    /**< idle 接收回调，可为 NULL。 */
    void *rx_idle_user;                              /**< idle 接收用户上下文。 */
    bsp_uart_event_callback_t tx_cplt_callback;      /**< 发送完成回调，可为 NULL。 */
    void *tx_cplt_user;                              /**< 发送完成用户上下文。 */
    bsp_uart_event_callback_t error_callback;        /**< 错误回调，可为 NULL。 */
    void *error_user;                                /**< 错误事件用户上下文。 */
} bsp_uart_slot_t;

/**
 * @brief 板级 UART 回调注册表。
 *
 * 新增或删除 UART 时，本表必须与 bsp_uart_id_t 和 CubeMX 生成的
 * huartx/USARTx 实例保持一致。
 */
static bsp_uart_slot_t bsp_uart_slots[] = {
    {.id = BSP_UART_ID_USART2, .huart = &huart2, .instance = USART2},
    {.id = BSP_UART_ID_USART1, .huart = &huart1, .instance = USART1},
    {.id = BSP_UART_ID_USART3, .huart = &huart3, .instance = USART3},
    {.id = BSP_UART_ID_UART5, .huart = &huart5, .instance = UART5},
    {.id = BSP_UART_ID_UART7, .huart = &huart7, .instance = UART7},
    {.id = BSP_UART_ID_UART8, .huart = &huart8, .instance = UART8},
    {.id = BSP_UART_ID_UART9, .huart = &huart9, .instance = UART9},
    {.id = BSP_UART_ID_USART10, .huart = &huart10, .instance = USART10},
};

#define BSP_UART_SLOT_COUNT ((uint32_t)(sizeof(bsp_uart_slots) / sizeof(bsp_uart_slots[0])))    /**< bsp_uart_slots[] 中受 BSP 管理的 UART 数量。 */

/**
 * @brief 根据 UART 句柄查找回调槽位。
 *
 * @param huart 目标 UART 句柄。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_uart_slot_t *slot_by_handle(UART_HandleTypeDef *huart)
{
    uint32_t i;

    if (huart == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_UART_SLOT_COUNT; i++) {
        if (bsp_uart_slots[i].huart == huart) {
            return &bsp_uart_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP UART 标识查找回调槽位。
 *
 * @param id BSP UART 标识。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_uart_slot_t *slot_by_id(bsp_uart_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_UART_SLOT_COUNT; i++) {
        if (bsp_uart_slots[i].id == id) {
            return &bsp_uart_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据外设实例查找回调槽位。
 *
 * @param instance UART 外设实例。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_uart_slot_t *slot_by_instance(USART_TypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_UART_SLOT_COUNT; i++) {
        if (bsp_uart_slots[i].instance == instance) {
            return &bsp_uart_slots[i];
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
 * @brief 清空单个 UART 槽位的事件绑定。
 *
 * @param slot UART 分发槽位。
 */
static void clear_callbacks(bsp_uart_slot_t *slot)
{
    if (slot == NULL) {
        return;
    }

    slot->rx_cplt_callback = NULL;
    slot->rx_cplt_user = NULL;
    slot->rx_idle_callback = NULL;
    slot->rx_idle_user = NULL;
    slot->tx_cplt_callback = NULL;
    slot->tx_cplt_user = NULL;
    slot->error_callback = NULL;
    slot->error_user = NULL;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_uart_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_UART_SLOT_COUNT; i++) {
        /* @diff: 旧版在 HAL 回调里直接处理协议；新版初始化只清空分发回调。 */
        clear_callbacks(&bsp_uart_slots[i]);
    }
}

UART_HandleTypeDef *bsp_uart_handle(bsp_uart_id_t id)
{
    bsp_uart_slot_t *slot = slot_by_id(id);

    if (slot == NULL) {
        return NULL;
    }

    return slot->huart;
}

UART_HandleTypeDef *bsp_uart_from_instance(USART_TypeDef *instance)
{
    bsp_uart_slot_t *slot = slot_by_instance(instance);

    if (slot == NULL) {
        return NULL;
    }

    return slot->huart;
}

uint8_t bsp_uart_is_supported(UART_HandleTypeDef *huart)
{
    return (slot_by_handle(huart) != NULL) ? 1U : 0U;
}

HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, const bsp_uart_tx_t *tx)
{
    if ((slot_by_handle(huart) == NULL) || (tx == NULL) || (tx->data == NULL) || (tx->size == 0U)) {
        return HAL_ERROR;
    }

    switch (tx->mode) {
    case BSP_UART_MODE_POLL:
        return HAL_UART_Transmit(huart, (uint8_t *)tx->data, tx->size, tx->timeout);

    case BSP_UART_MODE_IT:
        return HAL_UART_Transmit_IT(huart, (uint8_t *)tx->data, tx->size);

    case BSP_UART_MODE_DMA:
        return HAL_UART_Transmit_DMA(huart, (uint8_t *)tx->data, tx->size);

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef bsp_uart_rx(UART_HandleTypeDef *huart, const bsp_uart_rx_t *rx)
{
    if ((slot_by_handle(huart) == NULL) || (rx == NULL) || (rx->data == NULL) || (rx->size == 0U)) {
        return HAL_ERROR;
    }

    switch (rx->mode) {
    case BSP_UART_MODE_POLL:
        return HAL_UART_Receive(huart, rx->data, rx->size, rx->timeout);

    case BSP_UART_MODE_IT:
        return HAL_UART_Receive_IT(huart, rx->data, rx->size);

    case BSP_UART_MODE_DMA:
        return HAL_UART_Receive_DMA(huart, rx->data, rx->size);

    case BSP_UART_MODE_DMA_TO_IDLE:
        return HAL_UARTEx_ReceiveToIdle_DMA(huart, rx->data, rx->size);

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef bsp_uart_abort_rx(UART_HandleTypeDef *huart)
{
    if (slot_by_handle(huart) == NULL) {
        return HAL_ERROR;
    }

    return HAL_UART_AbortReceive(huart);
}

HAL_StatusTypeDef bsp_uart_abort_tx(UART_HandleTypeDef *huart)
{
    if (slot_by_handle(huart) == NULL) {
        return HAL_ERROR;
    }

    return HAL_UART_AbortTransmit(huart);
}

HAL_StatusTypeDef bsp_uart_abort(UART_HandleTypeDef *huart)
{
    if (slot_by_handle(huart) == NULL) {
        return HAL_ERROR;
    }

    return HAL_UART_Abort(huart);
}

HAL_StatusTypeDef bsp_uart_on(UART_HandleTypeDef *huart,
                              bsp_uart_event_t event,
                              bsp_uart_event_callback_t callback,
                              void *user)
{
    bsp_uart_slot_t *slot = slot_by_handle(huart);

    if ((slot == NULL) || (event >= BSP_UART_EVENT_COUNT) || (event == BSP_UART_EVENT_RX_IDLE)) {
        return HAL_ERROR;
    }

    switch (event) {
    case BSP_UART_EVENT_RX_CPLT:
        slot->rx_cplt_callback = (bsp_uart_event_callback_t)callback;
        slot->rx_cplt_user = user;
        return HAL_OK;

    case BSP_UART_EVENT_TX_CPLT:
        slot->tx_cplt_callback = callback;
        slot->tx_cplt_user = user;
        return HAL_OK;

    case BSP_UART_EVENT_ERROR:
        slot->error_callback = callback;
        slot->error_user = user;
        return HAL_OK;

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef bsp_uart_on_rx_idle(UART_HandleTypeDef *huart,
                                      bsp_uart_rx_idle_callback_t callback,
                                      void *user)
{
    bsp_uart_slot_t *slot = slot_by_handle(huart);

    if (slot == NULL) {
        return HAL_ERROR;
    }

    slot->rx_idle_callback = callback;
    slot->rx_idle_user = user;
    return HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_slot_t *slot = slot_by_handle(huart);

    if ((slot == NULL) || (slot->rx_cplt_callback == NULL)) {
        return;
    }

    slot->rx_cplt_callback(huart, slot->rx_cplt_user);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    bsp_uart_slot_t *slot = slot_by_handle(huart);

    if ((slot == NULL) || (slot->rx_idle_callback == NULL)) {
        return;
    }

    slot->rx_idle_callback(huart, size, slot->rx_idle_user);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_slot_t *slot = slot_by_handle(huart);

    if ((slot == NULL) || (slot->error_callback == NULL)) {
        return;
    }

    slot->error_callback(huart, slot->error_user);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_slot_t *slot = slot_by_handle(huart);

    if ((slot == NULL) || (slot->tx_cplt_callback == NULL)) {
        return;
    }

    slot->tx_cplt_callback(huart, slot->tx_cplt_user);
}
