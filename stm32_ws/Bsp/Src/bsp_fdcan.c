/**
 ******************************************************************************
 * @file    bsp_fdcan.c
 * @author  GPT
 * @version V1.3.0
 * @date    2026-05-15
 * @brief   FDCAN 控制器 BSP 抽象实现。
 *
 * @note    本文件只维护 FDCAN 句柄表、过滤器/通知启动、Classic CAN
 *          标准帧发送和 HAL RX FIFO 回调分发。
 * @note    与旧版 can_bsp 相比，新版不再包含 DJI/MIT 电机解析、控制帧
 *          封包或全局电机状态数组。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_fdcan.h"

#include "robot_board.h"

#include <stddef.h>

/**
 * @brief TX FIFO 满时的最大等待时间，单位 ms。
 *
 * 四轮台架会在一个循环里连续发送十几帧，而 FDCAN TX FIFO 只有 8 个元素。
 * 直接调用 HAL_FDCAN_AddMessageToTxFifoQ() 会在 FIFO 满时返回 HAL_BUSY，
 * 上层若忽略返回值就会静默丢帧。这里做有界等待/重试，保证控制帧入队。
 */
#define BSP_FDCAN_TX_TIMEOUT_MS 10U

volatile uint32_t g_bsp_fdcan_tx_wait_count;      /**< TX FIFO 满导致的等待次数。 */
volatile uint32_t g_bsp_fdcan_tx_timeout_count;   /**< TX FIFO 等待超时次数。 */

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief FDCAN 接收订阅者。
 *
 * 每个订阅者对应一个上层驱动回调及其用户上下文。
 */
typedef struct
{
    bsp_fdcan_rx_callback_t callback;    /**< 接收回调，可为 NULL。 */
    void *user;                          /**< 传递给接收回调的用户上下文。 */
} bsp_fdcan_rx_subscriber_t;

/**
 * @brief FDCAN 分发槽位。
 *
 * 每个槽位对应一个板级 FDCAN 句柄，以及 FIFO0/FIFO1 的订阅者列表。
 */
typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;                                                     /**< CubeMX/HAL 生成的 FDCAN 句柄。 */
    bsp_fdcan_rx_subscriber_t rx_fifo0_subscribers[BSP_FDCAN_MAX_RX_SUBSCRIBERS];    /**< FIFO0 订阅者。 */
    bsp_fdcan_rx_subscriber_t rx_fifo1_subscribers[BSP_FDCAN_MAX_RX_SUBSCRIBERS];    /**< FIFO1 订阅者。 */
} bsp_fdcan_slot_t;

/**
 * @brief 默认 FDCAN 启动配置。
 */
typedef struct
{
    bsp_fdcan_id_t id;       /**< FDCAN 总线标识。 */
    uint32_t rx_fifo;        /**< 默认接收 FIFO。 */
    uint32_t filter_index;   /**< 默认标准帧过滤器索引。 */
} bsp_fdcan_default_t;

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * BSP 层只缓存外设句柄、DMA 缓冲和硬件状态，具体设备用途由 Robot 配置决定。
 * ========================================================================== */

/**
 * @brief 板级 FDCAN 回调注册表。
 *
 * 新增或删除 FDCAN 时，本表必须与 bsp_fdcan_id_t 和 CubeMX 生成的
 * hfdcanx/FDCANx 实例保持一致。
 */
static bsp_fdcan_slot_t bsp_fdcan_slots[BSP_FDCAN_ID_COUNT] = {
    [BSP_FDCAN_ID_FDCAN1] = {.hfdcan = &hfdcan1},
    [BSP_FDCAN_ID_FDCAN2] = {.hfdcan = &hfdcan2},
    [BSP_FDCAN_ID_FDCAN3] = {.hfdcan = &hfdcan3},
};

#define BSP_FDCAN_SLOT_COUNT ((uint32_t)(sizeof(bsp_fdcan_slots) / sizeof(bsp_fdcan_slots[0])))    /**< bsp_fdcan_slots[] 中受 BSP 管理的 FDCAN 数量。 */

/**
 * @brief 当前板卡默认启动的 FDCAN 配置。
 */
static const bsp_fdcan_default_t bsp_fdcan_defaults[] = {
    {.id = BSP_FDCAN_DEFAULT0_ID, .rx_fifo = BSP_FDCAN_DEFAULT0_RX_FIFO, .filter_index = BSP_FDCAN_DEFAULT0_FILTER},
    {.id = BSP_FDCAN_DEFAULT1_ID, .rx_fifo = BSP_FDCAN_DEFAULT1_RX_FIFO, .filter_index = BSP_FDCAN_DEFAULT1_FILTER},
    {.id = BSP_FDCAN_DEFAULT2_ID, .rx_fifo = BSP_FDCAN_DEFAULT2_RX_FIFO, .filter_index = BSP_FDCAN_DEFAULT2_FILTER},
};

#define BSP_FDCAN_DEFAULT_COUNT ((uint32_t)(sizeof(bsp_fdcan_defaults) / sizeof(bsp_fdcan_defaults[0])))    /**< 默认 FDCAN 启动项数量。 */

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */

/**
 * @brief 清空指定订阅者列表。
 *
 * @param subscribers 订阅者列表。
 */
static void clear_subs(bsp_fdcan_rx_subscriber_t *subscribers)
{
    uint32_t i;

    if (subscribers == NULL) {
        return;
    }

    for (i = 0U; i < BSP_FDCAN_MAX_RX_SUBSCRIBERS; i++) {
        subscribers[i].callback = NULL;
        subscribers[i].user = NULL;
    }
}

/**
 * @brief 根据 FDCAN 句柄查找回调槽位。
 *
 * @param hfdcan 目标 FDCAN 句柄。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_fdcan_slot_t *slot_by_handle(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t i;

    if (hfdcan == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_FDCAN_SLOT_COUNT; i++) {
        if (bsp_fdcan_slots[i].hfdcan == hfdcan) {
            return &bsp_fdcan_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP FDCAN 标识查找回调槽位。
 *
 * @param id BSP FDCAN 标识。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_fdcan_slot_t *slot_by_id(bsp_fdcan_id_t id)
{
    if ((uint32_t)id >= BSP_FDCAN_SLOT_COUNT) {
        return NULL;
    }

    return &bsp_fdcan_slots[(uint32_t)id];
}

/**
 * @brief 根据 FIFO 编号获取订阅者列表。
 *
 * @param slot FDCAN 分发槽位。
 * @param fifo_index HAL 定义的 RX FIFO 编号。
 * @return 匹配成功返回订阅者列表，否则返回 `NULL`。
 */
static bsp_fdcan_rx_subscriber_t *get_subs(bsp_fdcan_slot_t *slot,
                                                            uint32_t fifo_index)
{
    if (slot == NULL) {
        return NULL;
    }

    if (fifo_index == FDCAN_RX_FIFO0) {
        return slot->rx_fifo0_subscribers;
    }

    if (fifo_index == FDCAN_RX_FIFO1) {
        return slot->rx_fifo1_subscribers;
    }

    return NULL;
}

/**
 * @brief 将 Classic CAN 数据长度转换为 HAL DLC 编码。
 *
 * @param len 数据长度，范围 0~8。
 * @param dlc 输出 HAL FDCAN DataLength 编码。
 * @return 转换成功返回 HAL_OK，否则返回 HAL_ERROR。
 */
static HAL_StatusTypeDef len_to_dlc(uint8_t len, uint32_t *dlc)
{
    if (dlc == NULL) {
        return HAL_ERROR;
    }

    switch (len) {
    case 0U:
        *dlc = FDCAN_DLC_BYTES_0;
        return HAL_OK;
    case 1U:
        *dlc = FDCAN_DLC_BYTES_1;
        return HAL_OK;
    case 2U:
        *dlc = FDCAN_DLC_BYTES_2;
        return HAL_OK;
    case 3U:
        *dlc = FDCAN_DLC_BYTES_3;
        return HAL_OK;
    case 4U:
        *dlc = FDCAN_DLC_BYTES_4;
        return HAL_OK;
    case 5U:
        *dlc = FDCAN_DLC_BYTES_5;
        return HAL_OK;
    case 6U:
        *dlc = FDCAN_DLC_BYTES_6;
        return HAL_OK;
    case 7U:
        *dlc = FDCAN_DLC_BYTES_7;
        return HAL_OK;
    case 8U:
        *dlc = FDCAN_DLC_BYTES_8;
        return HAL_OK;
    default:
        return HAL_ERROR;
    }
}

/* ==========================================================================
 * 收发与回调处理
 * --------------------------------------------------------------------------
 * 这里处理外设收发、DMA/中断回调和 HAL 适配，不在 BSP 内解释具体协议。
 * ========================================================================== */

/**
 * @brief 注册 RX FIFO 接收订阅回调。
 *
 * @param hfdcan 目标 FDCAN 句柄。
 * @param fifo_index HAL 定义的 RX FIFO 编号。
 * @param callback 接收订阅回调，传入 NULL 表示清空该 FIFO 订阅列表。
 * @param user 用户上下文。
 * @return 返回 HAL FDCAN 状态码。
 */
static HAL_StatusTypeDef bind_rx_cb(FDCAN_HandleTypeDef *hfdcan,
                                                    uint32_t fifo_index,
                                                    bsp_fdcan_rx_callback_t callback,
                                                    void *user)
{
    bsp_fdcan_slot_t *slot = slot_by_handle(hfdcan);
    bsp_fdcan_rx_subscriber_t *subscribers;
    uint32_t i;

    if (slot == NULL) {
        return HAL_ERROR;
    }

    subscribers = get_subs(slot, fifo_index);
    if (subscribers == NULL) {
        return HAL_ERROR;
    }

    if (callback == NULL) {
        clear_subs(subscribers);
        return HAL_OK;
    }

    for (i = 0U; i < BSP_FDCAN_MAX_RX_SUBSCRIBERS; i++) {
        if ((subscribers[i].callback == callback) && (subscribers[i].user == user)) {
            return HAL_OK;
        }
    }

    for (i = 0U; i < BSP_FDCAN_MAX_RX_SUBSCRIBERS; i++) {
        if (subscribers[i].callback == NULL) {
            subscribers[i].callback = callback;
            subscribers[i].user = user;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/**
 * @brief 分发指定 RX FIFO 中的所有待处理帧。
 *
 * 该函数在 HAL FDCAN 中断回调中调用，回调订阅者时仍处于中断上下文。
 *
 * @param hfdcan 触发回调的 FDCAN 句柄。
 * @param fifo_index HAL 定义的 RX FIFO 编号。
 */
static void dispatch_rx(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo_index)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    bsp_fdcan_slot_t *slot = slot_by_handle(hfdcan);
    bsp_fdcan_rx_subscriber_t *subscribers;
    uint32_t i;

    if (slot == NULL) {
        return;
    }

    subscribers = get_subs(slot, fifo_index);
    if (subscribers == NULL) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(slot->hfdcan, fifo_index) > 0U) {
        if (HAL_FDCAN_GetRxMessage(slot->hfdcan, fifo_index, &rx_header, rx_data) != HAL_OK) {
            return;
        }

        for (i = 0U; i < BSP_FDCAN_MAX_RX_SUBSCRIBERS; i++) {
            if (subscribers[i].callback != NULL) {
                subscribers[i].callback(slot->hfdcan, &rx_header, rx_data, subscribers[i].user);
            }
        }
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_fdcan_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_FDCAN_SLOT_COUNT; i++) {
        /* @diff: 旧版在 HAL 回调里直接解析电机协议；新版初始化只清空接收订阅表。 */
        clear_subs(bsp_fdcan_slots[i].rx_fifo0_subscribers);
        clear_subs(bsp_fdcan_slots[i].rx_fifo1_subscribers);
    }
}

FDCAN_HandleTypeDef *bsp_fdcan_handle(bsp_fdcan_id_t id)
{
    bsp_fdcan_slot_t *slot = slot_by_id(id);

    if (slot == NULL) {
        return NULL;
    }

    return slot->hfdcan;
}

HAL_StatusTypeDef bsp_fdcan_config_std_filter(FDCAN_HandleTypeDef *hfdcan,
                                              uint32_t filter_index,
                                              uint32_t rx_fifo_index,
                                              uint16_t id,
                                              uint16_t mask)
{
    FDCAN_FilterTypeDef filter_config;

    if ((hfdcan == NULL) || (id > BSP_FDCAN_STD_ID_MAX) || (mask > BSP_FDCAN_STD_ID_MAX)) {
        return HAL_ERROR;
    }

    if ((rx_fifo_index != FDCAN_RX_FIFO0) && (rx_fifo_index != FDCAN_RX_FIFO1)) {
        return HAL_ERROR;
    }

    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = filter_index;
    filter_config.FilterType = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = (rx_fifo_index == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter_config.FilterID1 = id;
    filter_config.FilterID2 = mask;

    return HAL_FDCAN_ConfigFilter(hfdcan, &filter_config);
}

HAL_StatusTypeDef bsp_fdcan_accept_std(FDCAN_HandleTypeDef *hfdcan,
                                       uint32_t filter_index,
                                       uint32_t rx_fifo_index)
{
    return bsp_fdcan_config_std_filter(hfdcan, filter_index, rx_fifo_index, 0U, 0U);
}

HAL_StatusTypeDef bsp_fdcan_reject_rest(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) {
        return HAL_ERROR;
    }

    return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT,
                                        FDCAN_FILTER_REMOTE,
                                        FDCAN_FILTER_REMOTE);
}

HAL_StatusTypeDef bsp_fdcan_start(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo_index)
{
    uint32_t notification;

    if (hfdcan == NULL) {
        return HAL_ERROR;
    }

    if ((rx_fifo_index != FDCAN_RX_FIFO0) && (rx_fifo_index != FDCAN_RX_FIFO1)) {
        return HAL_ERROR;
    }

    notification = (rx_fifo_index == FDCAN_RX_FIFO0) ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    if (HAL_FDCAN_ActivateNotification(hfdcan, notification, 0U) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_FDCAN_Start(hfdcan);
}

HAL_StatusTypeDef bsp_fdcan_start_default(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_FDCAN_DEFAULT_COUNT; i++) {
        FDCAN_HandleTypeDef *hfdcan = bsp_fdcan_handle(bsp_fdcan_defaults[i].id);
        if (hfdcan == NULL) {
            return HAL_ERROR;
        }

        if ((bsp_fdcan_accept_std(hfdcan,
                                  bsp_fdcan_defaults[i].filter_index,
                                  bsp_fdcan_defaults[i].rx_fifo) != HAL_OK) ||
            (bsp_fdcan_reject_rest(hfdcan) != HAL_OK) ||
            (bsp_fdcan_start(hfdcan, bsp_fdcan_defaults[i].rx_fifo) != HAL_OK)) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef bsp_fdcan_send_std(FDCAN_HandleTypeDef *hfdcan,
                                     uint16_t std_id,
                                     const uint8_t *data,
                                     uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint32_t dlc;
    uint8_t empty_data = 0U;
    uint8_t *tx_data;

    if ((hfdcan == NULL) || (std_id > BSP_FDCAN_STD_ID_MAX) || ((data == NULL) && (len > 0U))) {
        return HAL_ERROR;
    }

    if ((len > BSP_FDCAN_CLASSIC_DATA_MAX) || (len_to_dlc(len, &dlc) != HAL_OK)) {
        /* @diff: 旧版允许 12/16/20/24/48/64 字节 DLC 但仍使用 Classic CAN；新版 Classic 接口明确限制到 8 字节。 */
        return HAL_ERROR;
    }

    tx_header.Identifier = std_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = dlc;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    tx_data = (data != NULL) ? (uint8_t *)data : &empty_data;

    {
        uint32_t start_tick = HAL_GetTick();
        HAL_StatusTypeDef status;

        for (;;) {
            if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0U) {
                status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
                if (status == HAL_OK) {
                    return HAL_OK;
                }
                if (status != HAL_BUSY) {
                    return status;
                }
            } else {
                g_bsp_fdcan_tx_wait_count++;
            }

            if ((uint32_t)(HAL_GetTick() - start_tick) >= BSP_FDCAN_TX_TIMEOUT_MS) {
                g_bsp_fdcan_tx_timeout_count++;
                return HAL_TIMEOUT;
            }
        }
    }
}

uint8_t bsp_fdcan_dlc_to_length(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:
        return 0U;
    case FDCAN_DLC_BYTES_1:
        return 1U;
    case FDCAN_DLC_BYTES_2:
        return 2U;
    case FDCAN_DLC_BYTES_3:
        return 3U;
    case FDCAN_DLC_BYTES_4:
        return 4U;
    case FDCAN_DLC_BYTES_5:
        return 5U;
    case FDCAN_DLC_BYTES_6:
        return 6U;
    case FDCAN_DLC_BYTES_7:
        return 7U;
    case FDCAN_DLC_BYTES_8:
        return 8U;
    case FDCAN_DLC_BYTES_12:
        return 12U;
    case FDCAN_DLC_BYTES_16:
        return 16U;
    case FDCAN_DLC_BYTES_20:
        return 20U;
    case FDCAN_DLC_BYTES_24:
        return 24U;
    case FDCAN_DLC_BYTES_32:
        return 32U;
    case FDCAN_DLC_BYTES_48:
        return 48U;
    case FDCAN_DLC_BYTES_64:
        return 64U;
    default:
        return 0U;
    }
}

HAL_StatusTypeDef bsp_fdcan_rx_bind(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t fifo_index,
                                    bsp_fdcan_rx_callback_t callback,
                                    void *user)
{
    return bind_rx_cb(hfdcan, fifo_index, callback, user);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    dispatch_rx(hfdcan, FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo1_its)
{
    if ((rx_fifo1_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U) {
        return;
    }

    dispatch_rx(hfdcan, FDCAN_RX_FIFO1);
}
