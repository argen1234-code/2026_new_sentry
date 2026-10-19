/**
 ******************************************************************************
 * @file    bsp_cdc.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-05-01
 * @brief   ST USB_DEVICE CDC ACM 调试输出接口实现。
 *
 * @note    本文件只封装 ST USB Device CDC class，发送接口保持非阻塞，避免日志
 *          输出卡住业务任务。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_cdc.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>

/**
 * @brief CDC 调试输出缓存的最大字节数。
 */
#ifndef BSP_CDC_PENDING_SIZE
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */
#define BSP_CDC_PENDING_SIZE 512U
#endif

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * BSP 层只缓存外设句柄、DMA 缓冲和硬件状态，具体设备用途由 Robot 配置决定。
 * ========================================================================== */

/**
 * @brief 正在交给 USB Device 库发送的缓冲区。
 */
static uint8_t cdc_tx[BSP_CDC_PENDING_SIZE];

/**
 * @brief CDC 忙或主机未打开时保留的最近一条调试输出。
 */
static uint8_t cdc_pending[BSP_CDC_PENDING_SIZE];

/**
 * @brief `cdc_pending` 中有效数据长度。
 */
static uint16_t cdc_pending_len;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */

/**
 * @brief 缓存最近一条 CDC 输出。
 *
 * @param data 待缓存数据。
 * @param length 数据长度，单位 byte。
 */
static void cdc_cache_latest(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return;
    }

    if (length > (uint16_t)sizeof(cdc_pending)) {
        length = (uint16_t)sizeof(cdc_pending);
    }

    memcpy(cdc_pending, data, length);
    cdc_pending_len = length;
}

/* ==========================================================================
 * 收发与回调处理
 * --------------------------------------------------------------------------
 * 这里处理外设收发、DMA/中断回调和 HAL 适配，不在 BSP 内解释具体协议。
 * ========================================================================== */

/**
 * @brief 将数据拷贝到稳定缓冲区并启动一次 CDC 发送。
 *
 * ST USB Device CDC 发送是异步的，调用方传入的日志缓冲区可能很快被复用，
 * 因此这里必须先复制到 `cdc_tx`。
 *
 * @param data 待发送数据。
 * @param length 数据长度，单位 byte。
 * @return HAL_OK 表示已启动发送，否则返回 HAL_BUSY。
 */
static HAL_StatusTypeDef cdc_start_write(const uint8_t *data, uint16_t length)
{
    uint8_t status;

    if ((data == NULL) || (length == 0U)) {
        return HAL_ERROR;
    }

    if ((usbd_cdc_ready() == 0U) || (usbd_cdc_tx_idle() == 0U)) {
        return HAL_BUSY;
    }

    if (length > (uint16_t)sizeof(cdc_tx)) {
        length = (uint16_t)sizeof(cdc_tx);
    }

    memcpy(cdc_tx, data, length);
    status = CDC_Transmit_HS(cdc_tx, length);
    return (status == USBD_OK) ? HAL_OK : HAL_BUSY;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
HAL_StatusTypeDef bsp_cdc_init(void)
{
    /*
     * USB Device 必须由 BSP 阶段显式启动，不能依赖 CubeMX 默认任务的调度顺序。
     * MX_USB_DEVICE_Init() 已做幂等处理，后续默认任务再次调用会直接返回。
     */
    MX_USB_DEVICE_Init();
    return HAL_OK;
}

HAL_StatusTypeDef bsp_cdc_write(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U)) {
        return HAL_ERROR;
    }

    if ((usbd_cdc_ready() == 0U) || (usbd_cdc_tx_idle() == 0U)) {
        cdc_cache_latest(data, length);
        return HAL_BUSY;
    }

    if (cdc_pending_len != 0U) {
        uint8_t pending_is_current = ((cdc_pending_len == length) &&
                                      (memcmp(cdc_pending, data, length) == 0)) ? 1U : 0U;

        status = cdc_start_write(cdc_pending, cdc_pending_len);
        if (status != HAL_OK) {
            cdc_cache_latest(data, length);
            return status;
        }

        cdc_pending_len = 0U;
        if (pending_is_current != 0U) {
            return HAL_OK;
        }

        cdc_cache_latest(data, length);
        return HAL_BUSY;
    }

    status = cdc_start_write(data, length);
    if (status != HAL_OK) {
        cdc_cache_latest(data, length);
    }

    return status;
}

HAL_StatusTypeDef bsp_cdc_write_raw(const uint8_t *data, uint16_t length)
{
    return cdc_start_write(data, length);
}

void bsp_cdc_on_rx(bsp_cdc_rx_fn_t callback, void *user)
{
    usbd_cdc_on_rx(callback, user);
}

uint8_t bsp_cdc_is_ready(void)
{
    return usbd_cdc_ready();
}

uint8_t bsp_cdc_tx_idle(void)
{
    return usbd_cdc_tx_idle();
}
