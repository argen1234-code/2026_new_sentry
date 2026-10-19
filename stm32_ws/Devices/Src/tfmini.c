/**
 ******************************************************************************
 * @file    tfmini.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   TFmini 测距设备实现。
 *
 * @note    本文件直接实现 TFmini UART 接收、帧解析和最近测量值缓存。
 * @note    TFmini 当前使用标准 9 byte UART 帧：0x59 0x59 + distance + strength +
 *          temperature + checksum。
 * @note    本实现通过 bsp_uart_on() 绑定 RX/ERROR 事件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "tfmini.h"

#include "bsp_uart.h"
#include "main.h"

#if (TFMINI_USE_TIMEBASE == 1)
#include "bsp_timebase.h"
#elif (TFMINI_USE_DWT == 1)
#include "bsp_dwt.h"
#endif

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */
#define TFMINI_FRAME_HEAD 0x59U     /**< TFmini 标准 UART 帧头。 */
#define TFMINI_FRAME_LEN 9U         /**< TFmini 标准 UART 帧长度。 */
#define TFMINI_STRENGTH_MIN 100U    /**< TFmini 有效信号强度下限。 */

/**
 * @brief TFmini 帧解析状态。
 */
typedef enum
{
    TFMINI_PARSE_HEAD1 = 0,
    TFMINI_PARSE_HEAD2,
    TFMINI_PARSE_PAYLOAD
} tfmini_parse_state_t;

/**
 * @brief 板载 TFmini 运行上下文。
 */
typedef struct
{
    UART_HandleTypeDef *huart;          /**< TFmini 使用的 UART 句柄。 */
    uint8_t rx_byte;                    /**< 当前 1 byte 中断接收缓冲。 */
    uint8_t frame[TFMINI_FRAME_LEN];    /**< 当前正在解析的帧缓冲。 */
    uint8_t index;                      /**< 当前帧写入位置。 */
    tfmini_parse_state_t state;         /**< 当前解析状态。 */
    tfmini_meas_t last;                 /**< 最近一次有效测量结果。 */
    volatile uint32_t seq;              /**< 测量结果序号，收到有效帧后递增。 */
    uint8_t ready;                      /**< 设备是否已经初始化。 */
} tfmini_ctx_t;

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */

/**
 * @brief 当前板载 TFmini 上下文。
 */
static tfmini_ctx_t tfmini_ctx;

/**
 * @brief 获取 TFmini 使用的板载 UART 句柄。
 */
static UART_HandleTypeDef *tfmini_uart(void)
{
    return bsp_uart_handle(TFMINI_UART_ID);
}

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 获取当前时间戳。
 */
static uint64_t now_us(void)
{
#if (TFMINI_USE_TIMEBASE == 1)
    return bsp_timebase_now_us();
#elif (TFMINI_USE_DWT == 1)
    return bsp_dwt_get_timeline_us();
#else
    return ((uint64_t)HAL_GetTick()) * 1000ULL;
#endif
}

/**
 * @brief 计算 TFmini 帧校验和。
 */
static uint8_t checksum8(const uint8_t *buffer, uint8_t length)
{
    uint8_t sum = 0U;
    uint8_t i;

    if (buffer == NULL) {
        return 0U;
    }

    for (i = 0U; i < length; ++i) {
        sum = (uint8_t)(sum + buffer[i]);
    }

    return sum;
}

/**
 * @brief 进入极短临界区并返回进入前 PRIMASK。
 */
static uint32_t save_irq(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 恢复极短临界区进入前的中断状态。
 */
static void restore_irq(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 判断上下文是否已经初始化。
 */
static int data_ready(void)
{
    return (tfmini_ctx.ready != 0U) && (tfmini_ctx.huart != NULL);
}

/**
 * @brief 启动 1 byte UART 中断接收。
 */
static HAL_StatusTypeDef start_read(void)
{
    bsp_uart_rx_t rx;

    if (data_ready() == 0) {
        return HAL_ERROR;
    }

    rx.data = &tfmini_ctx.rx_byte;
    rx.size = 1U;
    rx.timeout = 0U;
    rx.mode = BSP_UART_MODE_IT;

    return bsp_uart_rx(tfmini_ctx.huart, &rx);
}

/**
 * @brief 中止当前 UART 接收。
 */
static HAL_StatusTypeDef abort_read(void)
{
    if (tfmini_ctx.huart == NULL) {
        return HAL_ERROR;
    }

    return bsp_uart_abort_rx(tfmini_ctx.huart);
}

/**
 * @brief 将有效 TFmini 帧转换为测量值。
 */
static void update_meas(const uint8_t *frame)
{
    tfmini_meas_t measurement;
    uint16_t temp_raw;
    int32_t temp_cdeg;

    measurement.distance_cm = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
    measurement.strength = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
    temp_raw = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
    temp_cdeg = (((int32_t)temp_raw * 100) / 8) - 25600;
    measurement.temp_cdeg = (int16_t)temp_cdeg;
    measurement.valid = ((measurement.strength >= TFMINI_STRENGTH_MIN) &&
                         (measurement.strength != 0xFFFFU)) ? 1U : 0U;
    measurement.timestamp_us = now_us();

    tfmini_ctx.last = measurement;
    ++tfmini_ctx.seq;
}

/**
 * @brief 复位当前帧解析过程。
 */
static void reset_parser(void)
{
    tfmini_ctx.index = 0U;
    tfmini_ctx.state = TFMINI_PARSE_HEAD1;
}

/**
 * @brief 向 TFmini 解析器输入一个字节。
 */
static void parse_byte(uint8_t byte)
{
    switch (tfmini_ctx.state) {
    case TFMINI_PARSE_HEAD1:
        if (byte == TFMINI_FRAME_HEAD) {
            tfmini_ctx.frame[0] = byte;
            tfmini_ctx.state = TFMINI_PARSE_HEAD2;
        }
        break;

    case TFMINI_PARSE_HEAD2:
        if (byte == TFMINI_FRAME_HEAD) {
            tfmini_ctx.frame[1] = byte;
            tfmini_ctx.index = 2U;
            tfmini_ctx.state = TFMINI_PARSE_PAYLOAD;
        } else {
            reset_parser();
        }
        break;

    case TFMINI_PARSE_PAYLOAD:
        tfmini_ctx.frame[tfmini_ctx.index] = byte;
        ++tfmini_ctx.index;

        if (tfmini_ctx.index >= TFMINI_FRAME_LEN) {
            if (checksum8(tfmini_ctx.frame, TFMINI_FRAME_LEN - 1U) == tfmini_ctx.frame[TFMINI_FRAME_LEN - 1U]) {
                update_meas(tfmini_ctx.frame);
            }

            reset_parser();
        }
        break;

    default:
        reset_parser();
        break;
    }
}

/**
 * @brief 处理 UART 接收完成事件。
 */
static void uart_rx(UART_HandleTypeDef *huart, void *user)
{
    (void)user;

    if ((data_ready() == 0) || (huart != tfmini_ctx.huart)) {
        return;
    }

    parse_byte(tfmini_ctx.rx_byte);
    (void)start_read();
}

/**
 * @brief 处理 UART 错误事件。
 */
static void uart_error(UART_HandleTypeDef *huart, void *user)
{
    (void)user;

    if ((data_ready() == 0) || (huart != tfmini_ctx.huart)) {
        return;
    }

    reset_parser();
    (void)abort_read();
    (void)start_read();
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
tfmini_status_t tfmini_init(void)
{
    UART_HandleTypeDef *huart = tfmini_uart();

    if (huart == NULL) {
        return TFMINI_ERR_IO;
    }

    (void)memset(&tfmini_ctx, 0, sizeof(tfmini_ctx));
    tfmini_ctx.huart = huart;
    tfmini_ctx.ready = 1U;
    reset_parser();

    if (bsp_uart_on(huart, BSP_UART_EVENT_RX_CPLT, uart_rx, NULL) != HAL_OK) {
        tfmini_ctx.ready = 0U;
        return TFMINI_ERR_IO;
    }

    if (bsp_uart_on(huart, BSP_UART_EVENT_ERROR, uart_error, NULL) != HAL_OK) {
        (void)bsp_uart_on(huart, BSP_UART_EVENT_RX_CPLT, NULL, NULL);
        tfmini_ctx.ready = 0U;
        return TFMINI_ERR_IO;
    }

    (void)abort_read();
    if (start_read() != HAL_OK) {
        (void)bsp_uart_on(huart, BSP_UART_EVENT_RX_CPLT, NULL, NULL);
        (void)bsp_uart_on(huart, BSP_UART_EVENT_ERROR, NULL, NULL);
        tfmini_ctx.ready = 0U;
        return TFMINI_ERR_IO;
    }

    return TFMINI_OK;
}

bool tfmini_ready(void)
{
    return data_ready() != 0;
}

bool tfmini_online(uint32_t timeout_ms)
{
    uint32_t primask;
    uint64_t timestamp_us;
    uint32_t seq;

    if (data_ready() == 0) {
        return false;
    }

    primask = save_irq();
    seq = tfmini_ctx.seq;
    timestamp_us = tfmini_ctx.last.timestamp_us;
    restore_irq(primask);

    if (seq == 0U) {
        return false;
    }

    return (now_us() - timestamp_us) <= ((uint64_t)timeout_ms * 1000ULL);
}

bool tfmini_read(tfmini_meas_t *out, uint32_t *io_seq)
{
    uint32_t primask;
    uint32_t seq;
    tfmini_meas_t measurement;

    if ((data_ready() == 0) || (out == NULL) || (io_seq == NULL)) {
        return false;
    }

    primask = save_irq();
    seq = tfmini_ctx.seq;
    measurement = tfmini_ctx.last;
    restore_irq(primask);

    if ((seq == 0U) || (seq == *io_seq)) {
        return false;
    }

    *out = measurement;
    *io_seq = seq;
    return true;
}

bool tfmini_peek(tfmini_meas_t *out)
{
    uint32_t primask;

    if ((data_ready() == 0) || (out == NULL) || (tfmini_ctx.seq == 0U)) {
        return false;
    }

    primask = save_irq();
    *out = tfmini_ctx.last;
    restore_irq(primask);

    return true;
}

void tfmini_reset(void)
{
    reset_parser();
}

tfmini_status_t tfmini_raw(const uint8_t *data, uint16_t length)
{
    bsp_uart_tx_t tx;

    if ((data_ready() == 0)) {
        return TFMINI_ERR_NOT_READY;
    }

    if ((data == NULL) || (length == 0U)) {
        return TFMINI_ERR_ARG;
    }

    tx.data = data;
    tx.size = length;
    tx.timeout = TFMINI_UART_TX_TIMEOUT_MS;
    tx.mode = BSP_UART_MODE_POLL;

    return (bsp_uart_tx(tfmini_ctx.huart, &tx) == HAL_OK) ? TFMINI_OK : TFMINI_ERR_IO;
}
