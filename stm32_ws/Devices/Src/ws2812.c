/**
 ******************************************************************************
 * @file    ws2812.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   WS2812 灯带设备实现。
 *
 * @note    本文件把 WS2812 的 RGB 数据编码成 SPI 4-bit 符号流，再通过
 *          `bsp_spi_transfer()` 阻塞发送。
 * @note    本文件不修改 CubeMX 生成配置；配置 SPI 必须已经完成 CubeMX 初始化，
 *          且数据位宽为 4-bit。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "ws2812.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */
/** @brief WS2812 逻辑 0 的 SPI 4-bit 编码。 */
#define WS2812_SYMBOL_ZERO ((uint8_t)0x08U)

/** @brief WS2812 逻辑 1 的 SPI 4-bit 编码。 */
#define WS2812_SYMBOL_ONE ((uint8_t)0x0EU)

/** @brief WS2812 复位低电平的 SPI 4-bit 编码。 */
#define WS2812_SYMBOL_RESET ((uint8_t)0x00U)

/** @brief 当前板级 WS2812 发送缓存长度。 */
#define WS2812_TX_BUF_SIZE WS2812_BUF_LEN(WS2812_LED_MAX)

/**
 * @brief WS2812 设备上下文。
 */
typedef struct
{
    SPI_HandleTypeDef *hspi;                  /**< 绑定的 SPI 句柄。 */
    uint16_t led_count;                       /**< 实际灯珠数量。 */
    uint16_t tx_len;                          /**< 当前需要发送的 SPI 4-bit 帧数量。 */
    Ws2812Order order;                        /**< 颜色通道顺序。 */
    Ws2812Status last_status;                 /**< 最近一次操作状态。 */
    uint8_t ready;                            /**< 初始化完成标志。 */
    uint8_t tx_buf[WS2812_TX_BUF_SIZE];       /**< SPI 4-bit 符号发送缓存。 */
} Ws2812Dev;

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */
/** @brief 当前板级 WS2812 固定设备实例。 */
static Ws2812Dev ws2812_dev;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 写入并返回最近一次操作状态。
 *
 * @param status 状态码。
 * @return 原样返回状态码。
 */
static Ws2812Status set_status(Ws2812Status status)
{
    ws2812_dev.last_status = status;
    return status;
}

/**
 * @brief 获取 WS2812 使用的 SPI 句柄。
 *
 * @return SPI 句柄，失败返回 NULL。
 */
static SPI_HandleTypeDef *ws2812_spi(void)
{
    return bsp_spi_handle(WS2812_SPI_ID);
}

/**
 * @brief 检查 SPI 当前配置是否满足 4-bit 符号发送。
 *
 * @param hspi SPI 句柄。
 * @return 满足返回 1，否则返回 0。
 */
static uint8_t spi_cfg_ok(const SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) {
        return 0U;
    }

    return (hspi->Init.DataSize == SPI_DATASIZE_4BIT) ? 1U : 0U;
}

/**
 * @brief 获取指定灯珠在发送缓存中的起始位置。
 *
 * @param index 灯珠序号。
 * @return 发送缓存指针。
 */
static uint8_t *led_ptr(uint16_t index)
{
    return &ws2812_dev.tx_buf[(uint16_t)(index * WS2812_BITS_PER_LED)];
}

/**
 * @brief 将 1 byte 颜色通道编码为 8 个 SPI 4-bit 符号。
 *
 * @param value 待编码颜色值。
 * @param out 输出缓存，长度至少为 8。
 */
static void encode_byte(uint8_t value, uint8_t *out)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        /*
         * WS2812 先发高位。这里每个颜色 bit 对应一个 SPI 4-bit 帧，
         * SPI MOSI 在一个 WS2812 bit 周期内输出 1110 或 1000。
         */
        out[bit] = ((value & (uint8_t)(0x80U >> bit)) != 0U) ? WS2812_SYMBOL_ONE : WS2812_SYMBOL_ZERO;
    }
}

/**
 * @brief 按当前颜色顺序编码一个灯珠。
 *
 * @param index 灯珠序号。
 * @param color RGB 颜色。
 */
static void encode_color(uint16_t index, Ws2812Color color)
{
    uint8_t *out = led_ptr(index);

    if (ws2812_dev.order == Ws2812OrderRgb) {
        encode_byte(color.r, &out[0]);
        encode_byte(color.g, &out[8]);
        encode_byte(color.b, &out[16]);
    } else {
        encode_byte(color.g, &out[0]);
        encode_byte(color.r, &out[8]);
        encode_byte(color.b, &out[16]);
    }
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 判断设备是否可执行颜色缓存操作。
 *
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t ctx_ready(void)
{
    return ((ws2812_dev.ready != 0U) && (ws2812_dev.hspi != NULL)) ? 1U : 0U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
Ws2812Status ws2812_init(uint16_t led_count)
{
    uint32_t tx_len;

    if ((led_count == 0U) || (led_count > WS2812_LED_MAX)) {
        return set_status(Ws2812Arg);
    }

    memset(&ws2812_dev, 0, sizeof(ws2812_dev));
    ws2812_dev.hspi = ws2812_spi();
    ws2812_dev.order = Ws2812OrderGrb;
    ws2812_dev.led_count = led_count;

    tx_len = ((uint32_t)led_count * (uint32_t)WS2812_BITS_PER_LED) + (uint32_t)WS2812_RESET_WORDS;
    if ((tx_len == 0UL) || (tx_len > (uint32_t)WS2812_TX_BUF_SIZE) || (tx_len > 0xFFFFUL)) {
        return set_status(Ws2812Arg);
    }

    if (spi_cfg_ok(ws2812_dev.hspi) == 0U) {
        return set_status(Ws2812Cfg);
    }

    ws2812_dev.tx_len = (uint16_t)tx_len;
    memset(ws2812_dev.tx_buf, WS2812_SYMBOL_RESET, sizeof(ws2812_dev.tx_buf));
    ws2812_dev.ready = 1U;

    return ws2812_clear();
}

uint8_t ws2812_ready(void)
{
    return ctx_ready();
}

uint16_t ws2812_count(void)
{
    return (ctx_ready() != 0U) ? ws2812_dev.led_count : 0U;
}

Ws2812Status ws2812_order(Ws2812Order order)
{
    if (ctx_ready() == 0U) {
        return set_status(Ws2812Cfg);
    }
    if ((order != Ws2812OrderGrb) && (order != Ws2812OrderRgb)) {
        return set_status(Ws2812Arg);
    }

    ws2812_dev.order = order;
    return set_status(Ws2812Ok);
}

Ws2812Status ws2812_set(uint16_t index, Ws2812Color color)
{
    if (ctx_ready() == 0U) {
        return set_status(Ws2812Cfg);
    }
    if (index >= ws2812_dev.led_count) {
        return set_status(Ws2812Arg);
    }

    encode_color(index, color);
    return set_status(Ws2812Ok);
}

Ws2812Status ws2812_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    Ws2812Color color;

    color.r = r;
    color.g = g;
    color.b = b;

    return ws2812_set(index, color);
}

Ws2812Status ws2812_fill(Ws2812Color color)
{
    uint16_t i;

    if (ctx_ready() == 0U) {
        return set_status(Ws2812Cfg);
    }

    for (i = 0U; i < ws2812_dev.led_count; i++) {
        encode_color(i, color);
    }

    return set_status(Ws2812Ok);
}

Ws2812Status ws2812_clear(void)
{
    Ws2812Color black;

    black.r = 0U;
    black.g = 0U;
    black.b = 0U;

    return ws2812_fill(black);
}

Ws2812Status ws2812_show(void)
{
    bsp_spi_transfer_t xfer;
    HAL_StatusTypeDef status;

    if (ctx_ready() == 0U) {
        return set_status(Ws2812Cfg);
    }

    if (bsp_spi_get_state(ws2812_dev.hspi) != HAL_SPI_STATE_READY) {
        return set_status(Ws2812Busy);
    }

    xfer.tx_data = ws2812_dev.tx_buf;
    xfer.rx_data = NULL;
    xfer.size = ws2812_dev.tx_len;
    xfer.timeout = WS2812_TX_TIMEOUT_MS;
    xfer.mode = BSP_SPI_MODE_POLL;

    status = bsp_spi_transfer(ws2812_dev.hspi, &xfer);
    return set_status((status == HAL_OK) ? Ws2812Ok : Ws2812Io);
}

Ws2812Status ws2812_status(void)
{
    return ws2812_dev.last_status;
}
