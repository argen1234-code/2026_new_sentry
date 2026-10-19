/**
 ******************************************************************************
 * @file    ws2812.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   WS2812 灯带设备接口。
 *
 * @note    本文件定义使用配置 SPI 驱动 WS2812 的板级设备接口。BSP 只暴露 SPI 句柄，
 *          颜色编码、复位码流和灯带缓存由本设备层负责。
 * @note    当前实现使用 SPI 4-bit 数据帧：WS2812 的 0 编码为 0x8，1 编码为
 *          0xE。SPI 时钟需要由 CubeMX 配到接近 3.2 MHz。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_WS2812_H
#define PRINTK_DEVICE_WS2812_H

#include "robot_board.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */
/** @brief 单个 WS2812 灯珠的颜色位数。 */
#define WS2812_BITS_PER_LED ((uint16_t)24U)

/** @brief 计算指定灯珠数量需要的 SPI 发送缓存长度。 */
#define WS2812_BUF_LEN(count) ((uint16_t)(((uint16_t)(count) * WS2812_BITS_PER_LED) + WS2812_RESET_WORDS))

/**
 * @brief WS2812 设备状态码。
 */
typedef enum
{
    Ws2812Ok = 0,    /**< 操作成功。 */
    Ws2812Arg = -1,  /**< 输入参数无效。 */
    Ws2812Cfg = -2,  /**< SPI 配置不符合当前编码要求。 */
    Ws2812Busy = -3, /**< SPI 正忙。 */
    Ws2812Io = -4    /**< 底层 SPI 发送失败。 */
} Ws2812Status;

/**
 * @brief WS2812 颜色通道顺序。
 */
typedef enum
{
    Ws2812OrderGrb = 0, /**< WS2812 常用 GRB 顺序。 */
    Ws2812OrderRgb      /**< 标准 RGB 顺序。 */
} Ws2812Order;

/**
 * @brief WS2812 RGB 颜色。
 */
typedef struct
{
    uint8_t r; /**< 红色通道。 */
    uint8_t g; /**< 绿色通道。 */
    uint8_t b; /**< 蓝色通道。 */
} Ws2812Color;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化 WS2812 设备。
 *
 * @param led_count 实际使用的灯珠数量，范围为 1 到 WS2812_LED_MAX。
 * @return 返回 WS2812 设备状态码。
 */
Ws2812Status ws2812_init(uint16_t led_count);

/**
 * @brief 判断 WS2812 设备是否已初始化。
 *
 * @return 已初始化返回 1，否则返回 0。
 */
uint8_t ws2812_ready(void);

/**
 * @brief 获取当前初始化的灯珠数量。
 *
 * @return 返回灯珠数量；未初始化时返回 0。
 */
uint16_t ws2812_count(void);

/**
 * @brief 设置 WS2812 颜色通道顺序。
 *
 * @param order 颜色通道顺序。
 * @return 返回 WS2812 设备状态码。
 */
Ws2812Status ws2812_order(Ws2812Order order);

/**
 * @brief 设置单个灯珠颜色。
 *
 * @param index 灯珠序号，从 0 开始。
 * @param color RGB 颜色。
 * @return 返回 WS2812 设备状态码。
 */
Ws2812Status ws2812_set(uint16_t index, Ws2812Color color);

/**
 * @brief 按 RGB 分量设置单个灯珠颜色。
 *
 * @param index 灯珠序号，从 0 开始。
 * @param r 红色通道。
 * @param g 绿色通道。
 * @param b 蓝色通道。
 * @return 返回 WS2812 设备状态码。
 */
Ws2812Status ws2812_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 填充所有灯珠为同一颜色。
 *
 * @param color RGB 颜色。
 * @return 返回 WS2812 设备状态码。
 */
Ws2812Status ws2812_fill(Ws2812Color color);

/**
 * @brief 清空所有灯珠颜色缓存为黑色。
 *
 * @return 返回 WS2812 设备状态码。
 */
Ws2812Status ws2812_clear(void);

/**
 * @brief 发送当前颜色缓存到 WS2812。
 *
 * @return 返回 WS2812 设备状态码。
 */
Ws2812Status ws2812_show(void);

/**
 * @brief 获取最近一次 WS2812 操作状态。
 *
 * @return 返回最近一次状态码。
 */
Ws2812Status ws2812_status(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_WS2812_H
