/**
 ******************************************************************************
 * @file    bsp_cdc.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-05-01
 * @brief   ST USB_DEVICE CDC ACM 调试输出接口。
 *
 * @note    本文件只提供 BSP 日志发送包装，USB 枚举和端点收发由 ST USB
 *          Device CDC class 负责。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_CDC_H
#define PRINTK_BSP_CDC_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief CDC 接收回调类型。
 *
 * 该回调运行在 USB Device CDC 接收路径中，只允许做短时间拷贝或置位事件，
 * 不应在回调内解析协议或执行业务逻辑。
 *
 * @param data 接收到的数据。
 * @param length 数据长度，单位 byte。
 * @param user 用户上下文。
 */
typedef void (*bsp_cdc_rx_fn_t)(const uint8_t *data, uint16_t length, void *user);

/**
 * @brief 初始化 USB CDC ACM 调试接口。
 *
 * @return HAL_OK 表示 USB Device CDC 启动入口已经执行。
 */
HAL_StatusTypeDef bsp_cdc_init(void);

/**
 * @brief 发送 CDC 调试数据。
 *
 * @param data 待发送数据。
 * @param length 数据长度，单位 byte。
 * @return HAL_OK 表示数据已交给 USB Device 发送，否则返回 HAL 错误状态。
 */
HAL_StatusTypeDef bsp_cdc_write(const uint8_t *data, uint16_t length);

/**
 * @brief 发送 CDC 二进制协议数据。
 *
 * 与 @ref bsp_cdc_write 不同，本接口不缓存最近一条数据，USB 忙时直接返回
 * HAL_BUSY，适合由上层协议队列自行保证 ACK/ERR 重试和 DATA 丢包统计。
 *
 * @param data 待发送数据。
 * @param length 数据长度，单位 byte。
 * @return HAL_OK 表示已启动发送；HAL_BUSY 表示 CDC 忙或未打开。
 */
HAL_StatusTypeDef bsp_cdc_write_raw(const uint8_t *data, uint16_t length);

/**
 * @brief 注册 CDC 接收回调。
 * @param callback 接收回调；传入 NULL 可取消注册。
 * @param user 用户上下文。
 */
void bsp_cdc_on_rx(bsp_cdc_rx_fn_t callback, void *user);

/**
 * @brief 判断 CDC 是否完成配置。
 *
 * USB 已完成枚举并进入 configured 状态后返回 1。
 *
 * @return 已配置且主机已打开返回 1，否则返回 0。
 */
uint8_t bsp_cdc_is_ready(void);

/**
 * @brief 判断 CDC IN 端点是否可立即发起发送。
 *
 * @return CDC 已打开且 IN 端点空闲返回 1，否则返回 0。
 */
uint8_t bsp_cdc_tx_idle(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_CDC_H
