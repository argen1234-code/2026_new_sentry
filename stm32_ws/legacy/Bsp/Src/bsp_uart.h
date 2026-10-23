/**
 ******************************************************************************
 * @file    bsp_uart.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   UART BSP 抽象接口。
 *
 * @note    本文件只提供 UART 句柄获取、HAL 收发事务包装和 HAL 回调分发。
 *          遥控器、裁判系统、板间通信、测距模块等协议解析应放在设备层。
 * @note    与旧版 bsp_usart 相比，新版不再导出协议 buffer、业务结构体
 *          或 USARTx_Transmit 这类设备混合接口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_UART_H
#define PRINTK_BSP_UART_H

#include "usart.h"

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
 * @brief 板上可由 BSP 管理的 UART 标识。
 *
 * 新增或删除 UART 实例时，需要同步维护：
 * 1. 本枚举；
 * 2. bsp_uart.c 中的 bsp_uart_slots[]。
 */
typedef enum
{
    BSP_UART_ID_USART2 = 0, /**< USART2。 */
    BSP_UART_ID_USART1,     /**< USART1。 */
    BSP_UART_ID_USART3,     /**< USART3。 */
    BSP_UART_ID_UART5,      /**< UART5。 */
    BSP_UART_ID_UART7,      /**< UART7。 */
    BSP_UART_ID_UART8,      /**< UART8。 */
    BSP_UART_ID_UART9,      /**< UART9。 */
    BSP_UART_ID_USART10,    /**< USART10。 */
    BSP_UART_ID_COUNT       /**< UART 标识数量。 */
} bsp_uart_id_t;

/**
 * @brief UART 传输模式。
 */
typedef enum
{
    BSP_UART_MODE_POLL = 0,      /**< 阻塞模式。 */
    BSP_UART_MODE_IT,            /**< 中断模式。 */
    BSP_UART_MODE_DMA,           /**< DMA 模式。 */
    BSP_UART_MODE_DMA_TO_IDLE    /**< DMA + idle 接收模式，仅用于 RX。 */
} bsp_uart_mode_t;

/**
 * @brief UART 事件类型。
 */
typedef enum
{
    BSP_UART_EVENT_RX_CPLT = 0,    /**< 接收完成。 */
    BSP_UART_EVENT_RX_IDLE,        /**< idle 接收事件。 */
    BSP_UART_EVENT_TX_CPLT,        /**< 发送完成。 */
    BSP_UART_EVENT_ERROR,          /**< 错误事件。 */
    BSP_UART_EVENT_COUNT
} bsp_uart_event_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief UART 普通事件回调函数类型。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param user 注册回调时传入的用户上下文。
 */
typedef void (*bsp_uart_event_callback_t)(UART_HandleTypeDef *huart, void *user);

/**
 * @brief UART idle 接收事件回调函数类型。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param size 本次接收到的数据长度，单位为字节。
 * @param user 注册回调时传入的用户上下文。
 */
typedef void (*bsp_uart_rx_idle_callback_t)(UART_HandleTypeDef *huart, uint16_t size, void *user);

/**
 * @brief UART 发送事务。
 */
typedef struct
{
    const uint8_t *data;     /**< 待发送数据缓冲区。 */
    uint16_t size;           /**< 待发送长度，单位为字节。 */
    uint32_t timeout;        /**< 阻塞模式超时时间，单位为毫秒。 */
    bsp_uart_mode_t mode;    /**< 发送模式，支持 POLL/IT/DMA。 */
} bsp_uart_tx_t;

/**
 * @brief UART 接收事务。
 */
typedef struct
{
    uint8_t *data;           /**< 接收缓冲区。 */
    uint16_t size;           /**< 接收长度或缓冲区长度，单位为字节。 */
    uint32_t timeout;        /**< 阻塞模式超时时间，单位为毫秒。 */
    bsp_uart_mode_t mode;    /**< 接收模式，支持 POLL/IT/DMA/DMA_TO_IDLE。 */
} bsp_uart_rx_t;

/**
 * @brief 初始化 UART BSP 回调表。
 *
 * @note 差异: 旧版在 BSP 内部直接处理各 UART 业务；新版只清空回调注册状态。
 */
void bsp_uart_init(void);

/**
 * @brief 根据 BSP UART 标识获取 UART 句柄。
 *
 * @param id BSP UART 标识。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
UART_HandleTypeDef *bsp_uart_handle(bsp_uart_id_t id);

/**
 * @brief 根据外设实例地址获取 UART 句柄。
 *
 * @param instance UART 外设实例，例如 `USART2`、`UART5`。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
UART_HandleTypeDef *bsp_uart_from_instance(USART_TypeDef *instance);

/**
 * @brief 判断 UART 句柄是否由本 BSP 管理。
 *
 * @param huart 待检查的 UART 句柄。
 * @return 受管理返回 1，否则返回 0。
 */
uint8_t bsp_uart_is_supported(UART_HandleTypeDef *huart);

/**
 * @brief 执行 UART 发送事务。
 *
 * @param huart 目标 UART 句柄。
 * @param tx 发送事务配置。
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, const bsp_uart_tx_t *tx);

/**
 * @brief 执行 UART 接收事务。
 *
 * @param huart 目标 UART 句柄。
 * @param rx 接收事务配置。
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef bsp_uart_rx(UART_HandleTypeDef *huart, const bsp_uart_rx_t *rx);

/**
 * @brief 中止 UART 接收。
 *
 * @param huart 目标 UART 句柄。
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef bsp_uart_abort_rx(UART_HandleTypeDef *huart);

/**
 * @brief 中止 UART 发送。
 *
 * @param huart 目标 UART 句柄。
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef bsp_uart_abort_tx(UART_HandleTypeDef *huart);

/**
 * @brief 中止 UART 收发。
 *
 * @param huart 目标 UART 句柄。
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef bsp_uart_abort(UART_HandleTypeDef *huart);

/**
 * @brief 绑定 UART 普通事件回调。
 *
 * 本接口用于 RX_CPLT/TX_CPLT/ERROR 等不携带额外参数的事件；传入 `NULL` 表示取消绑定。
 *
 * @param huart 目标 UART 句柄。
 * @param event UART 事件类型。
 * @param callback 事件回调函数指针。
 * @param user 用户上下文。
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef bsp_uart_on(UART_HandleTypeDef *huart,
                              bsp_uart_event_t event,
                              bsp_uart_event_callback_t callback,
                              void *user);

/**
 * @brief 绑定 UART idle 接收事件回调。
 *
 * 传入 `NULL` 表示取消绑定。
 *
 * @param huart 目标 UART 句柄。
 * @param callback idle 接收事件回调函数指针。
 * @param user 用户上下文。
 * @return 返回 HAL UART 状态码。
 */
HAL_StatusTypeDef bsp_uart_on_rx_idle(UART_HandleTypeDef *huart,
                                      bsp_uart_rx_idle_callback_t callback,
                                      void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_UART_H
