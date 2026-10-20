/**
 ******************************************************************************
 * @file    bsp_fdcan.h
 * @author  GPT
 * @version V1.3.0
 * @date    2026-05-15
 * @brief   FDCAN 控制器 BSP 抽象接口。
 * @note    差异: 本文件只封装 FDCAN 控制器初始化、过滤器、标准帧发送和 RX FIFO 分发；
 *          不解析电机反馈，不封装 DJI/MIT 电机控制帧，也不维护电机全局状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_FDCAN_H
#define PRINTK_BSP_FDCAN_H

#include "fdcan.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */
#define BSP_FDCAN_MAX_RX_SUBSCRIBERS 4U    /**< 单个 FDCAN RX FIFO 支持的最大订阅者数量。 */
#define BSP_FDCAN_STD_ID_MAX 0x7FFU        /**< 标准 CAN ID 最大值。 */
#define BSP_FDCAN_CLASSIC_DATA_MAX 8U      /**< Classic CAN 最大数据长度。 */

/**
 * @brief 板上可由 BSP 管理的 FDCAN 标识。
 *
 * 新增或删除 FDCAN 实例时，需要同步维护本枚举和 bsp_fdcan.c 中的句柄槽位表。
 */
typedef enum
{
    BSP_FDCAN_ID_FDCAN1 = 0,
    BSP_FDCAN_ID_FDCAN2,
    BSP_FDCAN_ID_FDCAN3,
    BSP_FDCAN_ID_COUNT
} bsp_fdcan_id_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief FDCAN 接收回调类型。
 * @param hfdcan 触发接收事件的 HAL FDCAN 句柄。
 * @param rx_header HAL 接收帧头。
 * @param rx_data 接收数据缓冲区。
 * @param user 用户上下文。
 * @note 回调在 HAL 中断上下文中执行，应保持短小且不阻塞。
 */
typedef void (*bsp_fdcan_rx_callback_t)(FDCAN_HandleTypeDef *hfdcan,
                                        const FDCAN_RxHeaderTypeDef *rx_header,
                                        const uint8_t *rx_data,
                                        void *user);

/**
 * @brief 初始化 FDCAN BSP 内部状态并启动默认 FDCAN 配置。
 * @note 该函数不替代 CubeMX 生成的 MX_FDCANx_Init()。
 */
void bsp_fdcan_init(void);

/**
 * @brief 获取指定 FDCAN 句柄。
 * @param id FDCAN 标识。
 * @return 匹配的 HAL FDCAN 句柄；未找到时返回 NULL。
 */
FDCAN_HandleTypeDef *bsp_fdcan_handle(bsp_fdcan_id_t id);

/**
 * @brief 配置标准 ID mask 过滤器。
 * @param hfdcan HAL FDCAN 句柄。
 * @param filter_index 过滤器索引。
 * @param rx_fifo_index 目标 RX FIFO，使用 FDCAN_FILTER_TO_RXFIFO0/1。
 * @param id 标准 ID。
 * @param mask 标准 ID mask。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_fdcan_config_std_filter(FDCAN_HandleTypeDef *hfdcan,
                                              uint32_t filter_index,
                                              uint32_t rx_fifo_index,
                                              uint16_t id,
                                              uint16_t mask);

/**
 * @brief 配置接收所有标准 ID 帧。
 * @param hfdcan HAL FDCAN 句柄。
 * @param filter_index 过滤器索引。
 * @param rx_fifo_index 目标 RX FIFO，使用 FDCAN_FILTER_TO_RXFIFO0/1。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_fdcan_accept_std(FDCAN_HandleTypeDef *hfdcan,
                                       uint32_t filter_index,
                                       uint32_t rx_fifo_index);

/**
 * @brief 拒收未匹配帧和 remote frame。
 * @param hfdcan HAL FDCAN 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_fdcan_reject_rest(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief 启动 FDCAN 并使能指定 RX FIFO new message 通知。
 * @param hfdcan HAL FDCAN 句柄。
 * @param rx_fifo_index RX FIFO 索引，使用 FDCAN_RX_FIFO0/1。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_fdcan_start(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo_index);

/**
 * @brief 启动当前板级默认 FDCAN 配置。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_fdcan_start_default(void);

/**
 * @brief 发送标准 ID Classic CAN 数据帧。
 * @param hfdcan HAL FDCAN 句柄。
 * @param std_id 标准 CAN ID，范围 0x000-0x7FF。
 * @param data 发送数据指针；len 为 0 时可为 NULL。
 * @param len 数据长度，Classic CAN 最大 8 字节。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_fdcan_send_std(FDCAN_HandleTypeDef *hfdcan,
                                     uint16_t std_id,
                                     const uint8_t *data,
                                     uint8_t len);

/**
 * @brief 发送标准 ID Classic CAN 数据帧，并指定 FIFO 满时的最大等待时间。
 * @param hfdcan HAL FDCAN 句柄。
 * @param std_id 标准 CAN ID，范围 0x000-0x7FF。
 * @param data 发送数据指针；len 为 0 时可为 NULL。
 * @param len 数据长度，Classic CAN 最大 8 字节。
 * @param timeout_ms FIFO 满时的最大等待时间；0 表示不等待。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_fdcan_send_std_timeout(FDCAN_HandleTypeDef *hfdcan,
                                             uint16_t std_id,
                                             const uint8_t *data,
                                             uint8_t len,
                                             uint32_t timeout_ms);

/**
 * @brief 将 FDCAN DLC 转换为字节长度。
 * @param dlc HAL FDCAN DLC 宏值。
 * @return 对应字节长度；未知 DLC 返回 0。
 */
uint8_t bsp_fdcan_dlc_to_length(uint32_t dlc);

/**
 * @brief 绑定 FDCAN RX FIFO 接收回调。
 * @param hfdcan HAL FDCAN 句柄。
 * @param fifo_index RX FIFO 索引，使用 FDCAN_RX_FIFO0/1。
 * @param callback 接收回调；传入 NULL 可清空该 FIFO 的订阅者。
 * @param user 用户上下文。
 * @return HAL_OK 表示绑定成功；HAL_ERROR 表示参数非法、句柄不受管或订阅者已满。
 */
HAL_StatusTypeDef bsp_fdcan_rx_bind(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t fifo_index,
                                    bsp_fdcan_rx_callback_t callback,
                                    void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_FDCAN_H
