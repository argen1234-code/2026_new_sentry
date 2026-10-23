/**
 ******************************************************************************
 * @file    communication_common.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   通讯组件公共状态与默认参数。
 *
 * @note    本文件只放 Component/Communication 内部共享的公共语言。
 * @note    具体 485 host 协议和 VOFA 输出分别位于 communication_host.* 与
 *          communication_vofa.*。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_COMMON_H
#define PRINTK_COMPONENT_COMMUNICATION_COMMON_H

#include "robot_board.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief host 自瞄 yaw 参考有效位。 */
#define COMM_HOST_AIM_YAW_VALID ((uint8_t)0x01U)

/** @brief host 自瞄 pitch 参考有效位。 */
#define COMM_HOST_AIM_PITCH_VALID ((uint8_t)0x02U)

/** @brief host 接收 DMA 缓冲区默认长度，单位 byte。 */
#ifndef COMM_HOST_RX_SIZE
#define COMM_HOST_RX_SIZE 128U
#endif

/** @brief 板间通信接收 DMA 缓冲区默认长度，单位 byte。 */
#ifndef COMM_BOARD_RX_SIZE
#define COMM_BOARD_RX_SIZE 128U
#endif

/** @brief 板间通信默认启用状态，未配置独立 UART 时保持关闭。 */
#ifndef COMM_BOARD_ENABLE_DEFAULT
#define COMM_BOARD_ENABLE_DEFAULT 0U
#endif

/** @brief 板间通讯默认 UART。 */
#ifndef COMM_BOARD_UART_ID
#define COMM_BOARD_UART_ID BSP_UART_ID_USART3
#endif

/** @brief VOFA 打包缓冲区默认长度，单位 byte。 */
#ifndef COMM_VOFA_TX_SIZE
#define COMM_VOFA_TX_SIZE 256U
#endif

/** @brief host 自瞄输入默认优先级。 */
#ifndef COMM_HOST_AIM_PRIORITY
#define COMM_HOST_AIM_PRIORITY 80U
#endif

/**
 * @brief 通讯组件状态码。
 */
typedef enum
{
    CommunicationOk = 0,       /**< 操作成功。 */
    CommunicationNull = 1,     /**< 输入指针为空。 */
    CommunicationArg = 2,      /**< 输入参数非法。 */
    CommunicationState = 3,    /**< 当前组件状态不允许执行。 */
    CommunicationIo = 4,       /**< 底层传输失败。 */
    CommunicationBusy = 5,     /**< 底层传输忙。 */
    CommunicationSmall = 6,    /**< 输出缓冲区不足。 */
    CommunicationLength = 7,   /**< 协议长度非法。 */
    CommunicationId = 8,       /**< 协议消息 ID 非法。 */
    CommunicationCrc = 9,      /**< 协议 CRC 校验失败。 */
    CommunicationDirection = 10, /**< 协议方向与 endpoint 角色不匹配。 */
    CommunicationDecode = 11   /**< payload 解码失败。 */
} CommunicationStatus;

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_COMMUNICATION_COMMON_H
