/**
 ******************************************************************************
 * @file    communication.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   通讯组件公共入口。
 *
 * @note    host 485 通讯由 communication_host.* 实现，默认绑定 USART2 RS485。
 * @note    VOFA 调试输出由 communication_vofa.* 实现，默认绑定 USB CDC ACM。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_H
#define PRINTK_COMPONENT_COMMUNICATION_H

#include "Src/communication_common.h"
#include "Src/communication_board.h"
#include "Src/communication_debug.h"
#include "Src/communication_host.h"
#include "Src/communication_time.h"
#include "Src/communication_vofa.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief 通讯组件配置。
 */
typedef struct
{
    CommunicationHostCfg host; /**< host 485 通讯配置。 */
    CommunicationBoardCfg board; /**< 板间通讯配置。 */
    CommunicationTimeCfg time; /**< 时间同步配置。 */
} CommunicationCfg;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 填充通讯组件默认配置。
 *
 * @param cfg 配置输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_default_cfg(CommunicationCfg *cfg);

/**
 * @brief 初始化通讯组件。
 *
 * 本函数初始化 host 485 通讯，把 VOFA 输出绑定到 USB CDC raw 发送，并接入 Debug LOG。
 *
 * @param cfg 通讯组件配置。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_init(const CommunicationCfg *cfg);

/**
 * @brief 处理一次 host UART DMA idle 接收事件。
 *
 * @param size 本次 DMA idle 收到的字节数。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_rx_event(uint16_t size);

/**
 * @brief 处理一次板间 UART DMA idle 接收事件。
 *
 * @param size 本次 DMA idle 收到的字节数。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_rx_board_event(uint16_t size);

/**
 * @brief 读取最近一次 host 原始自瞄误差。
 *
 * @param out 自瞄误差输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_get_aim(CommunicationAimState *out);

/**
 * @brief 读取最近一次 host probe。
 *
 * @param out probe 输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_get_probe(CommunicationProbe *out);

/**
 * @brief 读取最近一次 host probe ack。
 *
 * @param out probe ack 输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_get_probe_ack(CommunicationProbeAck *out);

/**
 * @brief 发送云台状态快照。
 *
 * @param state 云台状态载荷。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_send_state(const CommunicationGimbalState *state);

/**
 * @brief 发送 RTT probe。
 *
 * @param token 上层生成的 token。
 * @param time_us 发送方时间戳，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_send_probe(uint16_t token, uint32_t time_us);

/**
 * @brief 发送 RTT probe ack。
 *
 * @param token 原始 probe token。
 * @param request_time_us 原始请求时间戳，单位 us。
 * @param reply_time_us 应答发送方时间戳，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_send_probe_ack(uint16_t token,
                                                 uint32_t request_time_us,
                                                 uint32_t reply_time_us);

/**
 * @brief 读取最新云台板间状态。
 *
 * @param out 云台板状态输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_get_board_state(CommunicationBoardState *out);

/**
 * @brief 读取最新底盘板间状态。
 *
 * @param out 底盘板状态输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_get_chassis_state(CommunicationChassisState *out);

/**
 * @brief 发送云台板间状态。
 *
 * @param state 云台板状态。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_send_board_state(const CommunicationBoardState *state);

/**
 * @brief 发送底盘板间状态。
 *
 * @param state 底盘板状态。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_send_chassis_state(const CommunicationChassisState *state);

/**
 * @brief 读取板间通讯诊断。
 *
 * @param out 诊断输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_get_board_diag(CommunicationBoardDiag *out);

/**
 * @brief 读取时间同步状态。
 *
 * @param out 时间同步状态输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_get_time(CommunicationTimeState *out);

/**
 * @brief 将本地时间换算为远端时间估计。
 *
 * @param local_time_us 本地时间，单位 us。
 * @param remote_time_us 远端时间输出，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_remote_now(uint32_t local_time_us, uint32_t *remote_time_us);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_COMMUNICATION_H
