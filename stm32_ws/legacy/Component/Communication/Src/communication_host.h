/**
 ******************************************************************************
 * @file    communication_host.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   485 host 实时通讯接口。
 *
 * @note    本文件绑定 USART2 RS485 与 Proto2 RT 核，只负责收发和缓存协议数据。
 * @note    HostAim 只缓存原始视觉误差；RTT 只暴露 probe/ack 数据，不计算延迟。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_HOST_H
#define PRINTK_COMPONENT_COMMUNICATION_HOST_H

#include "communication_common.h"
#include "Generate/proto2_rt.h"
#include "bsp_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief host 通讯配置。
 */
typedef struct
{
    bsp_uart_id_t uart_id; /**< host 485 通讯使用的 UART 标识。 */
    uint8_t aim_priority;  /**< 写入 raw aim 缓存的默认优先级。 */
} CommunicationHostCfg;

/**
 * @brief host 原始自瞄误差。
 */
typedef struct
{
    float yaw_err_rad;     /**< yaw 视觉误差，单位 rad。 */
    float pitch_err_rad;   /**< pitch 视觉误差，单位 rad。 */
    uint32_t host_time_us; /**< host 侧时间戳，单位 us。 */
    uint32_t tick_ms;      /**< host 侧时间戳换算的 ms。 */
    uint32_t seq;          /**< 本地缓存递增序号，用于判断是否有新数据。 */
    uint8_t valid_mask;    /**< COMM_HOST_AIM_* 有效位。 */
    uint8_t enable;        /**< 非 0 表示视觉源参与输入链路。 */
    uint8_t priority;      /**< 输入仲裁优先级。 */
} CommunicationAimState;

/**
 * @brief 云台状态上报载荷。
 */
typedef struct
{
    float yaw_rad;          /**< yaw 角，单位 rad。 */
    float pitch_rad;        /**< pitch 角，单位 rad。 */
    float yaw_rate_rad_s;   /**< yaw 角速度，单位 rad/s。 */
    float pitch_rate_rad_s; /**< pitch 角速度，单位 rad/s。 */
    uint8_t status;         /**< 上层定义的云台状态字。 */
    uint32_t time_us;       /**< MCU 侧时间戳，单位 us。 */
} CommunicationGimbalState;

/**
 * @brief RTT probe 数据。
 */
typedef struct
{
    uint16_t token;   /**< 上层生成的 probe token。 */
    uint32_t time_us; /**< probe 携带的时间戳，单位 us。 */
    uint32_t seq;     /**< 本地缓存递增序号，用于判断是否有新数据。 */
} CommunicationProbe;

/**
 * @brief RTT probe ack 数据。
 */
typedef struct
{
    uint16_t token;           /**< 原始 probe token。 */
    uint32_t request_time_us; /**< 原始请求时间戳，单位 us。 */
    uint32_t reply_time_us;   /**< 应答发送方时间戳，单位 us。 */
    uint32_t seq;             /**< 本地缓存递增序号，用于判断是否有新数据。 */
} CommunicationProbeAck;

/**
 * @brief host 通讯诊断快照。
 */
typedef struct
{
    uint8_t ready;               /**< 非 0 表示 host 通道已经初始化。 */
    uint8_t last_status;         /**< 最近一次组件状态码。 */
    uint8_t last_restart_status; /**< 最近一次 DMA idle 接收重启状态。 */
    uint8_t last_msg_id;         /**< 最近一次合法帧消息 ID。 */
    uint16_t last_rx_size;       /**< 最近一次 idle 事件收到的字节数。 */
    uint32_t rx_events;          /**< UART idle 接收事件数量。 */
    uint32_t rx_frames;          /**< Proto2 RT 合法帧数量。 */
    uint32_t crc_errors;         /**< CRC 错误数量。 */
    uint32_t id_errors;          /**< 未知消息 ID 数量。 */
    uint32_t seq_errors;         /**< 序号不连续数量。 */
    uint32_t direction_errors;   /**< 方向错误数量。 */
    uint32_t decode_errors;      /**< payload 解码错误数量。 */
    uint32_t uart_errors;        /**< UART 错误事件数量。 */
    uint32_t aim_count;          /**< HostAim 接收数量。 */
    uint32_t probe_count;        /**< RtProbe 接收数量。 */
    uint32_t ack_count;          /**< RtProbeAck 接收数量。 */
    uint32_t tx_frames;          /**< 发送成功帧数量。 */
    uint32_t tx_errors;          /**< 发送失败数量。 */
    uint32_t restart_fail_count; /**< DMA idle 接收重启失败次数。 */
} CommunicationHostDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 填充 host 通讯默认配置。
 *
 * 默认使用 USART2 RS485，自瞄优先级使用 COMM_HOST_AIM_PRIORITY。
 *
 * @param cfg 配置输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_default_cfg(CommunicationHostCfg *cfg);

/**
 * @brief 初始化 host 485 通讯。
 *
 * @param cfg host 通讯配置。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_init(const CommunicationHostCfg *cfg);

/**
 * @brief 处理一次 UART DMA idle 接收事件。
 *
 * 本函数由 BSP UART idle 回调调用，也可由测试代码直接调用。
 *
 * @param size 本次 DMA idle 收到的字节数。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_rx_event(uint16_t size);

/**
 * @brief 直接喂入 host 协议字节流。
 *
 * @param data 字节流首地址。
 * @param length 字节流长度，单位 byte。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_feed(const uint8_t *data, uint16_t length);

/**
 * @brief 读取最近一次 HostAim 原始视觉误差。
 *
 * @param out 自瞄误差输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_get_aim(CommunicationAimState *out);

/**
 * @brief 读取最近一次收到的 RtProbe。
 *
 * @param out probe 输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_get_probe(CommunicationProbe *out);

/**
 * @brief 读取最近一次收到的 RtProbeAck。
 *
 * @param out probe ack 输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_get_ack(CommunicationProbeAck *out);

/**
 * @brief 发送云台状态快照。
 *
 * @param state 云台状态载荷。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_send_state(const CommunicationGimbalState *state);

/**
 * @brief 发送 RTT probe。
 *
 * @param token 上层生成的 token。
 * @param time_us 发送方时间戳，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_send_probe(uint16_t token, uint32_t time_us);

/**
 * @brief 发送 RTT probe ack。
 *
 * @param token 原始 probe token。
 * @param request_time_us 原始请求时间戳，单位 us。
 * @param reply_time_us 应答发送方时间戳，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_send_ack(uint16_t token,
                                                uint32_t request_time_us,
                                                uint32_t reply_time_us);

/**
 * @brief 读取 host 通讯诊断快照。
 *
 * @param out 诊断输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_host_diag(CommunicationHostDiag *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_COMMUNICATION_HOST_H
