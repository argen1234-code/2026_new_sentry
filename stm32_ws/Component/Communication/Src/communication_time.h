/**
 ******************************************************************************
 * @file    communication_time.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   通讯时间同步与 RTT 估计接口。
 *
 * @note    本模块只复用 RtProbe/RtProbeAck 的 token 与时间戳计算 RTT 和
 *          粗略时钟偏移，不拥有传输通道和调度周期。
 * @note    上层负责决定是否主动发起同步；响应端只需要使用 probe 生成 ack。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_TIME_H
#define PRINTK_COMPONENT_COMMUNICATION_TIME_H

#include "communication_common.h"
#include "communication_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief 时间同步配置。
 */
typedef struct
{
    uint8_t initiator;    /**< 非 0 表示本端会主动发起 probe。 */
    uint32_t max_rtt_us;  /**< 可接受的最大 RTT，0 表示不限制。 */
} CommunicationTimeCfg;

/**
 * @brief 时间同步状态快照。
 */
typedef struct
{
    uint8_t valid;         /**< 非 0 表示 offset_us 和 last_rtt_us 有效。 */
    uint8_t initiator;     /**< 当前配置下是否主动发起同步。 */
    uint16_t last_token;   /**< 最近一次有效 token。 */
    uint32_t last_send_us; /**< 最近一次本地 probe 发送时间。 */
    uint32_t last_rtt_us;  /**< 最近一次 RTT，单位 us。 */
    int32_t offset_us;     /**< 远端时间相对本地时间的估计偏移，单位 us。 */
    uint32_t version;      /**< 成功更新次数。 */
} CommunicationTimeState;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 填充时间同步默认配置。
 *
 * @param cfg 配置输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_time_default_cfg(CommunicationTimeCfg *cfg);

/**
 * @brief 初始化时间同步状态。
 *
 * @param cfg 时间同步配置。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_time_init(const CommunicationTimeCfg *cfg);

/**
 * @brief 记录一次本地主动 probe 发送。
 *
 * @param token probe token。
 * @param local_time_us 本地发送时间，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_time_note_probe(uint16_t token, uint32_t local_time_us);

/**
 * @brief 用收到的 probe ack 更新 RTT 与时钟偏移。
 *
 * @param ack 收到的 probe ack。
 * @param local_time_us 本地收到 ack 的时间，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_time_on_ack(const CommunicationProbeAck *ack, uint32_t local_time_us);

/**
 * @brief 根据 probe 生成 ack 载荷字段。
 *
 * @param probe 收到的 probe。
 * @param reply_time_us 本地回复时间，单位 us。
 * @param ack ack 输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_time_make_ack(const CommunicationProbe *probe,
                                                uint32_t reply_time_us,
                                                CommunicationProbeAck *ack);

/**
 * @brief 读取时间同步状态。
 *
 * @param out 状态输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_time_get(CommunicationTimeState *out);

/**
 * @brief 将本地时间换算为远端时间估计。
 *
 * @param local_time_us 本地时间，单位 us。
 * @param remote_time_us 远端时间估计输出，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus communication_time_remote_now(uint32_t local_time_us, uint32_t *remote_time_us);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_COMMUNICATION_TIME_H
