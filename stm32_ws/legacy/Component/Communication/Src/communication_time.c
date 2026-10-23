/**
 ******************************************************************************
 * @file    communication_time.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   通讯时间同步与 RTT 估计实现。
 *
 * @note    RTT 采用本地发送时间到本地收到 ack 时间的差值；偏移采用
 *          remote_reply - (local_send + rtt / 2) 的一阶估计。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "communication_time.h"

#include "main.h"

#include <string.h>

/** @brief 时间同步默认最大 RTT，单位 us，0 表示不限制。 */
#ifndef COMM_TIME_MAX_RTT_US
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
#define COMM_TIME_MAX_RTT_US 0U
#endif

/** @brief 时间同步运行上下文。 */
typedef struct
{
    CommunicationTimeCfg cfg;    /**< 配置副本。 */
    CommunicationTimeState state;/**< 状态缓存。 */
    uint8_t have_probe;          /**< 非 0 表示已有待匹配 probe。 */
} CommunicationTime;

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief 时间同步单例上下文。 */
static CommunicationTime comm_time;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 进入临界区。
 *
 * @return 进入前 PRIMASK。
 */
static uint32_t comm_time_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区。
 *
 * @param primask 进入前 PRIMASK。
 */
static void comm_time_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 有符号 32 位时间差。
 *
 * @param a 被减数。
 * @param b 减数。
 * @return a-b 的有符号结果。
 */
static int32_t comm_time_diff(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
CommunicationStatus communication_time_default_cfg(CommunicationTimeCfg *cfg)
{
    if (cfg == NULL) {
        return CommunicationNull;
    }

    cfg->initiator = 0U;
    cfg->max_rtt_us = COMM_TIME_MAX_RTT_US;
    return CommunicationOk;
}

CommunicationStatus communication_time_init(const CommunicationTimeCfg *cfg)
{
    if (cfg == NULL) {
        return CommunicationNull;
    }

    (void)memset(&comm_time, 0, sizeof(comm_time));
    comm_time.cfg = *cfg;
    comm_time.state.initiator = cfg->initiator;
    return CommunicationOk;
}

CommunicationStatus communication_time_note_probe(uint16_t token, uint32_t local_time_us)
{
    uint32_t primask;

    primask = comm_time_enter();
    comm_time.state.last_token = token;
    comm_time.state.last_send_us = local_time_us;
    comm_time.have_probe = 1U;
    comm_time_exit(primask);
    return CommunicationOk;
}

CommunicationStatus communication_time_on_ack(const CommunicationProbeAck *ack, uint32_t local_time_us)
{
    uint32_t rtt_us;
    uint32_t midpoint_us;
    int32_t offset_us;
    uint32_t primask;

    if (ack == NULL) {
        return CommunicationNull;
    }

    primask = comm_time_enter();
    if ((comm_time.have_probe == 0U) ||
        (ack->token != comm_time.state.last_token) ||
        (ack->request_time_us != comm_time.state.last_send_us)) {
        comm_time_exit(primask);
        return CommunicationState;
    }

    rtt_us = (uint32_t)(local_time_us - comm_time.state.last_send_us);
    if ((comm_time.cfg.max_rtt_us != 0U) && (rtt_us > comm_time.cfg.max_rtt_us)) {
        comm_time_exit(primask);
        return CommunicationArg;
    }

    midpoint_us = comm_time.state.last_send_us + (rtt_us / 2U);
    offset_us = comm_time_diff(ack->reply_time_us, midpoint_us);
    comm_time.state.last_rtt_us = rtt_us;
    comm_time.state.offset_us = offset_us;
    comm_time.state.valid = 1U;
    comm_time.state.version++;
    comm_time.have_probe = 0U;
    comm_time_exit(primask);
    return CommunicationOk;
}

CommunicationStatus communication_time_make_ack(const CommunicationProbe *probe,
                                                uint32_t reply_time_us,
                                                CommunicationProbeAck *ack)
{
    if ((probe == NULL) || (ack == NULL)) {
        return CommunicationNull;
    }

    ack->token = probe->token;
    ack->request_time_us = probe->time_us;
    ack->reply_time_us = reply_time_us;
    ack->seq = probe->seq;
    return CommunicationOk;
}

CommunicationStatus communication_time_get(CommunicationTimeState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }

    primask = comm_time_enter();
    *out = comm_time.state;
    comm_time_exit(primask);
    return CommunicationOk;
}

CommunicationStatus communication_time_remote_now(uint32_t local_time_us, uint32_t *remote_time_us)
{
    uint32_t primask;

    if (remote_time_us == NULL) {
        return CommunicationNull;
    }

    primask = comm_time_enter();
    if (comm_time.state.valid == 0U) {
        comm_time_exit(primask);
        return CommunicationState;
    }
    *remote_time_us = (uint32_t)((int32_t)local_time_us + comm_time.state.offset_us);
    comm_time_exit(primask);
    return CommunicationOk;
}
