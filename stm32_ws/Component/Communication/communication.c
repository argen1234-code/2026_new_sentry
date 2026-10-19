/**
 ******************************************************************************
 * @file    communication.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   通讯组件公共入口实现。
 *
 * @note    本文件只组合 host 485 通讯、VOFA CDC 调试输出和 Debug LOG 绑定，不做 App 接线。
 * @note    RTT 周期、视觉超时和控制参考使用方式由上层安排。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "communication.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
CommunicationStatus communication_default_cfg(CommunicationCfg *cfg)
{
    CommunicationStatus status;

    if (cfg == NULL) {
        return CommunicationNull;
    }

    status = communication_host_default_cfg(&cfg->host);
    if (status != CommunicationOk) {
        return status;
    }
    status = communication_board_default_cfg(&cfg->board);
    if (status != CommunicationOk) {
        return status;
    }
    return communication_time_default_cfg(&cfg->time);
}

CommunicationStatus communication_init(const CommunicationCfg *cfg)
{
    CommunicationStatus status;

    if (cfg == NULL) {
        return CommunicationNull;
    }

    status = communication_host_init(&cfg->host);
    if (status != CommunicationOk) {
        return status;
    }
    status = communication_board_init(&cfg->board);
    if (status != CommunicationOk) {
        return status;
    }
    status = communication_time_init(&cfg->time);
    if (status != CommunicationOk) {
        return status;
    }

    status = comm_vofa_init_cdc();
    if (status != CommunicationOk) {
        return status;
    }

    return comm_vofa_bind_log();
}

CommunicationStatus communication_rx_event(uint16_t size)
{
    return communication_host_rx_event(size);
}

CommunicationStatus communication_rx_board_event(uint16_t size)
{
    return communication_board_rx_event(size);
}

CommunicationStatus communication_get_aim(CommunicationAimState *out)
{
    return communication_host_get_aim(out);
}

CommunicationStatus communication_get_probe(CommunicationProbe *out)
{
    return communication_host_get_probe(out);
}

CommunicationStatus communication_get_probe_ack(CommunicationProbeAck *out)
{
    return communication_host_get_ack(out);
}

CommunicationStatus communication_send_state(const CommunicationGimbalState *state)
{
    return communication_host_send_state(state);
}

CommunicationStatus communication_send_probe(uint16_t token, uint32_t time_us)
{
    (void)communication_time_note_probe(token, time_us);
    return communication_host_send_probe(token, time_us);
}

CommunicationStatus communication_send_probe_ack(uint16_t token,
                                                 uint32_t request_time_us,
                                                 uint32_t reply_time_us)
{
    return communication_host_send_ack(token, request_time_us, reply_time_us);
}

CommunicationStatus communication_get_board_state(CommunicationBoardState *out)
{
    return communication_board_get_state(out);
}

CommunicationStatus communication_get_chassis_state(CommunicationChassisState *out)
{
    return communication_board_get_chassis(out);
}

CommunicationStatus communication_send_board_state(const CommunicationBoardState *state)
{
    return communication_board_send_state(state);
}

CommunicationStatus communication_send_chassis_state(const CommunicationChassisState *state)
{
    return communication_board_send_chassis(state);
}

CommunicationStatus communication_get_board_diag(CommunicationBoardDiag *out)
{
    return communication_board_diag(out);
}

CommunicationStatus communication_get_time(CommunicationTimeState *out)
{
    return communication_time_get(out);
}

CommunicationStatus communication_remote_now(uint32_t local_time_us, uint32_t *remote_time_us)
{
    return communication_time_remote_now(local_time_us, remote_time_us);
}
