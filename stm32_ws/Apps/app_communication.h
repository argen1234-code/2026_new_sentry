/**
 ******************************************************************************
 * @file    app_communication.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Communication 应用层接口。
 *
 * @note    本模块负责初始化 Debug 和 Component/Communication，并周期执行
 *          云台状态上报、probe ack 和 Debug VOFA flush。
 * @note    本模块不实现自瞄滤波、RTT 计算或具体控制逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_COMMUNICATION_H
#define PRINTK_APPS_COMMUNICATION_H

#include <stdint.h>

#include "communication.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 云台状态 bit：输入命令在线。 */
#define APP_COMM_GIMBAL_STATUS_INPUT_ONLINE ((uint8_t)0x01U)

/** @brief 云台状态 bit：INS 观测 ready。 */
#define APP_COMM_GIMBAL_STATUS_OBSERVE_READY ((uint8_t)0x02U)

/** @brief 云台状态 bit：yaw 电机反馈在线。 */
#define APP_COMM_GIMBAL_STATUS_YAW_ONLINE ((uint8_t)0x04U)

/** @brief 云台状态 bit：pitch 电机反馈在线。 */
#define APP_COMM_GIMBAL_STATUS_PITCH_ONLINE ((uint8_t)0x08U)

/** @brief 云台状态 bit：控制器允许发送输出。 */
#define APP_COMM_GIMBAL_STATUS_SEND_ENABLE ((uint8_t)0x10U)

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief Communication 应用层诊断信息。
 */
typedef struct
{
    uint32_t loop_count;          /**< 周期执行次数。 */
    uint32_t state_send_count;    /**< 云台状态发送成功次数。 */
    uint32_t state_fail_count;    /**< 云台状态发送失败或快照无效次数。 */
    uint32_t probe_send_count;    /**< 本端主动 probe 发送成功次数。 */
    uint32_t probe_fail_count;    /**< 本端主动 probe 发送失败次数。 */
    uint32_t probe_ack_count;     /**< probe ack 发送成功次数。 */
    uint32_t probe_ack_fail_count;/**< probe ack 发送失败次数。 */
    uint32_t debug_flush_count;   /**< Debug VOFA flush 成功次数。 */
    uint32_t debug_busy_count;    /**< Debug VOFA flush busy 次数。 */
    uint32_t debug_error_count;   /**< Debug VOFA flush 失败次数。 */
    uint32_t profiler_flush_count;/**< profiler VOFA flush 成功次数。 */
    uint32_t profiler_busy_count; /**< profiler VOFA flush busy 次数。 */
    uint32_t profiler_error_count;/**< profiler VOFA flush 失败次数。 */
    uint32_t aim_count;           /**< 读取到启用自瞄输入的次数。 */
    uint32_t last_time_us;        /**< 最近一次周期时间戳低 32 位，单位 us。 */
    uint32_t last_probe_seq;      /**< 最近一次处理的 host probe seq。 */
    uint32_t last_ack_seq;        /**< 最近一次读取到的 host ack seq。 */
    CommunicationStatus init_status;   /**< 初始化阶段最近状态。 */
    CommunicationStatus last_status;   /**< 最近一次组件状态。 */
    CommunicationStatus state_status;  /**< 最近一次云台状态发送状态。 */
    CommunicationStatus probe_status;  /**< 最近一次主动 probe 发送状态。 */
    CommunicationStatus ack_status;    /**< 最近一次 probe ack 发送状态。 */
    CommunicationStatus debug_status;  /**< 最近一次 Debug VOFA flush 状态。 */
    CommunicationStatus aim_status;    /**< 最近一次自瞄输入读取状态。 */
    uint8_t ready;               /**< 非 0 表示 app 已初始化。 */
    uint8_t aim_online;          /**< 非 0 表示最近一次自瞄输入启用。 */
    uint8_t gimbal_ready;        /**< 非 0 表示最近一次云台快照可上报。 */
} AppCommunicationDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Communication 任务。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_communication_init(void);

/**
 * @brief 读取最近一次 host 原始自瞄误差。
 *
 * @param out 自瞄误差输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_get_aim(CommunicationAimState *out);

/**
 * @brief 获取 Communication 应用层诊断快照。
 *
 * @param out 诊断输出。
 * @return 成功返回 1，参数无效返回 0。
 */
uint8_t app_communication_get_diag(AppCommunicationDiag *out);

/**
 * @brief 获取 host 485 通讯诊断快照。
 *
 * @param out host 诊断输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_get_host_diag(CommunicationHostDiag *out);

/**
 * @brief 获取 Debug VOFA 桥诊断快照。
 *
 * @param out Debug VOFA 诊断输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_get_debug_diag(CommunicationDebugDiag *out);

/**
 * @brief 读取最近一次 host probe。
 *
 * @param out probe 输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_get_probe(CommunicationProbe *out);

/**
 * @brief 读取最近一次 host probe ack。
 *
 * @param out probe ack 输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_get_probe_ack(CommunicationProbeAck *out);

/**
 * @brief 读取通讯时间同步状态。
 *
 * @param out 时间同步状态输出。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_get_time(CommunicationTimeState *out);

/**
 * @brief 主动发送 RTT probe。
 *
 * @param token 上层生成的 token。
 * @param time_us 发送方时间戳，单位 us。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_send_probe(uint16_t token, uint32_t time_us);

/**
 * @brief 立即 flush 一帧 Debug 探针到 VOFA JustFloat。
 *
 * @return 返回通讯组件状态码。
 */
CommunicationStatus app_communication_flush_debug(void);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_COMMUNICATION_H */
