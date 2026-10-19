/**
 ******************************************************************************
 * @file    communication_debug.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug 探针到 VOFA JustFloat 的桥接接口。
 *
 * @note    本文件只把 Component/Debug 的只读快照包装为 VOFA 输出，不拥有
 *          调度周期，也不读取 host 控制命令。
 * @note    输出固定为 DEBUG_PROBE_MAX 个 float，未注册槽位填 0，避免 VOFA
 *          通道数量随运行状态变化。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_DEBUG_H
#define PRINTK_COMPONENT_COMMUNICATION_DEBUG_H

#include "communication_common.h"
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief Debug VOFA 桥诊断信息。
 */
typedef struct
{
    uint16_t probe_count;  /**< 最近一次 flush 时的已注册探针数量。 */
    uint16_t float_count;  /**< 每帧输出 float 数量。 */
    uint32_t flush_count;  /**< 成功输出次数。 */
    uint32_t busy_count;   /**< VOFA 发送 busy 次数。 */
    uint32_t error_count;  /**< VOFA 发送失败次数。 */
    uint32_t profiler_flush_count; /**< profiler 成功输出次数。 */
    uint32_t profiler_busy_count;  /**< profiler 输出 busy 次数。 */
    uint32_t profiler_error_count; /**< profiler 输出失败次数。 */
} CommunicationDebugDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 将当前 Debug 探针快照输出为一帧 VOFA JustFloat。
 *
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_debug_flush(void);

/**
 * @brief 将当前 profiler 采样计数输出为一帧 VOFA JustFloat。
 *
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_debug_profiler_flush(void);

/**
 * @brief 读取 Debug VOFA 桥诊断信息。
 *
 * @param out 诊断信息输出。
 * @return 返回通信组件状态码。
 */
CommunicationStatus communication_debug_diag(CommunicationDebugDiag *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_COMMUNICATION_DEBUG_H
