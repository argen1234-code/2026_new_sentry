/**
 ******************************************************************************
 * @file    debug_profiler.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   Debug 轻量线程采样 profiler。
 *
 * @note    本模块只维护固定槽位采样计数，不创建任务、不输出文本、不使用
 *          动态内存。
 * @note    采样定时器和采样周期由上层配置；本模块只提供 O(1) 采样接口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_DEBUG_PROFILER_H
#define PRINTK_COMPONENT_DEBUG_PROFILER_H

#include "debug_probe.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DEBUG_PROFILER_MAX
/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief profiler 最大线程槽位数量。 */
#define DEBUG_PROFILER_MAX 32U
#endif

/**
 * @brief profiler 单槽位快照。
 */
typedef struct
{
    const void *task_id; /**< 线程 ID；中断上下文使用内部固定 ID。 */
    const char *name;    /**< 线程名；未命名时可为空。 */
    uint32_t count;      /**< 被采样次数。 */
    uint8_t valid;       /**< 非 0 表示该槽位有效。 */
} DebugProfilerSlot;

/**
 * @brief profiler 统计信息。
 */
typedef struct
{
    uint16_t count;         /**< 已注册槽位数量。 */
    uint16_t capacity;      /**< 最大槽位数量。 */
    uint32_t sample_count;  /**< 总采样次数。 */
    uint32_t full_count;    /**< 槽位不足次数。 */
    uint32_t arg_count;     /**< 参数错误次数。 */
} DebugProfilerStats;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 清空 profiler 状态。
 *
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_profiler_reset(void);

/**
 * @brief 对指定线程采样一次。
 *
 * @param task_id 线程 ID。
 * @param name 线程名，可为空。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_profiler_sample(const void *task_id, const char *name);

/**
 * @brief 对当前上下文采样一次。
 *
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_profiler_sample_current(void);

/**
 * @brief 读取指定索引槽位快照。
 *
 * @param index 槽位索引。
 * @param out 快照输出。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_profiler_snapshot(uint16_t index, DebugProfilerSlot *out);

/**
 * @brief 批量读取 profiler 快照。
 *
 * @param out 快照输出数组。
 * @param capacity 输出数组容量。
 * @param out_count 实际写入数量输出。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_profiler_snapshot_all(DebugProfilerSlot *out,
                                        uint16_t capacity,
                                        uint16_t *out_count);

/**
 * @brief 读取 profiler 统计信息。
 *
 * @param out 统计输出。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_profiler_stats(DebugProfilerStats *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_DEBUG_PROFILER_H
