/**
 ******************************************************************************
 * @file    debug_profiler.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   Debug 轻量线程采样 profiler 实现。
 *
 * @note    采样路径只做槽位查找和计数递增，不格式化字符串、不输出、不阻塞。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "debug_profiler.h"

#include "debug_platform.h"
#include "cmsis_os.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 中断上下文采样使用的固定 ID。 */
#define DEBUG_PROFILER_ISR_ID ((const void *)0xFFFFFFFFUL)

/** @brief 中断上下文槽位名称。 */
static const char debug_profiler_isr_name[] = "isr";

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief profiler 槽位数组。 */
static DebugProfilerSlot profiler_slots[DEBUG_PROFILER_MAX];

/** @brief profiler 统计信息。 */
static DebugProfilerStats profiler_stats;

/**
 * @brief 查找或创建线程槽位。
 *
 * @param task_id 线程 ID。
 * @param name 线程名。
 * @return 槽位指针，失败返回 NULL。
 */
static DebugProfilerSlot *profiler_resolve(const void *task_id, const char *name)
{
    uint16_t i;

    for (i = 0U; i < (uint16_t)DEBUG_PROFILER_MAX; i++) {
        if ((profiler_slots[i].valid != 0U) && (profiler_slots[i].task_id == task_id)) {
            return &profiler_slots[i];
        }
    }

    if (profiler_stats.count >= (uint16_t)DEBUG_PROFILER_MAX) {
        profiler_stats.full_count++;
        return NULL;
    }

    i = profiler_stats.count;
    profiler_slots[i].task_id = task_id;
    profiler_slots[i].name = name;
    profiler_slots[i].count = 0U;
    profiler_slots[i].valid = 1U;
    profiler_stats.count++;
    return &profiler_slots[i];
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
DebugStatus debug_profiler_reset(void)
{
    DebugIrqState irq;

    irq = debug_lock();
    (void)memset(profiler_slots, 0, sizeof(profiler_slots));
    (void)memset(&profiler_stats, 0, sizeof(profiler_stats));
    profiler_stats.capacity = (uint16_t)DEBUG_PROFILER_MAX;
    debug_unlock(irq);
    return DebugOk;
}

DebugStatus debug_profiler_sample(const void *task_id, const char *name)
{
    DebugProfilerSlot *slot;
    DebugIrqState irq;

    if (task_id == NULL) {
        irq = debug_lock();
        profiler_stats.arg_count++;
        debug_unlock(irq);
        return DebugArg;
    }

    irq = debug_lock();
    slot = profiler_resolve(task_id, name);
    if (slot == NULL) {
        debug_unlock(irq);
        return DebugFull;
    }
    slot->count++;
    profiler_stats.sample_count++;
    debug_unlock(irq);
    return DebugOk;
}

DebugStatus debug_profiler_sample_current(void)
{
    osThreadId_t task_id;
    const char *name;

    if (debug_in_isr()) {
        return debug_profiler_sample(DEBUG_PROFILER_ISR_ID, debug_profiler_isr_name);
    }

    task_id = osThreadGetId();
    if (task_id == NULL) {
        return debug_profiler_sample(DEBUG_PROFILER_ISR_ID, debug_profiler_isr_name);
    }
    name = osThreadGetName(task_id);
    return debug_profiler_sample((const void *)task_id, name);
}

DebugStatus debug_profiler_snapshot(uint16_t index, DebugProfilerSlot *out)
{
    DebugIrqState irq;

    if (out == NULL) {
        return DebugNull;
    }
    if (index >= (uint16_t)DEBUG_PROFILER_MAX) {
        return DebugArg;
    }

    irq = debug_lock();
    *out = profiler_slots[index];
    debug_unlock(irq);
    return (out->valid != 0U) ? DebugOk : DebugNotFound;
}

DebugStatus debug_profiler_snapshot_all(DebugProfilerSlot *out,
                                        uint16_t capacity,
                                        uint16_t *out_count)
{
    DebugIrqState irq;
    uint16_t i;
    uint16_t count;

    if ((out == NULL) || (out_count == NULL)) {
        return DebugNull;
    }

    irq = debug_lock();
    count = profiler_stats.count;
    if (count > capacity) {
        count = capacity;
    }
    for (i = 0U; i < count; i++) {
        out[i] = profiler_slots[i];
    }
    *out_count = count;
    debug_unlock(irq);
    return DebugOk;
}

DebugStatus debug_profiler_stats(DebugProfilerStats *out)
{
    DebugIrqState irq;

    if (out == NULL) {
        return DebugNull;
    }

    irq = debug_lock();
    *out = profiler_stats;
    debug_unlock(irq);
    return DebugOk;
}
