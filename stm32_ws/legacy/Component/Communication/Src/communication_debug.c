/**
 ******************************************************************************
 * @file    communication_debug.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug 探针到 VOFA JustFloat 的桥接实现。
 *
 * @note    JustFloat 只接受 float 数组，因此整型和 bool 探针会在发送前转换
 *          为 float。该转换只服务波形观察，不改变 Debug 内核原始类型。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "communication_debug.h"

#include "communication_vofa.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief Debug VOFA 桥诊断缓存。 */
static CommunicationDebugDiag comm_debug_diag;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 将 Debug 探针快照转换为 VOFA float。
 *
 * @param snapshot 探针快照。
 * @return 可用于 JustFloat 输出的 float 值。
 */
static float debug_value_to_float(const DebugProbeSnapshot *snapshot)
{
    if ((snapshot == NULL) || (snapshot->valid == 0U)) {
        return 0.0F;
    }

    switch (snapshot->type) {
    case DebugTypeF32:
        return snapshot->value.f32;
    case DebugTypeU32:
        return (float)snapshot->value.u32;
    case DebugTypeI32:
        return (float)snapshot->value.i32;
    case DebugTypeBool:
        return (snapshot->value.boolean != 0U) ? 1.0F : 0.0F;
    default:
        return 0.0F;
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
CommunicationStatus communication_debug_flush(void)
{
    DebugProbeSnapshot snapshot;
    float values[DEBUG_PROBE_MAX];
    CommunicationStatus status;
    uint16_t i;
    uint16_t count;

    (void)memset(values, 0, sizeof(values));
    count = debug_probe_count();

    for (i = 0U; i < (uint16_t)DEBUG_PROBE_MAX; i++) {
        if (debug_probe_snapshot(i, &snapshot) == DebugOk) {
            values[i] = debug_value_to_float(&snapshot);
        }
    }

    status = comm_vofa_float(values, (uint16_t)DEBUG_PROBE_MAX);
    comm_debug_diag.probe_count = count;
    comm_debug_diag.float_count = (uint16_t)DEBUG_PROBE_MAX;
    if (status == CommunicationOk) {
        comm_debug_diag.flush_count++;
    } else if (status == CommunicationBusy) {
        comm_debug_diag.busy_count++;
    } else {
        comm_debug_diag.error_count++;
    }

    return status;
}

CommunicationStatus communication_debug_profiler_flush(void)
{
    DebugProfilerSlot snapshot;
    float values[DEBUG_PROFILER_MAX];
    CommunicationStatus status;
    uint16_t i;

    (void)memset(values, 0, sizeof(values));
    for (i = 0U; i < (uint16_t)DEBUG_PROFILER_MAX; i++) {
        if (debug_profiler_snapshot(i, &snapshot) == DebugOk) {
            values[i] = (float)snapshot.count;
        }
    }

    status = comm_vofa_float(values, (uint16_t)DEBUG_PROFILER_MAX);
    if (status == CommunicationOk) {
        comm_debug_diag.profiler_flush_count++;
    } else if (status == CommunicationBusy) {
        comm_debug_diag.profiler_busy_count++;
    } else {
        comm_debug_diag.profiler_error_count++;
    }

    return status;
}

CommunicationStatus communication_debug_diag(CommunicationDebugDiag *out)
{
    if (out == NULL) {
        return CommunicationNull;
    }

    *out = comm_debug_diag;
    return CommunicationOk;
}
