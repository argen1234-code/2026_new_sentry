/**
 ******************************************************************************
 * @file    debug_probe.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug 只读探针注册与更新实现。
 *
 * @note    探针句柄包含注册表 generation，debug_probe_reset() 后旧句柄会自动
 *          失效并在下一次宏调用时重新注册。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "debug_probe.h"

#include "debug_platform.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 探针句柄 generation 位移。 */
#define DEBUG_HANDLE_GEN_SHIFT 16U

/** @brief 探针句柄索引掩码。 */
#define DEBUG_HANDLE_INDEX_MASK 0xFFFFUL

/**
 * @brief 探针表项。
 */
typedef struct
{
    DebugProbeSnapshot snapshot; /**< 对外快照数据。 */
} DebugEntry;

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief 探针注册表。 */
static DebugEntry debug_entries[DEBUG_PROBE_MAX];

/** @brief 探针统计信息。 */
static DebugStats debug_stat;

/** @brief 当前探针句柄 generation，0 保留为初始无效代。 */
static uint16_t debug_generation = 1U;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 生成探针句柄。
 *
 * @param index 探针索引。
 * @return 返回编码后的探针句柄。
 */
static DebugProbeHandle debug_make_handle(uint16_t index)
{
    return (((DebugProbeHandle)debug_generation << DEBUG_HANDLE_GEN_SHIFT) |
            (DebugProbeHandle)index);
}

/**
 * @brief 从句柄解析当前 generation 下的探针索引。
 *
 * @param handle 探针句柄。
 * @param index 索引输出。
 * @return 有效返回 true，否则返回 false。
 */
static bool debug_handle_index(DebugProbeHandle handle, uint16_t *index)
{
    uint16_t gen;
    uint16_t idx;

    if ((handle == DEBUG_PROBE_HANDLE_INVALID) ||
        (handle == DEBUG_PROBE_HANDLE_DROPPED) ||
        (index == NULL)) {
        return false;
    }

    gen = (uint16_t)(handle >> DEBUG_HANDLE_GEN_SHIFT);
    idx = (uint16_t)(handle & DEBUG_HANDLE_INDEX_MASK);
    if ((gen != debug_generation) || (idx >= debug_stat.count)) {
        return false;
    }

    *index = idx;
    return true;
}

/**
 * @brief 按名称查找已注册探针。
 *
 * @param name 探针名称。
 * @param index 索引输出。
 * @return 找到返回 true，否则返回 false。
 */
static bool debug_find_name(const char *name, uint16_t *index)
{
    uint16_t i;

    if ((name == NULL) || (index == NULL)) {
        return false;
    }

    for (i = 0U; i < debug_stat.count; i++) {
        if ((debug_entries[i].snapshot.name != NULL) &&
            (strcmp(debug_entries[i].snapshot.name, name) == 0)) {
            *index = i;
            return true;
        }
    }

    return false;
}

/**
 * @brief 注册或解析探针句柄。
 *
 * @param handle 调用点持有的探针句柄。
 * @param name 探针名称。
 * @param type 探针类型。
 * @param index 索引输出。
 * @return 返回 Debug 状态码。
 */
static DebugStatus debug_resolve(DebugProbeHandle *handle,
                                 const char *name,
                                 DebugType type,
                                 uint16_t *index)
{
    DebugIrqState state;
    DebugProbeSnapshot *snapshot;
    uint16_t idx;

    if ((handle == NULL) || (index == NULL)) {
        debug_stat.arg_error_count++;
        return DebugNull;
    }
    if ((name == NULL) || (name[0] == '\0') || (type == DebugTypeNone)) {
        debug_stat.arg_error_count++;
        return DebugArg;
    }
    if (*handle == DEBUG_PROBE_HANDLE_DROPPED) {
        return DebugFull;
    }

    state = debug_lock();

    if (debug_handle_index(*handle, &idx)) {
        snapshot = &debug_entries[idx].snapshot;
        if (snapshot->type != type) {
            debug_stat.type_error_count++;
            debug_unlock(state);
            return DebugTypeMismatch;
        }
        *index = idx;
        debug_unlock(state);
        return DebugOk;
    }

    if (debug_find_name(name, &idx)) {
        snapshot = &debug_entries[idx].snapshot;
        if (snapshot->type != type) {
            debug_stat.type_error_count++;
            debug_unlock(state);
            return DebugTypeMismatch;
        }
        *handle = debug_make_handle(idx);
        *index = idx;
        debug_unlock(state);
        return DebugOk;
    }

    if (debug_stat.count >= (uint16_t)DEBUG_PROBE_MAX) {
        debug_stat.full_count++;
        *handle = DEBUG_PROBE_HANDLE_DROPPED;
        debug_unlock(state);
        return DebugFull;
    }

    idx = debug_stat.count;
    debug_stat.count++;
    snapshot = &debug_entries[idx].snapshot;
    (void)memset(snapshot, 0, sizeof(*snapshot));
    snapshot->name = name;
    snapshot->type = type;
    snapshot->valid = 1U;
    *handle = debug_make_handle(idx);
    *index = idx;

    debug_unlock(state);
    return DebugOk;
}

/**
 * @brief 写入探针公共逻辑。
 *
 * @param handle 调用点持有的探针句柄。
 * @param name 探针名称。
 * @param type 探针类型。
 * @param value 探针值。
 * @param timestamp_us 时间戳，0 表示使用平台时间。
 * @return 返回 Debug 状态码。
 */
static DebugStatus debug_write(DebugProbeHandle *handle,
                               const char *name,
                               DebugType type,
                               DebugValue value,
                               uint64_t timestamp_us)
{
    DebugIrqState state;
    DebugProbeSnapshot *snapshot;
    DebugStatus status;
    uint16_t index;

    status = debug_resolve(handle, name, type, &index);
    if (status != DebugOk) {
        return status;
    }

    if (timestamp_us == 0ULL) {
        timestamp_us = debug_time_us();
    }

    state = debug_lock();
    snapshot = &debug_entries[index].snapshot;
    snapshot->value = value;
    snapshot->timestamp_us = timestamp_us;
    snapshot->update_count++;
    debug_stat.update_count++;
    debug_unlock(state);

    return DebugOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
DebugStatus debug_probe_reset(void)
{
    DebugIrqState state = debug_lock();

    (void)memset(debug_entries, 0, sizeof(debug_entries));
    (void)memset(&debug_stat, 0, sizeof(debug_stat));
    debug_stat.capacity = (uint16_t)DEBUG_PROBE_MAX;
    debug_generation++;
    if (debug_generation == 0U) {
        debug_generation = 1U;
    }

    debug_unlock(state);
    return DebugOk;
}

DebugStatus debug_probe_f32_ts(DebugProbeHandle *handle,
                               const char *name,
                               float value,
                               uint64_t timestamp_us)
{
    DebugValue v;

    v.f32 = value;
    return debug_write(handle, name, DebugTypeF32, v, timestamp_us);
}

DebugStatus debug_probe_u32_ts(DebugProbeHandle *handle,
                               const char *name,
                               uint32_t value,
                               uint64_t timestamp_us)
{
    DebugValue v;

    v.u32 = value;
    return debug_write(handle, name, DebugTypeU32, v, timestamp_us);
}

DebugStatus debug_probe_i32_ts(DebugProbeHandle *handle,
                               const char *name,
                               int32_t value,
                               uint64_t timestamp_us)
{
    DebugValue v;

    v.i32 = value;
    return debug_write(handle, name, DebugTypeI32, v, timestamp_us);
}

DebugStatus debug_probe_bool_ts(DebugProbeHandle *handle,
                                const char *name,
                                bool value,
                                uint64_t timestamp_us)
{
    DebugValue v;

    v.boolean = value ? 1U : 0U;
    return debug_write(handle, name, DebugTypeBool, v, timestamp_us);
}

DebugStatus debug_probe_snapshot(uint16_t index, DebugProbeSnapshot *out)
{
    DebugIrqState state;

    if (out == NULL) {
        debug_stat.arg_error_count++;
        return DebugNull;
    }

    state = debug_lock();
    if (index >= debug_stat.count) {
        debug_unlock(state);
        (void)memset(out, 0, sizeof(*out));
        return DebugNotFound;
    }

    *out = debug_entries[index].snapshot;
    debug_unlock(state);
    return DebugOk;
}

DebugStatus debug_probe_snapshot_all(DebugProbeSnapshot *out,
                                     uint16_t capacity,
                                     uint16_t *out_count)
{
    DebugIrqState state;
    uint16_t count;
    uint16_t i;

    if ((out == NULL) || (out_count == NULL)) {
        debug_stat.arg_error_count++;
        return DebugNull;
    }

    state = debug_lock();
    count = debug_stat.count;
    if (count > capacity) {
        count = capacity;
    }
    for (i = 0U; i < count; i++) {
        out[i] = debug_entries[i].snapshot;
    }
    *out_count = count;
    debug_unlock(state);

    return DebugOk;
}

uint16_t debug_probe_count(void)
{
    uint16_t count;
    DebugIrqState state = debug_lock();

    count = debug_stat.count;
    debug_unlock(state);
    return count;
}

DebugStatus debug_stats(DebugStats *out)
{
    DebugIrqState state;

    if (out == NULL) {
        debug_stat.arg_error_count++;
        return DebugNull;
    }

    state = debug_lock();
    *out = debug_stat;
    debug_unlock(state);
    return DebugOk;
}
