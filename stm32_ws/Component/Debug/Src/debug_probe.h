/**
 ******************************************************************************
 * @file    debug_probe.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug 只读探针注册与快照接口。
 *
 * @note    探针槽位为固定数组，默认上限 32，不使用动态内存。
 * @note    本文件不包含 VOFA、USB、串口或 App 调度逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_DEBUG_PROBE_H
#define PRINTK_COMPONENT_DEBUG_PROBE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DEBUG_PROBE_MAX
/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief Debug 在线探针最大数量。 */
#define DEBUG_PROBE_MAX 32U
#endif

/** @brief 无效探针句柄。 */
#define DEBUG_PROBE_HANDLE_INVALID ((DebugProbeHandle)0xFFFFFFFFUL)

/** @brief 探针注册已被丢弃的句柄。 */
#define DEBUG_PROBE_HANDLE_DROPPED ((DebugProbeHandle)0xFFFFFFFEUL)

/**
 * @brief Debug 状态码。
 */
typedef enum
{
    DebugOk = 0,       /**< 操作成功。 */
    DebugNull = 1,     /**< 输入指针为空。 */
    DebugArg = 2,      /**< 输入参数非法。 */
    DebugFull = 3,         /**< 探针槽位已满。 */
    DebugTypeMismatch = 4, /**< 同名或同句柄探针类型不匹配。 */
    DebugNotFound = 5,     /**< 指定探针不存在。 */
    DebugState = 6,        /**< 当前状态不允许执行。 */
    DebugBusy = 7,         /**< 输出端正忙。 */
    DebugSmall = 8,        /**< 内部缓冲区不足。 */
    DebugIo = 9            /**< 底层输出失败。 */
} DebugStatus;

/**
 * @brief Debug 探针数据类型。
 */
typedef enum
{
    DebugTypeNone = 0, /**< 空类型。 */
    DebugTypeF32 = 1,  /**< float 类型。 */
    DebugTypeU32 = 2,  /**< uint32_t 类型。 */
    DebugTypeI32 = 3,  /**< int32_t 类型。 */
    DebugTypeBool = 4  /**< bool 类型。 */
} DebugType;

/**
 * @brief Debug 探针值。
 */
typedef union
{
    float f32;       /**< float 值。 */
    uint32_t u32;    /**< uint32_t 值。 */
    int32_t i32;     /**< int32_t 值。 */
    uint8_t boolean; /**< bool 值，0 表示 false，1 表示 true。 */
} DebugValue;

/** @brief Debug 探针句柄。 */
typedef uint32_t DebugProbeHandle;

/**
 * @brief Debug 探针快照。
 */
typedef struct
{
    const char *name;      /**< 探针名称，生命周期由调用宏处字符串负责。 */
    DebugType type;        /**< 探针类型。 */
    DebugValue value;      /**< 最近一次写入值。 */
    uint64_t timestamp_us; /**< 最近一次更新时间戳，单位 us。 */
    uint32_t update_count; /**< 本探针更新次数。 */
    uint8_t valid;         /**< 非 0 表示快照有效。 */
} DebugProbeSnapshot;

/**
 * @brief Debug 探针统计信息。
 */
typedef struct
{
    uint16_t count;             /**< 当前已注册探针数量。 */
    uint16_t capacity;          /**< 探针总容量。 */
    uint32_t update_count;      /**< 全局探针更新次数。 */
    uint32_t full_count;        /**< 槽位耗尽次数。 */
    uint32_t type_error_count;  /**< 类型不匹配次数。 */
    uint32_t arg_error_count;   /**< 参数错误次数。 */
} DebugStats;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 清空探针注册表。
 *
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_probe_reset(void);

/**
 * @brief 写入带时间戳的 float 探针。
 *
 * @param handle 调用点持有的探针句柄。
 * @param name 探针名称。
 * @param value 探针值。
 * @param timestamp_us 时间戳，单位 us；为 0 时使用平台当前时间。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_probe_f32_ts(DebugProbeHandle *handle,
                               const char *name,
                               float value,
                               uint64_t timestamp_us);

/**
 * @brief 写入带时间戳的 uint32_t 探针。
 *
 * @param handle 调用点持有的探针句柄。
 * @param name 探针名称。
 * @param value 探针值。
 * @param timestamp_us 时间戳，单位 us；为 0 时使用平台当前时间。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_probe_u32_ts(DebugProbeHandle *handle,
                               const char *name,
                               uint32_t value,
                               uint64_t timestamp_us);

/**
 * @brief 写入带时间戳的 int32_t 探针。
 *
 * @param handle 调用点持有的探针句柄。
 * @param name 探针名称。
 * @param value 探针值。
 * @param timestamp_us 时间戳，单位 us；为 0 时使用平台当前时间。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_probe_i32_ts(DebugProbeHandle *handle,
                               const char *name,
                               int32_t value,
                               uint64_t timestamp_us);

/**
 * @brief 写入带时间戳的 bool 探针。
 *
 * @param handle 调用点持有的探针句柄。
 * @param name 探针名称。
 * @param value 探针值。
 * @param timestamp_us 时间戳，单位 us；为 0 时使用平台当前时间。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_probe_bool_ts(DebugProbeHandle *handle,
                                const char *name,
                                bool value,
                                uint64_t timestamp_us);

/**
 * @brief 读取指定索引探针快照。
 *
 * @param index 探针索引。
 * @param out 快照输出。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_probe_snapshot(uint16_t index, DebugProbeSnapshot *out);

/**
 * @brief 批量读取探针快照。
 *
 * @param out 快照输出数组。
 * @param capacity 输出数组容量。
 * @param out_count 实际写入数量输出。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_probe_snapshot_all(DebugProbeSnapshot *out,
                                     uint16_t capacity,
                                     uint16_t *out_count);

/**
 * @brief 读取当前探针数量。
 *
 * @return 返回当前已注册探针数量。
 */
uint16_t debug_probe_count(void);

/**
 * @brief 读取 Debug 统计信息。
 *
 * @param out 统计信息输出。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_stats(DebugStats *out);

/**
 * @brief 获取当前平台时间戳。
 *
 * @return 当前单调时间戳，单位 us。
 */
uint64_t debug_time_us(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_DEBUG_PROBE_H
