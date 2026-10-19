/**
 ******************************************************************************
 * @file    debug.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug 只读探针与 LOG 公共入口。
 *
 * @note    本组件只负责探针注册、采样和快照读取，不提供热修改、订阅或
 *          host 命令解析。
 * @note    业务代码直接使用 DBG_F32、DBG_U32、DBG_I32、DBG_BOOL 等宏打点，
 *          使用 LOG、LOGW、LOGE_TAG 等宏输出 Printk 风格文本。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_DEBUG_H
#define PRINTK_COMPONENT_DEBUG_H

#include "Src/debug_log.h"
#include "Src/debug_profiler.h"
#include "Src/debug_probe.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 Debug 探针与 LOG 内核。
 *
 * 本函数会清空已注册探针、统计计数和 LOG 输出绑定。应在业务任务开始打点前调用一次。
 *
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_init(void);

#ifdef __cplusplus
}
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 拼接宏内部辅助。 */
#define DEBUG_JOIN_INNER(a, b) a##b

/** @brief 拼接宏内部辅助。 */
#define DEBUG_JOIN(a, b) DEBUG_JOIN_INNER(a, b)

#ifndef DEBUG_ENABLE
/** @brief Debug 宏总开关，默认开启。 */
#define DEBUG_ENABLE 1U
#endif

#ifndef DEBUG_LOG_ENABLE
/** @brief Debug LOG 宏开关，默认跟随 DEBUG_ENABLE。 */
#define DEBUG_LOG_ENABLE DEBUG_ENABLE
#endif

#if (DEBUG_ENABLE != 0U)

/**
 * @brief 带时间戳探针宏内部展开。
 *
 * 每个宏展开点拥有一个静态句柄，首次调用注册，后续调用只更新数值。
 */
#define DEBUG_PROBE_TS(fn, name, value, timestamp_us)                                     \
    do {                                                                                  \
        static DebugProbeHandle DEBUG_JOIN(debug_probe_handle_, __LINE__) =               \
            DEBUG_PROBE_HANDLE_INVALID;                                                   \
        (void)(fn)(&DEBUG_JOIN(debug_probe_handle_, __LINE__), (name), (value),            \
                   (uint64_t)(timestamp_us));                                             \
    } while (0)

/** @brief 写入 float 探针，时间戳由 Debug 平台层提供。 */
#define DBG_F32(name, value) DEBUG_PROBE_TS(debug_probe_f32_ts, (name), (float)(value), 0ULL)

/** @brief 写入 uint32_t 探针，时间戳由 Debug 平台层提供。 */
#define DBG_U32(name, value) DEBUG_PROBE_TS(debug_probe_u32_ts, (name), (uint32_t)(value), 0ULL)

/** @brief 写入 int32_t 探针，时间戳由 Debug 平台层提供。 */
#define DBG_I32(name, value) DEBUG_PROBE_TS(debug_probe_i32_ts, (name), (int32_t)(value), 0ULL)

/** @brief 写入 bool 探针，时间戳由 Debug 平台层提供。 */
#define DBG_BOOL(name, value) DEBUG_PROBE_TS(debug_probe_bool_ts, (name), ((value) ? true : false), 0ULL)

/** @brief 写入带外部时间戳的 float 探针。 */
#define DBG_F32_TS(name, value, timestamp_us) \
    DEBUG_PROBE_TS(debug_probe_f32_ts, (name), (float)(value), (timestamp_us))

/** @brief 写入带外部时间戳的 uint32_t 探针。 */
#define DBG_U32_TS(name, value, timestamp_us) \
    DEBUG_PROBE_TS(debug_probe_u32_ts, (name), (uint32_t)(value), (timestamp_us))

/** @brief 写入带外部时间戳的 int32_t 探针。 */
#define DBG_I32_TS(name, value, timestamp_us) \
    DEBUG_PROBE_TS(debug_probe_i32_ts, (name), (int32_t)(value), (timestamp_us))

/** @brief 写入带外部时间戳的 bool 探针。 */
#define DBG_BOOL_TS(name, value, timestamp_us) \
    DEBUG_PROBE_TS(debug_probe_bool_ts, (name), ((value) ? true : false), (timestamp_us))

#else

/** @brief Debug 关闭时不求值的 float 探针空实现。 */
#define DBG_F32(name, value) do { } while (0)

/** @brief Debug 关闭时不求值的 uint32_t 探针空实现。 */
#define DBG_U32(name, value) do { } while (0)

/** @brief Debug 关闭时不求值的 int32_t 探针空实现。 */
#define DBG_I32(name, value) do { } while (0)

/** @brief Debug 关闭时不求值的 bool 探针空实现。 */
#define DBG_BOOL(name, value) do { } while (0)

/** @brief Debug 关闭时不求值的带时间戳 float 探针空实现。 */
#define DBG_F32_TS(name, value, timestamp_us) do { } while (0)

/** @brief Debug 关闭时不求值的带时间戳 uint32_t 探针空实现。 */
#define DBG_U32_TS(name, value, timestamp_us) do { } while (0)

/** @brief Debug 关闭时不求值的带时间戳 int32_t 探针空实现。 */
#define DBG_I32_TS(name, value, timestamp_us) do { } while (0)

/** @brief Debug 关闭时不求值的带时间戳 bool 探针空实现。 */
#define DBG_BOOL_TS(name, value, timestamp_us) do { } while (0)

#endif

#if (DEBUG_LOG_ENABLE != 0U)

/** @brief 输出 info 等级 Printk 风格日志。 */
#define LOG(format, ...) debug_log_printf(DebugLogInfo, NULL, (format), ##__VA_ARGS__)

/** @brief 输出 debug 等级 Printk 风格日志。 */
#define LOGD(format, ...) debug_log_printf(DebugLogDebug, NULL, (format), ##__VA_ARGS__)

/** @brief 输出 info 等级 Printk 风格日志。 */
#define LOGI(format, ...) debug_log_printf(DebugLogInfo, NULL, (format), ##__VA_ARGS__)

/** @brief 输出 warn 等级 Printk 风格日志。 */
#define LOGW(format, ...) debug_log_printf(DebugLogWarn, NULL, (format), ##__VA_ARGS__)

/** @brief 输出 error 等级 Printk 风格日志。 */
#define LOGE(format, ...) debug_log_printf(DebugLogError, NULL, (format), ##__VA_ARGS__)

/** @brief 输出带 tag 的 info 等级 Printk 风格日志。 */
#define LOG_TAG(tag, format, ...) debug_log_printf(DebugLogInfo, (tag), (format), ##__VA_ARGS__)

/** @brief 输出带 tag 的 debug 等级 Printk 风格日志。 */
#define LOGD_TAG(tag, format, ...) debug_log_printf(DebugLogDebug, (tag), (format), ##__VA_ARGS__)

/** @brief 输出带 tag 的 info 等级 Printk 风格日志。 */
#define LOGI_TAG(tag, format, ...) debug_log_printf(DebugLogInfo, (tag), (format), ##__VA_ARGS__)

/** @brief 输出带 tag 的 warn 等级 Printk 风格日志。 */
#define LOGW_TAG(tag, format, ...) debug_log_printf(DebugLogWarn, (tag), (format), ##__VA_ARGS__)

/** @brief 输出带 tag 的 error 等级 Printk 风格日志。 */
#define LOGE_TAG(tag, format, ...) debug_log_printf(DebugLogError, (tag), (format), ##__VA_ARGS__)

/** @brief 输出无等级前缀的原始 Printk 风格日志。 */
#define LOG_RAW(format, ...) debug_log_raw((format), ##__VA_ARGS__)

#else

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOG(format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGD(format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGI(format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGW(format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGE(format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOG_TAG(tag, format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGD_TAG(tag, format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGI_TAG(tag, format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGW_TAG(tag, format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOGE_TAG(tag, format, ...) do { } while (0)

/** @brief Debug LOG 关闭时不求值的空实现。 */
#define LOG_RAW(format, ...) do { } while (0)

#endif

#endif // PRINTK_COMPONENT_DEBUG_H
