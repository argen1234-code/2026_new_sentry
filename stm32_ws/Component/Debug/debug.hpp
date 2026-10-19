/**
 ******************************************************************************
 * @file    debug.hpp
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-20
 * @brief   Debug 只读探针 C++ 薄包装。
 *
 * @note    C++ 层提供类型别名、inline 转发和可选 ProbeSet 编译期探针集合；
 *          运行时事实来源仍为 C 接口。
 * @note    ProbeSet 只做模板静态检查和 C 接口转发，不引入虚函数、RTTI 或动态分派。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_DEBUG_HPP
#define PRINTK_COMPONENT_DEBUG_HPP

#include "debug.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Debug {

/** @brief Debug 状态码。 */
using Status = DebugStatus;

/** @brief Debug 探针类型。 */
using Type = DebugType;

/** @brief Debug 探针值。 */
using Value = DebugValue;

/** @brief Debug 探针句柄。 */
using ProbeHandle = DebugProbeHandle;

/** @brief Debug 探针快照。 */
using ProbeSnapshot = DebugProbeSnapshot;

/** @brief Debug 统计信息。 */
using Stats = DebugStats;

/** @brief Debug profiler 槽位快照。 */
using ProfilerSlot = DebugProfilerSlot;

/** @brief Debug profiler 统计信息。 */
using ProfilerStats = DebugProfilerStats;

/** @brief Debug LOG 等级。 */
using LogLevel = DebugLogLevel;

/** @brief Debug LOG 输出函数。 */
using LogTxFn = DebugLogTxFn;

/**
 * @brief float 探针标签基类。
 *
 * 派生类型需要提供 `static const char *name()`。
 */
struct F32Probe {
    static constexpr Type type = DebugTypeF32; /**< 探针类型。 */
};

/**
 * @brief uint32_t 探针标签基类。
 *
 * 派生类型需要提供 `static const char *name()`。
 */
struct U32Probe {
    static constexpr Type type = DebugTypeU32; /**< 探针类型。 */
};

/**
 * @brief int32_t 探针标签基类。
 *
 * 派生类型需要提供 `static const char *name()`。
 */
struct I32Probe {
    static constexpr Type type = DebugTypeI32; /**< 探针类型。 */
};

/**
 * @brief bool 探针标签基类。
 *
 * 派生类型需要提供 `static const char *name()`。
 */
struct BoolProbe {
    static constexpr Type type = DebugTypeBool; /**< 探针类型。 */
};

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */

/**
 * @brief 探针标签合法性检查。
 *
 * @tparam Probe 探针标签类型。
 */
template<typename Probe, typename = void>
struct IsProbe : std::false_type {
};

/**
 * @brief 探针标签合法性检查特化。
 *
 * @tparam Probe 探针标签类型。
 */
template<typename Probe>
struct IsProbe<Probe, std::void_t<decltype(Probe::type), decltype(Probe::name())>>
    : std::integral_constant<bool,
                             (Probe::type == DebugTypeF32) ||
                             (Probe::type == DebugTypeU32) ||
                             (Probe::type == DebugTypeI32) ||
                             (Probe::type == DebugTypeBool)> {
};

/**
 * @brief 单个 C++ 探针运行时句柄。
 *
 * @tparam Probe 探针标签类型。
 */
template<typename Probe>
class ProbeRef {
public:
    static_assert(IsProbe<Probe>::value, "invalid debug probe tag");

    /**
     * @brief 写入 float 值。
     *
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    Status set(float value, uint64_t timestamp_us = 0ULL)
    {
        static_assert(Probe::type == DebugTypeF32, "probe type is not float");
        return debug_probe_f32_ts(&handle_, Probe::name(), value, timestamp_us);
    }

    /**
     * @brief 写入 uint32_t 值。
     *
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    Status set(uint32_t value, uint64_t timestamp_us = 0ULL)
    {
        static_assert(Probe::type == DebugTypeU32, "probe type is not uint32_t");
        return debug_probe_u32_ts(&handle_, Probe::name(), value, timestamp_us);
    }

    /**
     * @brief 写入 int32_t 值。
     *
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    Status set(int32_t value, uint64_t timestamp_us = 0ULL)
    {
        static_assert(Probe::type == DebugTypeI32, "probe type is not int32_t");
        return debug_probe_i32_ts(&handle_, Probe::name(), value, timestamp_us);
    }

    /**
     * @brief 写入 bool 值。
     *
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    Status set(bool value, uint64_t timestamp_us = 0ULL)
    {
        static_assert(Probe::type == DebugTypeBool, "probe type is not bool");
        return debug_probe_bool_ts(&handle_, Probe::name(), value, timestamp_us);
    }

private:
    ProbeHandle handle_ = DEBUG_PROBE_HANDLE_INVALID; /**< C 层探针句柄。 */
};

/**
 * @brief C++ 探针集合。
 *
 * @tparam Probes 探针标签列表。
 */
template<typename... Probes>
class ProbeSet {
public:
    static_assert(sizeof...(Probes) <= DEBUG_PROBE_MAX, "too many debug probes");
    static_assert((IsProbe<Probes>::value && ...), "invalid debug probe tag");

    /** @brief 探针数量。 */
    static constexpr std::size_t size = sizeof...(Probes);

    /**
     * @brief 按标签读取一个探针句柄。
     *
     * @tparam Probe 探针标签类型。
     * @return 返回探针运行时句柄。
     */
    template<typename Probe>
    ProbeRef<Probe> &get()
    {
        static_assert((std::is_same<Probe, Probes>::value || ...),
                      "probe is not in this ProbeSet");
        return slot<Probe>();
    }

    /**
     * @brief 按标签写入 float 探针。
     *
     * @tparam Probe 探针标签类型。
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    template<typename Probe>
    Status f32(float value, uint64_t timestamp_us = 0ULL)
    {
        return get<Probe>().set(value, timestamp_us);
    }

    /**
     * @brief 按标签写入 uint32_t 探针。
     *
     * @tparam Probe 探针标签类型。
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    template<typename Probe>
    Status u32(uint32_t value, uint64_t timestamp_us = 0ULL)
    {
        return get<Probe>().set(value, timestamp_us);
    }

    /**
     * @brief 按标签写入 int32_t 探针。
     *
     * @tparam Probe 探针标签类型。
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    template<typename Probe>
    Status i32(int32_t value, uint64_t timestamp_us = 0ULL)
    {
        return get<Probe>().set(value, timestamp_us);
    }

    /**
     * @brief 按标签写入 bool 探针。
     *
     * @tparam Probe 探针标签类型。
     * @param value 探针值。
     * @param timestamp_us 时间戳，0 表示使用平台时间。
     * @return 返回 Debug 状态码。
     */
    template<typename Probe>
    Status boolean(bool value, uint64_t timestamp_us = 0ULL)
    {
        return get<Probe>().set(value, timestamp_us);
    }

private:
    /**
     * @brief 获取标签对应的静态槽位。
     *
     * @tparam Probe 探针标签类型。
     * @return 返回探针运行时句柄。
     */
    template<typename Probe>
    static ProbeRef<Probe> &slot()
    {
        static ProbeRef<Probe> ref;
        return ref;
    }
};

/** @brief 初始化 Debug 探针内核。 */
inline Status init()
{
    return debug_init();
}

/** @brief 读取平台微秒时间戳。 */
inline uint64_t time_us()
{
    return debug_time_us();
}

/** @brief 读取当前探针数量。 */
inline uint16_t count()
{
    return debug_probe_count();
}

/** @brief 读取指定索引探针快照。 */
inline Status snapshot(uint16_t index, ProbeSnapshot &out)
{
    return debug_probe_snapshot(index, &out);
}

/** @brief 读取 Debug 统计信息。 */
inline Status stats(Stats &out)
{
    return debug_stats(&out);
}

/** @brief 对当前上下文采样一次。 */
inline Status sample_current()
{
    return debug_profiler_sample_current();
}

/** @brief 读取 profiler 槽位快照。 */
inline Status profiler_snapshot(uint16_t index, ProfilerSlot &out)
{
    return debug_profiler_snapshot(index, &out);
}

/** @brief 读取 profiler 统计信息。 */
inline Status profiler_stats(ProfilerStats &out)
{
    return debug_profiler_stats(&out);
}

/** @brief 绑定 Debug LOG 输出函数。 */
inline Status log_custom(LogTxFn tx_fn, void *user = nullptr)
{
    return debug_log_custom(tx_fn, user);
}

/** @brief 判断 Debug LOG 是否已经绑定输出端。 */
inline bool log_ready()
{
    return debug_log_ready();
}

/** @brief 输出带等级和 tag 的 Printk 风格文本。 */
template<typename... Args>
inline Status log(LogLevel level, const char *tag, const char *format, Args... args)
{
    return debug_log_printf(level, tag, format, args...);
}

/** @brief 输出不带等级前缀的 Printk 风格文本。 */
template<typename... Args>
inline Status log_raw(const char *format, Args... args)
{
    return debug_log_raw(format, args...);
}

} // namespace Component::Debug

#endif // PRINTK_COMPONENT_DEBUG_HPP
