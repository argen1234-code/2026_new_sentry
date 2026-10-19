/**
 ******************************************************************************
 * @file    debug_log.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug printf 风格日志接口。
 *
 * @note    本文件只定义格式化日志和输出回调，不直接绑定 USB、UART 或 VOFA。
 * @note    LOG 宏默认追加 CRLF，并可通过 Communication/VOFA 绑定到 FireWater
 *          文本输出。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_DEBUG_LOG_H
#define PRINTK_COMPONENT_DEBUG_LOG_H

#include "debug_probe.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DEBUG_LOG_SIZE
/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief Debug LOG 单帧格式化缓冲区长度，单位 byte。 */
#define DEBUG_LOG_SIZE 192U
#endif

/** @brief Debug LOG 输出端 busy 返回值。 */
#define DEBUG_LOG_TX_BUSY (-2)

/**
 * @brief Debug LOG 等级。
 */
typedef enum
{
    DebugLogDebug = 0, /**< 调试信息。 */
    DebugLogInfo = 1,  /**< 普通信息。 */
    DebugLogWarn = 2,  /**< 警告信息。 */
    DebugLogError = 3  /**< 错误信息。 */
} DebugLogLevel;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief Debug LOG 输出函数类型。
 *
 * @param user 用户上下文。
 * @param data 待输出字节流。
 * @param length 待输出长度，单位 byte。
 * @return 成功返回输出字节数；busy 返回 DEBUG_LOG_TX_BUSY；其他失败返回负数。
 */
typedef int32_t (*DebugLogTxFn)(void *user, const char *data, uint16_t length);

/**
 * @brief 重置 Debug LOG 输出端。
 *
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_log_reset(void);

/**
 * @brief 绑定 Debug LOG 输出端。
 *
 * @param tx_fn 输出函数。
 * @param user 输出函数上下文。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_log_custom(DebugLogTxFn tx_fn, void *user);

/**
 * @brief 判断 Debug LOG 是否已经绑定输出端。
 *
 * @return 已绑定返回 true，否则返回 false。
 */
bool debug_log_ready(void);

/**
 * @brief 输出带等级和可选 tag 的格式化日志。
 *
 * @param level 日志等级。
 * @param tag 可选 tag；传 NULL 或空字符串时不输出 tag。
 * @param format printf 风格格式字符串。
 * @param ... 可变参数。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_log_printf(DebugLogLevel level, const char *tag, const char *format, ...);

/**
 * @brief 输出带等级和可选 tag 的格式化日志。
 *
 * @param level 日志等级。
 * @param tag 可选 tag；传 NULL 或空字符串时不输出 tag。
 * @param format printf 风格格式字符串。
 * @param args 可变参数列表。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_log_vprintf(DebugLogLevel level,
                              const char *tag,
                              const char *format,
                              va_list args);

/**
 * @brief 输出不带等级前缀的原始格式化文本。
 *
 * @param format printf 风格格式字符串。
 * @param ... 可变参数。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_log_raw(const char *format, ...);

/**
 * @brief 输出不带等级前缀的原始格式化文本。
 *
 * @param format printf 风格格式字符串。
 * @param args 可变参数列表。
 * @return 返回 Debug 状态码。
 */
DebugStatus debug_log_vraw(const char *format, va_list args);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_DEBUG_LOG_H
