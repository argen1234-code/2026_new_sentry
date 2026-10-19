/**
 ******************************************************************************
 * @file    debug_log.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Debug printf 风格日志实现。
 *
 * @note    本实现使用固定缓冲区格式化，不使用动态内存。busy 标志只用于保护
 *          单例缓冲区，避免并发日志互相覆盖。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "debug_log.h"

#include "debug_platform.h"

#include <stdio.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */

/**
 * @brief Debug LOG 上下文。
 */
typedef struct
{
    DebugLogTxFn tx;                 /**< 输出函数。 */
    void *user;                      /**< 输出函数上下文。 */
    char buffer[DEBUG_LOG_SIZE];     /**< 格式化缓冲区。 */
    volatile uint8_t ready;          /**< 非 0 表示输出端已绑定。 */
    volatile uint8_t busy;           /**< 非 0 表示缓冲区正在使用。 */
} DebugLogCtx;

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief Debug LOG 单例上下文。 */
static DebugLogCtx log_ctx;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 读取日志等级字符。
 *
 * @param level 日志等级。
 * @return 日志等级字符。
 */
static char log_level_char(DebugLogLevel level)
{
    switch (level) {
    case DebugLogDebug:
        return 'D';
    case DebugLogInfo:
        return 'I';
    case DebugLogWarn:
        return 'W';
    case DebugLogError:
        return 'E';
    default:
        return '?';
    }
}

/**
 * @brief 尝试占用 LOG 格式化缓冲区。
 *
 * @return 成功返回 DebugOk，忙返回 DebugBusy。
 */
static DebugStatus log_take(void)
{
    DebugIrqState state = debug_lock();

    if (log_ctx.busy != 0U) {
        debug_unlock(state);
        return DebugBusy;
    }
    log_ctx.busy = 1U;
    debug_unlock(state);
    return DebugOk;
}

/**
 * @brief 释放 LOG 格式化缓冲区。
 */
static void log_release(void)
{
    DebugIrqState state = debug_lock();

    log_ctx.busy = 0U;
    debug_unlock(state);
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 输出已经完成格式化的日志缓冲区。
 *
 * @param length 输出长度，单位 byte。
 * @return 返回 Debug 状态码。
 */
static DebugStatus log_send(uint16_t length)
{
    int32_t sent;
    DebugLogTxFn tx;
    void *user;

    if (log_ctx.ready == 0U) {
        return DebugState;
    }

    tx = log_ctx.tx;
    user = log_ctx.user;
    if ((tx == NULL) || (length == 0U)) {
        return DebugState;
    }

    sent = tx(user, log_ctx.buffer, length);
    if (sent == (int32_t)length) {
        return DebugOk;
    }
    if (sent == DEBUG_LOG_TX_BUSY) {
        return DebugBusy;
    }
    return DebugIo;
}

/**
 * @brief 向缓冲区追加 CRLF。
 *
 * @param length 当前字符串长度输入输出。
 * @return 返回 Debug 状态码。
 */
static DebugStatus log_append_crlf(uint16_t *length)
{
    uint16_t pos;

    if (length == NULL) {
        return DebugNull;
    }

    pos = *length;
    if ((uint32_t)pos + 2UL >= (uint32_t)sizeof(log_ctx.buffer)) {
        return DebugSmall;
    }

    log_ctx.buffer[pos++] = '\r';
    log_ctx.buffer[pos++] = '\n';
    log_ctx.buffer[pos] = '\0';
    *length = pos;
    return DebugOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
DebugStatus debug_log_reset(void)
{
    DebugIrqState state = debug_lock();

    (void)memset(&log_ctx, 0, sizeof(log_ctx));
    debug_unlock(state);
    return DebugOk;
}

DebugStatus debug_log_custom(DebugLogTxFn tx_fn, void *user)
{
    DebugIrqState state;

    if (tx_fn == NULL) {
        return DebugNull;
    }

    state = debug_lock();
    log_ctx.tx = tx_fn;
    log_ctx.user = user;
    log_ctx.ready = 1U;
    log_ctx.busy = 0U;
    debug_unlock(state);
    return DebugOk;
}

bool debug_log_ready(void)
{
    return (log_ctx.ready != 0U);
}

DebugStatus debug_log_vprintf(DebugLogLevel level,
                              const char *tag,
                              const char *format,
                              va_list args)
{
    DebugStatus status;
    uint16_t length;
    int written;
    int prefix_len;

    if (format == NULL) {
        return DebugNull;
    }
    if (log_ctx.ready == 0U) {
        return DebugState;
    }

    status = log_take();
    if (status != DebugOk) {
        return status;
    }

    if ((tag != NULL) && (tag[0] != '\0')) {
        prefix_len = snprintf(log_ctx.buffer,
                              sizeof(log_ctx.buffer),
                              "[%c][%s] ",
                              log_level_char(level),
                              tag);
    } else {
        prefix_len = snprintf(log_ctx.buffer,
                              sizeof(log_ctx.buffer),
                              "[%c] ",
                              log_level_char(level));
    }

    if ((prefix_len < 0) || ((uint32_t)prefix_len >= sizeof(log_ctx.buffer))) {
        log_release();
        return DebugSmall;
    }

    written = vsnprintf(&log_ctx.buffer[prefix_len],
                        sizeof(log_ctx.buffer) - (uint32_t)prefix_len,
                        format,
                        args);
    if ((written < 0) ||
        (((uint32_t)prefix_len + (uint32_t)written) >= sizeof(log_ctx.buffer))) {
        log_release();
        return DebugSmall;
    }

    length = (uint16_t)((uint32_t)prefix_len + (uint32_t)written);
    status = log_append_crlf(&length);
    if (status == DebugOk) {
        status = log_send(length);
    }

    log_release();
    return status;
}

DebugStatus debug_log_printf(DebugLogLevel level, const char *tag, const char *format, ...)
{
    DebugStatus status;
    va_list args;

    va_start(args, format);
    status = debug_log_vprintf(level, tag, format, args);
    va_end(args);
    return status;
}

DebugStatus debug_log_vraw(const char *format, va_list args)
{
    DebugStatus status;
    uint16_t length;
    int written;

    if (format == NULL) {
        return DebugNull;
    }
    if (log_ctx.ready == 0U) {
        return DebugState;
    }

    status = log_take();
    if (status != DebugOk) {
        return status;
    }

    written = vsnprintf(log_ctx.buffer, sizeof(log_ctx.buffer), format, args);
    if ((written < 0) || ((uint32_t)written >= sizeof(log_ctx.buffer))) {
        log_release();
        return DebugSmall;
    }

    length = (uint16_t)written;
    status = log_append_crlf(&length);
    if (status == DebugOk) {
        status = log_send(length);
    }

    log_release();
    return status;
}

DebugStatus debug_log_raw(const char *format, ...)
{
    DebugStatus status;
    va_list args;

    va_start(args, format);
    status = debug_log_vraw(format, args);
    va_end(args);
    return status;
}
