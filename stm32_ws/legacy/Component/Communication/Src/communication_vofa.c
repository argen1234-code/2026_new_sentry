/**
 ******************************************************************************
 * @file    communication_vofa.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   VOFA 调试输出包装实现。
 *
 * @note    本文件只完成 FireWater 文本帧和 JustFloat 二进制帧打包。
 * @note    默认发送端为 USB CDC ACM raw write，CDC busy 时立即返回 Busy。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "communication_vofa.h"

#include "bsp_cdc.h"
#include "debug_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief 自定义发送函数用于表达忙状态的返回值。 */
#define COMM_VOFA_TX_BUSY (-2)

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief VOFA JustFloat 固定帧尾。 */
static const uint8_t justfloat_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

/**
 * @brief VOFA 输出上下文。
 */
typedef struct
{
    CommVofaTxFn tx;                    /**< 当前发送函数。 */
    void *user;                         /**< 当前发送函数上下文。 */
    uint8_t buffer[COMM_VOFA_TX_SIZE];  /**< 打包临时缓冲区。 */
    uint8_t ready;                      /**< 初始化完成标志。 */
} CommVofa;

/** @brief VOFA 输出单例。 */
static CommVofa vofa;

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
 * @brief USB CDC raw 发送适配器。
 *
 * @param user 用户上下文，当前未使用。
 * @param data 待发送数据。
 * @param length 待发送长度，单位 byte。
 * @return 成功返回 length，CDC busy 返回 COMM_VOFA_TX_BUSY，失败返回 -1。
 */
static int32_t cdc_tx(void *user, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    (void)user;

    if ((data == NULL) || (length == 0U)) {
        return -1;
    }

    status = bsp_cdc_write_raw(data, length);
    if (status == HAL_OK) {
        return (int32_t)length;
    }
    if (status == HAL_BUSY) {
        return COMM_VOFA_TX_BUSY;
    }
    return -1;
}

/**
 * @brief 发送已经打包完成的数据。
 *
 * @param data 待发送数据。
 * @param length 待发送长度，单位 byte。
 * @return 返回通讯组件状态码。
 */
static CommunicationStatus tx_buf(const uint8_t *data, uint16_t length)
{
    int32_t sent;

    if (vofa.ready == 0U) {
        return CommunicationState;
    }
    if ((vofa.tx == NULL) || (data == NULL) || (length == 0U)) {
        return CommunicationArg;
    }

    sent = vofa.tx(vofa.user, data, length);
    if (sent == (int32_t)length) {
        return CommunicationOk;
    }
    if (sent == COMM_VOFA_TX_BUSY) {
        return CommunicationBusy;
    }
    return CommunicationIo;
}

/**
 * @brief Debug LOG 到 VOFA FireWater 文本输出的适配器。
 *
 * @param user 用户上下文，当前未使用。
 * @param data 待发送文本。
 * @param length 待发送长度，单位 byte。
 * @return 成功返回 length，busy 返回 DEBUG_LOG_TX_BUSY，失败返回 -1。
 */
static int32_t log_tx(void *user, const char *data, uint16_t length)
{
    CommunicationStatus status;

    (void)user;

    status = comm_vofa_raw(data, length);
    if (status == CommunicationOk) {
        return (int32_t)length;
    }
    if (status == CommunicationBusy) {
        return DEBUG_LOG_TX_BUSY;
    }
    return -1;
}

/**
 * @brief 将 Debug 状态码映射为通讯组件状态码。
 *
 * @param status Debug 状态码。
 * @return 返回通讯组件状态码。
 */
static CommunicationStatus map_debug_status(DebugStatus status)
{
    switch (status) {
    case DebugOk:
        return CommunicationOk;
    case DebugNull:
        return CommunicationNull;
    case DebugBusy:
        return CommunicationBusy;
    case DebugSmall:
        return CommunicationSmall;
    case DebugState:
        return CommunicationState;
    case DebugIo:
        return CommunicationIo;
    default:
        return CommunicationIo;
    }
}

/**
 * @brief 打包 FireWater 文本帧。
 *
 * @param format printf 风格格式字符串。
 * @param args 可变参数列表。
 * @param out_len 输出帧长度。
 * @return 返回通讯组件状态码。
 */
static CommunicationStatus pack_fire(const char *format, va_list args, uint16_t *out_len)
{
    int written;

    if ((format == NULL) || (out_len == NULL)) {
        return CommunicationArg;
    }

    written = vsnprintf((char *)vofa.buffer, sizeof(vofa.buffer), format, args);
    if (written < 0) {
        return CommunicationIo;
    }
    if ((uint32_t)written >= sizeof(vofa.buffer)) {
        return CommunicationSmall;
    }

    *out_len = (uint16_t)written;
    return CommunicationOk;
}

/**
 * @brief 打包 JustFloat 二进制帧。
 *
 * @param data float 数组首地址。
 * @param count float 元素数量。
 * @param out_len 输出帧长度。
 * @return 返回通讯组件状态码。
 */
static CommunicationStatus pack_float(const float *data, uint16_t count, uint16_t *out_len)
{
    uint32_t payload_len;

    if (out_len == NULL) {
        return CommunicationArg;
    }
    if ((data == NULL) && (count != 0U)) {
        return CommunicationArg;
    }

    payload_len = (uint32_t)count * (uint32_t)sizeof(float);
    if ((payload_len + sizeof(justfloat_tail)) > sizeof(vofa.buffer)) {
        return CommunicationSmall;
    }

    if (payload_len != 0U) {
        (void)memcpy(vofa.buffer, data, payload_len);
    }
    (void)memcpy(&vofa.buffer[payload_len], justfloat_tail, sizeof(justfloat_tail));

    *out_len = (uint16_t)(payload_len + sizeof(justfloat_tail));
    return CommunicationOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
CommunicationStatus comm_vofa_init_cdc(void)
{
    (void)memset(&vofa, 0, sizeof(vofa));
    vofa.tx = cdc_tx;
    vofa.user = NULL;
    vofa.ready = 1U;
    return CommunicationOk;
}

CommunicationStatus comm_vofa_custom(CommVofaTxFn tx_fn, void *user)
{
    if (tx_fn == NULL) {
        return CommunicationNull;
    }

    (void)memset(&vofa, 0, sizeof(vofa));
    vofa.tx = tx_fn;
    vofa.user = user;
    vofa.ready = 1U;
    return CommunicationOk;
}

bool comm_vofa_ready(void)
{
    return (vofa.ready != 0U);
}

CommunicationStatus comm_vofa_bind_log(void)
{
    if (vofa.ready == 0U) {
        return CommunicationState;
    }

    return map_debug_status(debug_log_custom(log_tx, NULL));
}

CommunicationStatus comm_vofa_raw(const void *data, uint16_t length)
{
    return tx_buf((const uint8_t *)data, length);
}

CommunicationStatus comm_vofa_fire(const char *format, ...)
{
    CommunicationStatus status;
    uint16_t length = 0U;
    va_list args;

    va_start(args, format);
    status = pack_fire(format, args, &length);
    va_end(args);

    if (status != CommunicationOk) {
        return status;
    }

    return tx_buf(vofa.buffer, length);
}

CommunicationStatus comm_vofa_float(const float *data, uint16_t count)
{
    CommunicationStatus status;
    uint16_t length = 0U;

    status = pack_float(data, count, &length);
    if (status != CommunicationOk) {
        return status;
    }

    return tx_buf(vofa.buffer, length);
}
