/**
 ******************************************************************************
 * @file    communication_vofa.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   VOFA 调试输出包装接口。
 *
 * @note    本文件位于 Component/Communication，默认使用 USB CDC ACM raw 发送。
 * @note    支持 FireWater 文本输出和 JustFloat float 数组输出，不绑定业务变量。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_VOFA_H
#define PRINTK_COMPONENT_COMMUNICATION_VOFA_H

#include "communication_common.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief VOFA 底层发送函数类型。
 *
 * @param user 调用方上下文。
 * @param data 待发送数据。
 * @param length 待发送长度，单位 byte。
 * @return 成功返回发送字节数；忙返回 -2；其他失败返回负数。
 */
typedef int32_t (*CommVofaTxFn)(void *user, const uint8_t *data, uint16_t length);

/**
 * @brief 初始化 VOFA 到 USB CDC ACM。
 *
 * @return 返回通讯组件状态码。
 */
CommunicationStatus comm_vofa_init_cdc(void);

/**
 * @brief 初始化 VOFA 到自定义发送端。
 *
 * @param tx_fn 自定义发送函数。
 * @param user 发送函数上下文。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus comm_vofa_custom(CommVofaTxFn tx_fn, void *user);

/**
 * @brief 判断 VOFA 输出是否已经初始化。
 *
 * @return 已初始化返回 true，否则返回 false。
 */
bool comm_vofa_ready(void);

/**
 * @brief 将 Debug LOG 输出绑定到当前 VOFA 发送端。
 *
 * 本函数只绑定 Debug 的 printf 风格文本输出，不影响 JustFloat 波形输出。
 *
 * @return 返回通讯组件状态码。
 */
CommunicationStatus comm_vofa_bind_log(void);

/**
 * @brief 发送 VOFA 原始数据。
 *
 * @param data 待发送数据。
 * @param length 待发送长度，单位 byte。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus comm_vofa_raw(const void *data, uint16_t length);

/**
 * @brief 按 VOFA FireWater 文本协议发送一帧数据。
 *
 * @param format printf 风格格式字符串。
 * @param ... 可变参数。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus comm_vofa_fire(const char *format, ...);

/**
 * @brief 按 VOFA JustFloat 协议发送 float 数组。
 *
 * 帧格式为小端 float 原始字节流，末尾固定追加 00 00 80 7F。
 *
 * @param data float 数组首地址；count 为 0 时可为空。
 * @param count float 元素数量。
 * @return 返回通讯组件状态码。
 */
CommunicationStatus comm_vofa_float(const float *data, uint16_t count);

/**
 * @brief 按数组长度自动发送 VOFA JustFloat 数据。
 *
 * @param arr float 数组对象。
 * @return 返回通讯组件状态码。
 */
#define comm_vofa_float_auto(arr) \
    comm_vofa_float((arr), (uint16_t)(sizeof(arr) / sizeof((arr)[0])))

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_COMMUNICATION_VOFA_H
