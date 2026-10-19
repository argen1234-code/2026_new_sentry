/**
 ******************************************************************************
 * @file    rc_dbus.h
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-19
 * @brief   DBUS 遥控协议解析接口。
 *
 * @note    本文件只把 DBUS 字节流解析为 RcDbusState 物理输入状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_RC_DBUS_H
#define PRINTK_DEVICE_RC_DBUS_H

#include "rc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 解析 18 byte DBUS 帧。
 *
 * @param buf 原始 DBUS 帧缓冲区。
 * @param len 原始 DBUS 帧长度，必须为 RC_DBUS_LEN。
 * @param tick_ms 本帧接收时间，单位 ms tick。
 * @param out DBUS 风格物理输入状态输出指针。
 * @return 解析状态。
 */
RcStatus rc_dbus_parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, RcDbusState *out);

/**
 * @brief 将 18 byte 原始载荷打包为参考转发帧。
 *
 * 帧格式为 0xA6 + 18 byte payload + 前 19 byte 累加和。
 *
 * @param payload 18 byte 原始载荷。
 * @param len 原始载荷长度，必须为 RC_DBUS_LEN。
 * @param out 20 byte 输出缓冲区。
 * @param out_len 输出缓冲区长度，必须不小于 RC_TX_LEN。
 * @return 打包状态。
 */
RcStatus rc_dbus_pack(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_len);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_RC_DBUS_H
