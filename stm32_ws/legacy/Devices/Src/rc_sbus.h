/**
 ******************************************************************************
 * @file    rc_sbus.h
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-19
 * @brief   SBUS 遥控协议解析接口。
 *
 * @note    本文件提供 SBUS 原始状态解析，并可同步输出 DBUS 风格物理输入状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_RC_SBUS_H
#define PRINTK_DEVICE_RC_SBUS_H

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
 * @brief 解析参考 Sentinel 路径的 25 byte SBUS 帧。
 *
 * @param buf 原始 SBUS 帧缓冲区。
 * @param len 原始 SBUS 帧长度，必须为 RC_SBUS_LEN。
 * @param tick_ms 本帧接收时间，单位 ms tick。
 * @param out DBUS 风格物理输入状态输出指针。
 * @param sbus SBUS 原始物理输入状态输出指针。
 * @return 解析状态。
 */
RcStatus rc_sbus_parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, RcDbusState *out, RcSbusState *sbus);

/**
 * @brief 解析参考上遥控路径的 25 byte SBUS 帧。
 *
 * @param buf 原始 SBUS 帧缓冲区。
 * @param len 原始 SBUS 帧长度，必须为 RC_SBUS_LEN。
 * @param tick_ms 本帧接收时间，单位 ms tick。
 * @param out DBUS 风格物理输入状态输出指针。
 * @param sbus SBUS 原始物理输入状态输出指针。
 * @return 解析状态。
 */
RcStatus rc_sbus_parse_upper(const uint8_t *buf,
                             uint16_t len,
                             uint32_t tick_ms,
                             RcDbusState *out,
                             RcSbusState *sbus);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_RC_SBUS_H
