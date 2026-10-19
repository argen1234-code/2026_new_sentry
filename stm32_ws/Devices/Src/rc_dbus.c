/**
 ******************************************************************************
 * @file    rc_dbus.c
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-19
 * @brief   DBUS 遥控协议解析和转发打包实现。
 *
 * @note    DBUS 通道为 11 bit 无符号量，本实现统一转换为以 0 为中心的物理输入量。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "rc_dbus.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
RcStatus rc_dbus_parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, RcDbusState *out)
{
    RcDbusState next;

    if ((buf == NULL) || (out == NULL)) {
        return RcNull;
    }

    if (len != RC_DBUS_LEN) {
        return RcLen;
    }

    memset(&next, 0, sizeof(next));
    next.rc.ch[0] = (int16_t)((buf[0] | ((uint16_t)buf[1] << 8)) & 0x07FFU);
    next.rc.ch[1] = (int16_t)(((buf[1] >> 3) | ((uint16_t)buf[2] << 5)) & 0x07FFU);
    next.rc.ch[2] = (int16_t)(((buf[2] >> 6) | ((uint16_t)buf[3] << 2) | ((uint16_t)buf[4] << 10)) & 0x07FFU);
    next.rc.ch[3] = (int16_t)(((buf[4] >> 1) | ((uint16_t)buf[5] << 7)) & 0x07FFU);
    next.rc.sw[0] = (uint8_t)((buf[5] >> 4) & 0x03U);
    next.rc.sw[1] = (uint8_t)(((buf[5] >> 4) & 0x0CU) >> 2);
    next.mouse.x = (int16_t)(buf[6] | ((uint16_t)buf[7] << 8));
    next.mouse.y = (int16_t)(buf[8] | ((uint16_t)buf[9] << 8));
    next.mouse.z = (int16_t)(buf[10] | ((uint16_t)buf[11] << 8));
    next.mouse.press_l = buf[12];
    next.mouse.press_r = buf[13];
    next.key.v = (uint16_t)(buf[14] | ((uint16_t)buf[15] << 8));
    next.rc.ch[4] = (int16_t)(buf[16] | ((uint16_t)buf[17] << 8));

    next.rc.ch[0] = (int16_t)(next.rc.ch[0] - RC_CH_OFFSET);
    next.rc.ch[1] = (int16_t)(next.rc.ch[1] - RC_CH_OFFSET);
    next.rc.ch[2] = (int16_t)(next.rc.ch[2] - RC_CH_OFFSET);
    next.rc.ch[3] = (int16_t)(next.rc.ch[3] - RC_CH_OFFSET);
    next.rc.ch[4] = (int16_t)(next.rc.ch[4] - RC_CH_OFFSET);
    next.last_update_tick = tick_ms;
    next.online = 1U;

    if (rc_dbus_state_valid(&next) == 0U) {
        return RcValue;
    }

    *out = next;
    return RcOk;
}

RcStatus rc_dbus_pack(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_len)
{
    uint8_t i;

    if ((payload == NULL) || (out == NULL)) {
        return RcNull;
    }

    if ((len != RC_DBUS_LEN) || (out_len < RC_TX_LEN)) {
        return RcLen;
    }

    out[0] = 0xA6U;
    memcpy(&out[1], payload, RC_DBUS_LEN);
    out[19] = 0U;
    for (i = 0U; i < 19U; i++) {
        out[19] = (uint8_t)(out[19] + out[i]);
    }

    return RcOk;
}
