/**
 ******************************************************************************
 * @file    rc_sbus.c
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-19
 * @brief   SBUS 遥控协议解析实现。
 *
 * @note    本文件只做 SBUS 物理通道解析和 DBUS 风格状态归一化，不包含机器人控制语义。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "rc_sbus.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */
/** @brief SBUS 帧头。 */
#define RC_SBUS_HEAD ((uint8_t)0x0FU)

/** @brief SBUS 帧尾。 */
#define RC_SBUS_END ((uint8_t)0x00U)

/** @brief SBUS 拨杆上/中档分界阈值。 */
#define RC_SBUS_TOGGLE_A ((int16_t)600)

/** @brief SBUS 拨杆中/下档分界阈值。 */
#define RC_SBUS_TOGGLE_B ((int16_t)1400)

/** @brief SBUS 摇杆死区。 */
#define RC_SBUS_DEAD_AREA ((int16_t)120)

/** @brief SBUS 离线标志 byte。 */
#define RC_SBUS_OFFLINE_FLAG ((uint8_t)0x0CU)

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 对 SBUS 控制量执行死区。
 *
 * @param value 输入控制量。
 * @return 死区处理后的控制量。
 */
static int16_t sbus_deadband(int16_t value)
{
    if ((value > (int16_t)-RC_SBUS_DEAD_AREA) &&
        (value < RC_SBUS_DEAD_AREA)) {
        return 0;
    }

    return value;
}

/**
 * @brief 将 SBUS 原始通道映射为 DBUS 风格物理输入状态。
 *
 * @param sbus SBUS 原始状态。
 * @param upper 非 0 使用上遥控符号映射。
 * @param tick_ms 本帧接收时间。
 * @param out DBUS 风格物理输入状态输出。
 */
static void rc_sbus_map(RcSbusState *sbus, uint8_t upper, uint32_t tick_ms, RcDbusState *out)
{
    if ((sbus == NULL) || (out == NULL)) {
        return;
    }

    sbus->joy.right_hori = (int16_t)(sbus->rc.ch[3] - RC_CH_OFFSET);
    sbus->joy.right_vert = (int16_t)(sbus->rc.ch[2] - RC_CH_OFFSET);
    sbus->joy.left_hori = (int16_t)(sbus->rc.ch[0] - RC_CH_OFFSET);
    sbus->joy.left_vert = (int16_t)(sbus->rc.ch[1] - RC_CH_OFFSET);

    sbus->joy.left_hori = sbus_deadband(sbus->joy.left_hori);
    sbus->joy.right_hori = sbus_deadband(sbus->joy.right_hori);
    sbus->joy.right_vert = sbus_deadband(sbus->joy.right_vert);

    if (sbus->rc.ch[4] < RC_SBUS_TOGGLE_A) {
        out->rc.sw[1] = RC_SW_UP;
    } else if (sbus->rc.ch[4] <= RC_SBUS_TOGGLE_B) {
        out->rc.sw[1] = RC_SW_MID;
    } else {
        out->rc.sw[1] = RC_SW_DOWN;
    }

    if (sbus->rc.ch[5] < RC_SBUS_TOGGLE_A) {
        out->rc.sw[0] = RC_SW_UP;
    } else if (sbus->rc.ch[5] <= RC_SBUS_TOGGLE_B) {
        out->rc.sw[0] = RC_SW_MID;
    } else {
        out->rc.sw[0] = RC_SW_DOWN;
    }

    out->rc.ch[0] = (int16_t)(-sbus->joy.right_hori);
    out->rc.ch[1] = (upper != 0U) ? sbus->joy.right_vert : (int16_t)(-sbus->joy.right_vert);
    out->rc.ch[2] = (int16_t)(-sbus->joy.left_hori);
    out->rc.ch[3] = (int16_t)(-sbus->joy.left_vert);
    out->rc.ch[4] = 0;
    out->last_update_tick = tick_ms;
    out->online = (uint8_t)((sbus->online != 0U) ? 1U : 0U);
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 解析 25 byte SBUS 帧。
 *
 * @param buf 原始 SBUS 帧。
 * @param len 原始 SBUS 帧长度。
 * @param upper 非 0 使用上遥控符号映射。
 * @param tick_ms 本帧接收时间。
 * @param out DBUS 风格物理输入状态输出。
 * @param sbus SBUS 原始状态输出。
 * @return 解析状态。
 */
static RcStatus rc_sbus_parse_core(const uint8_t *buf,
                                   uint16_t len,
                                   uint8_t upper,
                                   uint32_t tick_ms,
                                   RcDbusState *out,
                                   RcSbusState *sbus)
{
    RcDbusState next;
    RcSbusState raw;

    if ((buf == NULL) || (out == NULL) || (sbus == NULL)) {
        return RcNull;
    }

    if (len != RC_SBUS_LEN) {
        return RcLen;
    }

    if ((buf[0] != RC_SBUS_HEAD) || (buf[24] != RC_SBUS_END)) {
        return RcFrameErr;
    }

    memset(&next, 0, sizeof(next));
    memset(&raw, 0, sizeof(raw));
    raw.online = (buf[23] == RC_SBUS_OFFLINE_FLAG) ? 0U : 1U;
    raw.rc.ch[0] = (int16_t)((buf[1] | ((uint16_t)buf[2] << 8)) & 0x07FFU);
    raw.rc.ch[1] = (int16_t)(((buf[2] >> 3) | ((uint16_t)buf[3] << 5)) & 0x07FFU);
    raw.rc.ch[2] = (int16_t)(((buf[3] >> 6) | ((uint16_t)buf[4] << 2) | ((uint16_t)buf[5] << 10)) & 0x07FFU);
    raw.rc.ch[3] = (int16_t)(((buf[5] >> 1) | ((uint16_t)buf[6] << 7)) & 0x07FFU);
    raw.rc.ch[4] = (int16_t)(((buf[6] >> 4) | ((uint16_t)buf[7] << 4)) & 0x07FFU);
    raw.rc.ch[5] = (int16_t)(((buf[7] >> 7) | ((uint16_t)buf[8] << 1) | ((uint16_t)buf[9] << 9)) & 0x07FFU);
    raw.rc.ch[6] = (int16_t)(((buf[9] >> 2) | ((uint16_t)buf[10] << 6)) & 0x07FFU);
    raw.rc.ch[7] = (int16_t)(((buf[10] >> 5) | ((uint16_t)buf[11] << 3)) & 0x07FFU);
    raw.rc.ch[8] = (int16_t)((buf[12] | ((uint16_t)buf[13] << 8)) & 0x07FFU);
    raw.rc.ch[9] = (int16_t)(((buf[13] >> 3) | ((uint16_t)buf[14] << 5)) & 0x07FFU);
    raw.sbus_receiver_tick = tick_ms;

    rc_sbus_map(&raw, upper, tick_ms, &next);
    if (rc_dbus_state_valid(&next) == 0U) {
        return RcValue;
    }

    *out = next;
    *sbus = raw;
    return RcOk;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
RcStatus rc_sbus_parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, RcDbusState *out, RcSbusState *sbus)
{
    return rc_sbus_parse_core(buf, len, 0U, tick_ms, out, sbus);
}

RcStatus rc_sbus_parse_upper(const uint8_t *buf,
                             uint16_t len,
                             uint32_t tick_ms,
                             RcDbusState *out,
                             RcSbusState *sbus)
{
    return rc_sbus_parse_core(buf, len, 1U, tick_ms, out, sbus);
}
