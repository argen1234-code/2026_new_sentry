/**
 ******************************************************************************
 * @file    rc.hpp
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-19
 * @brief   遥控器设备 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/rc*.c。
 * @note    C 文件应包含 Src/rc.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_RC_HPP
#define PRINTK_DEVICE_RC_HPP

#include "Src/rc.h"
#include "Src/rc_dbus.h"
#include "Src/rc_sbus.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Rc {

/** @brief DBUS 协议标签。 */
struct DbusTag {};

/** @brief SBUS 协议标签。 */
struct SbusTag {};

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 遥控器 DMA 接收缓冲区长度。 */
inline constexpr uint16_t RxBufferSize = RC_RX_BUF_SIZE;

/** @brief 遥控器有效原始帧长度。 */
inline constexpr uint16_t FrameLength = RC_FRAME_LEN;

/** @brief DBUS 原始帧长度。 */
inline constexpr uint16_t DbusLength = RC_DBUS_LEN;

/** @brief SBUS 原始帧长度。 */
inline constexpr uint16_t SbusLength = RC_SBUS_LEN;

/** @brief 遥控器协议解析状态。 */
using Status = RcStatus;

/** @brief 遥控器原始帧快照。 */
using Frame = RcFrame;

/** @brief 遥控器接收诊断快照。 */
using Diag = RcDiag;

/** @brief DBUS 风格物理输入状态。 */
using DbusState = RcDbusState;

/** @brief SBUS 原始物理输入状态。 */
using SbusState = RcSbusState;

/** @brief 初始化遥控器接收。 */
inline HAL_StatusTypeDef init()
{
    return rc_init();
}

/** @brief 处理遥控器 RX idle 事件。 */
inline void rx_event(uint16_t size)
{
    rc_rx_event(size);
}

/** @brief 处理遥控器 UART 错误事件。 */
inline void error_event()
{
    rc_error_event();
}

/** @brief 获取遥控器原始帧。 */
inline bool frame(Frame *out)
{
    return rc_frame(out) != 0U;
}

/** @brief 获取遥控器诊断信息。 */
inline bool diag(Diag *out)
{
    return rc_diag(out) != 0U;
}

/** @brief 清空 DBUS 风格物理输入状态。 */
inline void clear(DbusState *state)
{
    rc_dbus_state_clear(state);
}

/** @brief 判断 DBUS 风格物理输入是否有效。 */
inline bool valid(const DbusState *state)
{
    return rc_dbus_state_valid(state) != 0U;
}

/**
 * @brief 遥控协议模板。
 *
 * @tparam Protocol 遥控协议标签。
 */
template<typename Protocol>
struct Proto;

/**
 * @brief DBUS 协议特化。
 */
template<>
struct Proto<DbusTag> {
    /** @brief 解析 DBUS 帧。 */
    static Status parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, DbusState *out)
    {
        return rc_dbus_parse(buf, len, tick_ms, out);
    }

    /** @brief 打包 DBUS 转发帧。 */
    static Status pack(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_len)
    {
        return rc_dbus_pack(payload, len, out, out_len);
    }
};

/**
 * @brief SBUS 协议特化。
 */
template<>
struct Proto<SbusTag> {
    /** @brief 解析参考 Sentinel 路径的 SBUS 帧。 */
    static Status parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, DbusState *out, SbusState *sbus)
    {
        return rc_sbus_parse(buf, len, tick_ms, out, sbus);
    }

    /** @brief 解析参考上遥控路径的 SBUS 帧。 */
    static Status parse_upper(const uint8_t *buf,
                              uint16_t len,
                              uint32_t tick_ms,
                              DbusState *out,
                              SbusState *sbus)
    {
        return rc_sbus_parse_upper(buf, len, tick_ms, out, sbus);
    }
};

/** @brief DBUS 协议包装。 */
namespace Dbus {

/** @brief 解析 DBUS 帧。 */
inline Status parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, DbusState *out)
{
    return Proto<DbusTag>::parse(buf, len, tick_ms, out);
}

/** @brief 打包 DBUS 转发帧。 */
inline Status pack(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_len)
{
    return Proto<DbusTag>::pack(payload, len, out, out_len);
}

} // namespace Dbus

/** @brief SBUS 协议包装。 */
namespace Sbus {

/** @brief 解析参考 Sentinel 路径的 SBUS 帧。 */
inline Status parse(const uint8_t *buf, uint16_t len, uint32_t tick_ms, DbusState *out, SbusState *sbus)
{
    return Proto<SbusTag>::parse(buf, len, tick_ms, out, sbus);
}

/** @brief 解析参考上遥控路径的 SBUS 帧。 */
inline Status parse_upper(const uint8_t *buf, uint16_t len, uint32_t tick_ms, DbusState *out, SbusState *sbus)
{
    return Proto<SbusTag>::parse_upper(buf, len, tick_ms, out, sbus);
}

} // namespace Sbus

} // namespace Device::Rc

#endif // PRINTK_DEVICE_RC_HPP
