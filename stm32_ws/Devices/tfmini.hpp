/**
 ******************************************************************************
 * @file    tfmini.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   TFmini 测距设备 C++ 包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/tfmini.c。
 * @note    C 文件应包含 Src/tfmini.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_TFMINI_HPP
#define PRINTK_DEVICE_TFMINI_HPP

#include "Src/tfmini.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Tfmini {

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief TFmini UART 默认发送超时时间。 */
inline constexpr uint32_t UartTxTimeout = TFMINI_UART_TX_TIMEOUT_MS;

/** @brief TFmini 设备状态。 */
using Status = tfmini_status_t;

/** @brief TFmini 测距数据。 */
using Measurement = tfmini_meas_t;

/** @brief 初始化 TFmini。 */
inline Status init()
{
    return tfmini_init();
}

/** @brief 判断 TFmini 是否已经初始化。 */
inline bool ready()
{
    return tfmini_ready();
}

/** @brief 判断 TFmini 是否在线。 */
inline bool online(uint32_t timeout_ms)
{
    return tfmini_online(timeout_ms);
}

/** @brief 读取最新 TFmini 数据。 */
inline bool read(Measurement *out, uint32_t *io_seq)
{
    return tfmini_read(out, io_seq);
}

/** @brief 获取最近一次 TFmini 数据。 */
inline bool peek(Measurement *out)
{
    return tfmini_peek(out);
}

/** @brief 重置 TFmini 帧解析状态。 */
inline void reset()
{
    tfmini_reset();
}

/** @brief 发送 TFmini 原始字节。 */
inline Status raw(const uint8_t *data, uint16_t length)
{
    return tfmini_raw(data, length);
}

} // namespace Device::Tfmini

#endif // PRINTK_DEVICE_TFMINI_HPP
