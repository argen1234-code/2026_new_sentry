/**
 ******************************************************************************
 * @file    referee.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   裁判系统设备 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 薄包装，运行时实现全部位于 Src/referee*.c。
 * @note    C 文件应包含 Src/referee.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_REFEREE_HPP
#define PRINTK_DEVICE_REFEREE_HPP

#include "Src/referee.h"
#include "Src/referee_protocol.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装设备层 C API，便于 C++ 调用侧保持零成本访问。
 * ========================================================================== */
namespace Device::Referee {

/** @brief 裁判系统状态码。 */
using Status = RefereeStatus;

/** @brief 裁判系统命令 ID。 */
using CmdId = RefereeCmdId;

/** @brief 裁判系统机器人 ID。 */
using RobotId = RefereeRobotId;

/** @brief 裁判系统帧头。 */
using Header = RefereeHeader;

/** @brief 裁判系统完整原始帧。 */
using Frame = RefereeFrame;

/** @brief 裁判系统状态缓存。 */
using State = RefereeState;

/** @brief 裁判系统诊断信息。 */
using Diag = RefereeDiag;

/** @brief 裁判系统解包上下文。 */
using Unpack = RefereeUnpack;

/** @brief 裁判系统配置。 */
using Cfg = RefereeCfg;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 裁判系统最大帧长。 */
inline constexpr uint16_t FrameMaxSize = REFEREE_FRAME_MAX_SIZE;

/** @brief 裁判系统接收缓存长度。 */
inline constexpr uint16_t RxBufferSize = REFEREE_RX_BUF_SIZE;

/** @brief 初始化裁判系统设备。 */
inline Status init()
{
    return referee_init();
}

/** @brief 处理 UART idle 接收事件。 */
inline void rx_event(uint16_t size)
{
    referee_rx_event(size);
}

/** @brief 向裁判系统状态机喂入字节流。 */
inline Status feed(const uint8_t *data, uint16_t len, uint32_t tick_ms)
{
    return referee_feed(data, len, tick_ms);
}

/** @brief 获取状态快照。 */
inline Status get(State *out)
{
    return referee_get(out);
}

/** @brief 获取诊断快照。 */
inline Status diag(Diag *out)
{
    return referee_diag(out);
}

/** @brief 判断裁判系统是否在线。 */
inline bool online(uint32_t timeout_ms, uint32_t now_ms)
{
    return referee_online(timeout_ms, now_ms) != 0U;
}

/** @brief 发送普通裁判系统帧。 */
inline Status send(uint16_t cmd_id, const uint8_t *payload, uint16_t payload_len)
{
    return referee_send(cmd_id, payload, payload_len);
}

/** @brief 发送机器人交互帧。 */
inline Status send_interact(uint16_t sender_id,
                            uint16_t receiver_id,
                            uint16_t data_cmd_id,
                            const uint8_t *data,
                            uint16_t data_len)
{
    return referee_send_interact(sender_id, receiver_id, data_cmd_id, data, data_len);
}

/** @brief 获取本机器人 ID。 */
inline uint8_t robot_id()
{
    return referee_robot_id();
}

/** @brief 获取最近一次合法帧接收 tick。 */
inline uint32_t last_rx_tick()
{
    return referee_last_rx_tick();
}

/** @brief 获取底盘功率预算。 */
inline Status chassis_budget(float *limit, float *buffer)
{
    return referee_chassis_budget(limit, buffer);
}

/** @brief 获取发射热量。 */
inline Status shoot_heat(uint8_t shooter, uint16_t *heat_limit, uint16_t *heat)
{
    return referee_shoot_heat(shooter, heat_limit, heat);
}

/** @brief 复位独立解包上下文。 */
inline void unpack_reset(Unpack *unpack)
{
    referee_unpack_reset(unpack);
}

/** @brief 输入一个字节到独立解包上下文。 */
inline Status unpack_byte(Unpack *unpack, uint8_t byte, Frame *frame)
{
    return referee_unpack_byte(unpack, byte, frame);
}

/** @brief 打包普通裁判系统帧。 */
inline Status pack_frame(uint16_t cmd_id,
                         const uint8_t *payload,
                         uint16_t payload_len,
                         uint8_t seq,
                         uint8_t *out,
                         uint16_t out_size,
                         uint16_t *out_len)
{
    return referee_pack_frame(cmd_id, payload, payload_len, seq, out, out_size, out_len);
}

/** @brief 打包机器人交互帧。 */
inline Status pack_interact(uint16_t sender_id,
                            uint16_t receiver_id,
                            uint16_t data_cmd_id,
                            const uint8_t *data,
                            uint16_t data_len,
                            uint8_t seq,
                            uint8_t *out,
                            uint16_t out_size,
                            uint16_t *out_len)
{
    return referee_pack_interact(sender_id, receiver_id, data_cmd_id, data, data_len, seq, out, out_size, out_len);
}

} // namespace Device::Referee

#endif // PRINTK_DEVICE_REFEREE_HPP
