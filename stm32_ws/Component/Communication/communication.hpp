/**
 ******************************************************************************
 * @file    communication.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   通讯组件 C++ 薄包装。
 *
 * @note    本文件只做类型别名和 inline 转发，运行时事实来源位于 C 接口。
 * @note    不引入虚函数、RTTI 或动态分派。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_COMMUNICATION_HPP
#define PRINTK_COMPONENT_COMMUNICATION_HPP

#include "communication.h"
#include "Src/Generate/proto2_rt.hpp"

#include <cstddef>

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只包装组件 C API 和类型别名，不写运行时控制逻辑。
 * ========================================================================== */
namespace Component::Communication {

/** @brief 通讯组件状态码。 */
using Status = CommunicationStatus;

/** @brief 通讯组件配置。 */
using Cfg = CommunicationCfg;

/** @brief host 通讯配置。 */
using HostCfg = CommunicationHostCfg;

/** @brief host 通讯诊断快照。 */
using HostDiag = CommunicationHostDiag;

/** @brief 板间通讯配置。 */
using BoardCfg = CommunicationBoardCfg;

/** @brief 板间通讯诊断。 */
using BoardDiag = CommunicationBoardDiag;

/** @brief 云台状态上报载荷。 */
using GimbalState = CommunicationGimbalState;

/** @brief 云台板间状态。 */
using BoardState = CommunicationBoardState;

/** @brief 底盘板间状态。 */
using ChassisState = CommunicationChassisState;

/** @brief RTT probe 数据。 */
using Probe = CommunicationProbe;

/** @brief RTT probe ack 数据。 */
using ProbeAck = CommunicationProbeAck;

/** @brief 时间同步状态。 */
using TimeState = CommunicationTimeState;

/** @brief host 原始自瞄误差。 */
using AimState = CommunicationAimState;

/** @brief VOFA 自定义发送函数。 */
using VofaTxFn = CommVofaTxFn;

/** @brief Debug VOFA 桥诊断信息。*/
using DebugDiag = CommunicationDebugDiag;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 填充默认配置。 */
inline Status default_cfg(Cfg &cfg)
{
    return communication_default_cfg(&cfg);
}

/** @brief 初始化通讯组件。 */
inline Status init(const Cfg &cfg)
{
    return communication_init(&cfg);
}

/** @brief 处理 host UART DMA idle 接收事件。 */
inline Status rx_event(uint16_t size)
{
    return communication_rx_event(size);
}

/** @brief 处理板间 UART DMA idle 接收事件。 */
inline Status rx_board_event(uint16_t size)
{
    return communication_rx_board_event(size);
}

/** @brief 读取最新原始自瞄误差。 */
inline Status get_aim(AimState &out)
{
    return communication_get_aim(&out);
}

/** @brief 读取最新 probe。 */
inline Status get_probe(Probe &out)
{
    return communication_get_probe(&out);
}

/** @brief 读取最新 probe ack。 */
inline Status get_probe_ack(ProbeAck &out)
{
    return communication_get_probe_ack(&out);
}

/** @brief 发送云台状态。 */
inline Status send_state(const GimbalState &state)
{
    return communication_send_state(&state);
}

/** @brief 发送 RTT probe。 */
inline Status send_probe(uint16_t token, uint32_t time_us)
{
    return communication_send_probe(token, time_us);
}

/** @brief 发送 RTT probe ack。 */
inline Status send_probe_ack(uint16_t token, uint32_t request_time_us, uint32_t reply_time_us)
{
    return communication_send_probe_ack(token, request_time_us, reply_time_us);
}

/** @brief 读取云台板间状态。 */
inline Status get_board_state(BoardState &out)
{
    return communication_get_board_state(&out);
}

/** @brief 读取底盘板间状态。 */
inline Status get_chassis_state(ChassisState &out)
{
    return communication_get_chassis_state(&out);
}

/** @brief 发送云台板间状态。 */
inline Status send_board_state(const BoardState &state)
{
    return communication_send_board_state(&state);
}

/** @brief 发送底盘板间状态。 */
inline Status send_chassis_state(const ChassisState &state)
{
    return communication_send_chassis_state(&state);
}

/** @brief 读取时间同步状态。 */
inline Status get_time(TimeState &out)
{
    return communication_get_time(&out);
}

/** @brief 将本地时间换算为远端时间估计。 */
inline Status remote_now(uint32_t local_time_us, uint32_t &remote_time_us)
{
    return communication_remote_now(local_time_us, &remote_time_us);
}

namespace Host {

/** @brief 填充 host 默认配置。 */
inline Status default_cfg(HostCfg &cfg)
{
    return communication_host_default_cfg(&cfg);
}

/** @brief 初始化 host 485 通讯。 */
inline Status init(const HostCfg &cfg)
{
    return communication_host_init(&cfg);
}

/** @brief 直接喂入 host 协议字节流。 */
inline Status feed(const uint8_t *data, uint16_t length)
{
    return communication_host_feed(data, length);
}

/** @brief 读取最新原始自瞄误差。 */
inline Status get_aim(AimState &out)
{
    return communication_host_get_aim(&out);
}

/** @brief 读取最新 probe。 */
inline Status get_probe(Probe &out)
{
    return communication_host_get_probe(&out);
}

/** @brief 读取最新 probe ack。 */
inline Status get_ack(ProbeAck &out)
{
    return communication_host_get_ack(&out);
}

/** @brief 读取 host 通讯诊断。 */
inline Status diag(HostDiag &out)
{
    return communication_host_diag(&out);
}

} // namespace Host

namespace Board {

/** @brief 填充板间通讯默认配置。 */
inline Status default_cfg(BoardCfg &cfg)
{
    return communication_board_default_cfg(&cfg);
}

/** @brief 初始化板间通讯。 */
inline Status init(const BoardCfg &cfg)
{
    return communication_board_init(&cfg);
}

/** @brief 直接喂入板间协议字节流。 */
inline Status feed(const uint8_t *data, uint16_t length)
{
    return communication_board_feed(data, length);
}

/** @brief 读取板间通讯诊断。 */
inline Status diag(BoardDiag &out)
{
    return communication_board_diag(&out);
}

} // namespace Board

namespace Vofa {

/** @brief 初始化 VOFA 到 USB CDC ACM。 */
inline Status init_cdc()
{
    return comm_vofa_init_cdc();
}

/** @brief 初始化 VOFA 到自定义发送端。 */
inline Status custom(VofaTxFn tx_fn, void *user)
{
    return comm_vofa_custom(tx_fn, user);
}

/** @brief 判断 VOFA 是否就绪。 */
inline bool ready()
{
    return comm_vofa_ready();
}

/** @brief 将 Debug LOG 输出绑定到当前 VOFA 发送端。 */
inline Status bind_log()
{
    return comm_vofa_bind_log();
}

/** @brief 发送 VOFA 原始数据。 */
inline Status raw(const void *data, uint16_t length)
{
    return comm_vofa_raw(data, length);
}

/** @brief 发送 FireWater 文本。 */
template<typename... Args>
inline Status fire(const char *format, Args... args)
{
    return comm_vofa_fire(format, args...);
}

/** @brief 发送 JustFloat 数据。 */
inline Status floats(const float *data, uint16_t count)
{
    return comm_vofa_float(data, count);
}

/** @brief 发送定长数组 JustFloat 数据。 */
template<std::size_t N>
inline Status floats(const float (&data)[N])
{
    return comm_vofa_float(data, static_cast<uint16_t>(N));
}

} // namespace Vofa

namespace Debug {

/** @brief 将 Debug 探针快照输出为 VOFA JustFloat。 */
inline Status flush()
{
    return communication_debug_flush();
}

/** @brief 输出 profiler 快照。 */
inline Status profiler_flush()
{
    return communication_debug_profiler_flush();
}

/** @brief 读取 Debug VOFA 桥诊断信息。 */
inline Status diag(DebugDiag &out)
{
    return communication_debug_diag(&out);
}

} // namespace Debug

} // namespace Component::Communication

#endif // PRINTK_COMPONENT_COMMUNICATION_HPP
