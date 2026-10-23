/**
  * @file       legacy_bridge.h
  * @brief      上游固件对旧版 usb_bridge 协议的兼容通道
  * @details    在不改动 ROS 侧 sentry_hardware/usb_bridge 的前提下，让
  *             sentinel 固件能够被现有 Jetson 链路直接驱动：
  *
  *             - 下行：解析 `AA 55 | id | seq | len | payload | xor` 的
  *               UnifiedCmd（0x10）与心跳（0x01），映射到上游
  *               gimbal_chassis_cmd_t，并复用 comm_app 的遥控注入链路；
  *             - 上行：按旧格式周期回发 0x81 / 0x82 / 0x83 / 0x84。
  *
  *             与 uproto 共存：未被识别的字节原样交给 uproto 解析器；
  *             只有检测到旧协议主机在线时，才暂停 uproto 发送，避免两种
  *             帧在同一 CDC 流中交错。
  */

#ifndef LEGACY_BRIDGE_H
#define LEGACY_BRIDGE_H

#include <stdint.h>

#include "../channel/gimbal/gimbal_channel.h"
#include "legacy_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 旧固件的命令新鲜度与主机在线超时 */
#ifndef LEGACY_BRIDGE_CMD_TIMEOUT_MS
#define LEGACY_BRIDGE_CMD_TIMEOUT_MS   100u
#endif

#ifndef LEGACY_BRIDGE_LINK_TIMEOUT_MS
#define LEGACY_BRIDGE_LINK_TIMEOUT_MS  500u
#endif

/* 状态回传周期，与旧固件保持一致（20 ms / 100 ms） */
#ifndef LEGACY_BRIDGE_STATE_PERIOD_MS
#define LEGACY_BRIDGE_STATE_PERIOD_MS  20u
#endif

#ifndef LEGACY_BRIDGE_HEARTBEAT_PERIOD_MS
#define LEGACY_BRIDGE_HEARTBEAT_PERIOD_MS 100u
#endif

/**
 * @brief 初始化兼容通道（清零解析器与缓存）
 */
void legacy_bridge_init(void);

/**
 * @brief 把 USB CDC 收到的字节送入兼容通道
 * @param[in] data 收到的字节
 * @param[in] len  字节数
 * @param[in] pass 非旧协议字节的透传回调（通常转给 uproto），可为 NULL
 * @param[in] user 透传上下文
 */
void legacy_bridge_feed_rx(const uint8_t *data,
                           uint32_t len,
                           legacy_pass_fn pass,
                           void *user);

/**
 * @brief 周期任务：主机在线时回发旧格式状态帧
 * @details 需在通信任务循环中按 <= 20 ms 周期调用。
 */
void legacy_bridge_tick(void);

/**
 * @brief 旧协议主机是否在线（最近 500 ms 内收到过合法帧）
 */
uint8_t legacy_bridge_active(void);

/**
 * @brief 取最新的底盘命令
 * @param[out] out 上位机格式的底盘命令
 * @retval 1 命令有效且未超过 100 ms；0 无可用命令
 * @details 返回的 vx/vy/wz/mode 已经翻译成上游语义；命令过期后返回 0，
 *          调用方必须回到安全停车状态。
 */
uint8_t legacy_bridge_get_chassis_cmd(gimbal_chassis_cmd_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LEGACY_BRIDGE_H */
