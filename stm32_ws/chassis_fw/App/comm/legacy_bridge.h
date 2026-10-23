/**
  * @file       legacy_bridge.h
  * @brief      旧版 usb_bridge 协议兼容通道（本工程精简版）
  * @details    与 sentinel 上游的 legacy_bridge 使用同一套帧格式：
  *
  *             - 下行：解析 `AA 55 | id | seq | len | payload | xor` 的
  *               UnifiedCmd（0x10）与心跳（0x01）；
  *             - 上行：按旧格式周期回发 0x81 / 0x82 / 0x83 / 0x84。
  *
  *             本工程不含云台与 IMU，因此 0x81 的云台/姿态字段与 0x84 的
  *             IMU 字段固定为零；0x84 的 valid_mask 恒为 0，ROS 侧不会
  *             把这些零值当作有效测量。
  */

#ifndef LEGACY_BRIDGE_H
#define LEGACY_BRIDGE_H

#include <stdint.h>

#include "host_cmd.h"
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
 * @note  本工程不使用 uproto，未识别的字节直接丢弃。
 */
void legacy_bridge_feed_rx(const uint8_t *data, uint32_t len);

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
 * @brief 取最新的底盘速度指令
 * @param[out] out 上位机格式的速度指令（m/s、rad/s）
 * @retval 1 命令有效（CMD_VALID 置位）且未超过 100 ms；0 无可用命令
 * @details 命令过期或未置 CMD_VALID 时返回 0，调用方必须回到安全停车状态。
 */
uint8_t legacy_bridge_get_chassis_cmd(host_chassis_cmd_t *out);

/**
 * @brief 旧协议主机心跳帧计数（调试用）
 */
uint32_t legacy_bridge_rx_frames(void);

#ifdef __cplusplus
}
#endif

#endif /* LEGACY_BRIDGE_H */
