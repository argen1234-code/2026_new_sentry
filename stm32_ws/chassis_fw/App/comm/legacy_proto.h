/**
  * @file       legacy_proto.h
  * @brief      旧版 Jetson <-> STM32 USB 桥协议（纯逻辑，无 HAL 依赖）
  * @details    帧格式与 stm32_ws/legacy/Component/Communication/Src/communication_usb.h
  *             完全一致：
  *
  *                 AA 55 | id | seq | len | payload[len] | xor
  *                 xor = id ^ seq ^ len ^ payload[0] ^ ... ^ payload[len-1]
  *
  *             本文件只做收发编解码和命令映射，不访问任何外设，便于在 PC 上
  *             用 tools/host_tests/legacy_proto_test.c 做协议单元测试。
  */

#ifndef LEGACY_PROTO_H
#define LEGACY_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 帧常量（与 legacy 固件一致） ========================= */
#define LEGACY_SOF0                       0xAAu
#define LEGACY_SOF1                       0x55u
#define LEGACY_MAX_PAYLOAD                64u
#define LEGACY_MAX_FRAME                  (5u + LEGACY_MAX_PAYLOAD + 1u)

#define LEGACY_ID_HOST_HEARTBEAT          0x01u
#define LEGACY_ID_UNIFIED_CMD             0x10u
#define LEGACY_ID_MCU_STATE               0x81u
#define LEGACY_ID_MCU_HEARTBEAT           0x82u
#define LEGACY_ID_WHEEL_STATE             0x83u
#define LEGACY_ID_IMU_STATE               0x84u

#define LEGACY_UNIFIED_CMD_LEN            45u
#define LEGACY_MCU_STATE_LEN              40u
#define LEGACY_MCU_HEARTBEAT_LEN          8u
#define LEGACY_WHEEL_STATE_LEN            58u
#define LEGACY_IMU_STATE_LEN              58u

/* UnifiedCmd.chassis_status 位定义 */
#define LEGACY_CMD_VALID                  0x0001u
#define LEGACY_CMD_MOTION_ENABLE          0x0002u

/* 旧固件 ChassisMode 枚举（stm32_ws/legacy 的 chassis.h） */
#define LEGACY_CHASSIS_MODE_STOP          0
#define LEGACY_CHASSIS_MODE_NO_FOLLOW     2
#define LEGACY_CHASSIS_MODE_FOLLOW_YAW    3
#define LEGACY_CHASSIS_MODE_SPIN          5

/* 上游 sentinel 的底盘模式编号（comm_app 的 gimbal_on_chassis 注释） */
#define SENTINEL_CHASSIS_MODE_NO_FOLLOW   0
#define SENTINEL_CHASSIS_MODE_FOLLOW      1
#define SENTINEL_CHASSIS_MODE_SPIN        2
#define SENTINEL_CHASSIS_MODE_NO_MOVE     3

/**
 * @brief 一个完整的旧协议帧（已通过 xor 校验）
 */
typedef struct
{
    uint8_t msg_id;
    uint8_t seq;
    uint8_t payload_len;
    uint8_t payload[LEGACY_MAX_PAYLOAD];
} legacy_frame_t;

/**
 * @brief UnifiedCmd 解码结果
 */
typedef struct
{
    int32_t delta_yaw_udeg;
    int32_t delta_pitch_udeg;
    uint16_t gimbal_status;
    int32_t vx_mm_s;
    int32_t vy_mm_s;
    int32_t wz_mdeg_s;
    int32_t chassis_mode;
    uint16_t chassis_status;
    uint8_t fire_on;
    int32_t fire_mode;
    int32_t burst_count;
    uint64_t host_ts_ns;
} legacy_unified_cmd_t;

/**
 * @brief 增量帧解析器
 * @details buf 只需容纳“尚未判定”的字节：最长候选帧 51 B。
 */
typedef struct
{
    uint8_t buf[LEGACY_MAX_FRAME];
    uint16_t len;
    uint32_t rx_frames;
    uint32_t errors;
} legacy_parser_t;

/** @brief 非旧协议字节的透传回调（交给 uproto 解析器） */
typedef void (*legacy_pass_fn)(uint8_t byte, void *user);

/** @brief 完整旧协议帧回调 */
typedef void (*legacy_frame_fn)(const legacy_frame_t *frame, void *user);

void legacy_parser_init(legacy_parser_t *parser);

/**
 * @brief 增量喂入接收字节
 * @param[in,out] parser   解析器
 * @param[in]     data     新到字节
 * @param[in]     len      字节数
 * @param[in]     pass     非旧协议字节透传回调，可为 NULL
 * @param[in]     on_frame 完整帧回调，可为 NULL
 * @param[in]     user     回调上下文
 * @details      识别 0x01/0x10 两种主机帧；其余字节原样交给 pass，
 *               因此 uproto 数据流可以与旧协议共存于同一条 USB CDC 上。
 */
void legacy_parser_feed(legacy_parser_t *parser,
                        const uint8_t *data,
                        uint32_t len,
                        legacy_pass_fn pass,
                        legacy_frame_fn on_frame,
                        void *user);

/**
 * @brief 组帧
 * @param[out] out          输出缓冲，至少 LEGACY_MAX_FRAME 字节
 * @param[in]  msg_id       消息 ID
 * @param[in]  seq          序列号
 * @param[in]  payload      负载，可为 NULL
 * @param[in]  payload_len  负载长度，<= LEGACY_MAX_PAYLOAD
 * @retval 帧总长度；参数非法返回 0
 */
uint16_t legacy_encode_frame(uint8_t *out,
                             uint8_t msg_id,
                             uint8_t seq,
                             const uint8_t *payload,
                             uint8_t payload_len);

/**
 * @brief 解码 UnifiedCmd
 * @retval 1 成功；0 帧 ID/长度不符
 */
uint8_t legacy_decode_unified_cmd(const legacy_frame_t *frame,
                                  legacy_unified_cmd_t *out);

/**
 * @brief 把旧协议底盘命令翻译成上游 sentinel 的底盘模式与速度
 * @details - chassis_status 必须同时带 CMD_VALID 和 CMD_MOTION_ENABLE，
 *            否则视为安全停车（速度清零、模式切到 no-move）；
 *          - 旧模式 2 (NoFollow) -> 上游 0，3 (FollowGimbalYaw) -> 1，
 *            5 (Spin) -> 2，Stop/未知模式 -> 3 (no-move)。
 * @param[in]  in           旧协议命令
 * @param[out] vx_mm_s      上游命令 vx
 * @param[out] vy_mm_s      上游命令 vy
 * @param[out] wz_mdeg_s    上游命令 wz
 * @param[out] upstream_mode 上游模式编号
 */
void legacy_map_chassis_cmd(const legacy_unified_cmd_t *in,
                            int32_t *vx_mm_s,
                            int32_t *vy_mm_s,
                            int32_t *wz_mdeg_s,
                            int32_t *upstream_mode);

/**
 * @brief 把小端字节流读成定长整数/浮点（不依赖对齐）
 */
uint16_t legacy_read_u16(const uint8_t *p);
uint32_t legacy_read_u32(const uint8_t *p);
int32_t legacy_read_i32(const uint8_t *p);
uint64_t legacy_read_u64(const uint8_t *p);
float legacy_read_f32(const uint8_t *p);

/**
 * @brief 写小端整数/浮点
 */
void legacy_write_u16(uint8_t *p, uint16_t v);
void legacy_write_u32(uint8_t *p, uint32_t v);
void legacy_write_i32(uint8_t *p, int32_t v);
void legacy_write_u64(uint8_t *p, uint64_t v);
void legacy_write_f32(uint8_t *p, float v);

#ifdef __cplusplus
}
#endif

#endif /* LEGACY_PROTO_H */
