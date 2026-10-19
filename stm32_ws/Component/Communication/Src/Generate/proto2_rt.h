/**
 * @file
 * @brief Proto2 RT 自动生成文件，请勿手动修改。
 *
 * @see communication_host_schema.hpp
 */

#ifndef GENERATED_PROTO2_RT_H
#define GENERATED_PROTO2_RT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup proto2_rt_generated 自动生成的紧凑实时协议核
 * @brief 固定帧、固定 payload 编解码、流式解析和链路统计。
 *
 * 本核只描述 wire codec 和 parser，不绑定物理层、调度周期或任务模型。
 * 上层按自己的控制环节拍调用 encode/feed，并自行决定传输和超时策略。
 * @{
 */

#define PROTO2_RT_MAGIC_0 ((uint8_t)0xA5u)
#define PROTO2_RT_MAGIC_1 ((uint8_t)0x5Au)
#define PROTO2_RT_HEADER_SIZE 5u
#define PROTO2_RT_CRC_SIZE 2u
#define PROTO2_RT_OVERHEAD (PROTO2_RT_HEADER_SIZE + PROTO2_RT_CRC_SIZE)
#define PROTO2_RT_MAX_PAYLOAD 33u
#define PROTO2_RT_PAYLOAD_STORAGE 33u
#define PROTO2_RT_MAX_FRAME_BYTES (PROTO2_RT_OVERHEAD + PROTO2_RT_MAX_PAYLOAD)

/** @brief 实时消息方向，仅作为 schema 事实，不驱动传输层。 */
typedef enum {
    PROTO2_RT_DIRECTION_HOST_TO_DEVICE = 0,
    PROTO2_RT_DIRECTION_DEVICE_TO_HOST = 1,
    PROTO2_RT_DIRECTION_BIDIRECTIONAL = 2
} proto2_rt_direction_t;

/** @brief endpoint 角色，用于方向校验。 */
typedef enum {
    PROTO2_RT_ROLE_DEVICE = 0,
    PROTO2_RT_ROLE_HOST = 1,
    PROTO2_RT_ROLE_ANY = 2
} proto2_rt_role_t;

/** @brief 实时核返回状态。 */
typedef enum {
    PROTO2_RT_STATUS_OK = 0,
    PROTO2_RT_STATUS_NULL = 1,
    PROTO2_RT_STATUS_SMALL = 2,
    PROTO2_RT_STATUS_LENGTH = 3,
    PROTO2_RT_STATUS_ID = 4,
    PROTO2_RT_STATUS_CRC = 5,
    PROTO2_RT_STATUS_DIRECTION = 6,
    PROTO2_RT_STATUS_DECODE = 7
} proto2_rt_status_t;

#define PROTO2_RT_MSG_ID_HOST_AIM ((uint8_t)0x01u)
#define PROTO2_RT_WIRE_SIZE_HOST_AIM 14u
#define PROTO2_RT_FRAME_SIZE_HOST_AIM 21u
#define PROTO2_RT_DIRECTION_HOST_AIM PROTO2_RT_DIRECTION_HOST_TO_DEVICE

/** @brief HostAim 的固定 payload 结构。 */
typedef struct {
    float yaw_rad;
    float pitch_rad;
    uint8_t valid_mask;
    uint8_t enable;
    uint32_t host_time_us;
} proto2_rt_msg_host_aim_t;

#define PROTO2_RT_MSG_ID_GIMBAL_STATE ((uint8_t)0x02u)
#define PROTO2_RT_WIRE_SIZE_GIMBAL_STATE 21u
#define PROTO2_RT_FRAME_SIZE_GIMBAL_STATE 28u
#define PROTO2_RT_DIRECTION_GIMBAL_STATE PROTO2_RT_DIRECTION_DEVICE_TO_HOST

/** @brief GimbalState 的固定 payload 结构。 */
typedef struct {
    float yaw_rad;
    float pitch_rad;
    float yaw_rate_rad_s;
    float pitch_rate_rad_s;
    uint8_t status;
    uint32_t time_us;
} proto2_rt_msg_gimbal_state_t;

#define PROTO2_RT_MSG_ID_RT_PROBE ((uint8_t)0x03u)
#define PROTO2_RT_WIRE_SIZE_RT_PROBE 6u
#define PROTO2_RT_FRAME_SIZE_RT_PROBE 13u
#define PROTO2_RT_DIRECTION_RT_PROBE PROTO2_RT_DIRECTION_BIDIRECTIONAL

/** @brief RtProbe 的固定 payload 结构。 */
typedef struct {
    uint16_t token;
    uint32_t time_us;
} proto2_rt_msg_rt_probe_t;

#define PROTO2_RT_MSG_ID_RT_PROBE_ACK ((uint8_t)0x04u)
#define PROTO2_RT_WIRE_SIZE_RT_PROBE_ACK 10u
#define PROTO2_RT_FRAME_SIZE_RT_PROBE_ACK 17u
#define PROTO2_RT_DIRECTION_RT_PROBE_ACK PROTO2_RT_DIRECTION_BIDIRECTIONAL

/** @brief RtProbeAck 的固定 payload 结构。 */
typedef struct {
    uint16_t token;
    uint32_t request_time_us;
    uint32_t reply_time_us;
} proto2_rt_msg_rt_probe_ack_t;

#define PROTO2_RT_MSG_ID_COMMUNICATION_BOARD_STATE ((uint8_t)0x05u)
#define PROTO2_RT_WIRE_SIZE_COMMUNICATION_BOARD_STATE 33u
#define PROTO2_RT_FRAME_SIZE_COMMUNICATION_BOARD_STATE 40u
#define PROTO2_RT_DIRECTION_COMMUNICATION_BOARD_STATE PROTO2_RT_DIRECTION_BIDIRECTIONAL

/** @brief CommunicationBoardState 的固定 payload 结构。 */
typedef struct {
    float yaw_motor_rad;
    float ins_yaw_rad;
    float ins_pitch_rad;
    float ins_roll_rad;
    uint8_t gimbal_mode;
    uint8_t shoot_mode;
    int16_t rc_ch0;
    int16_t rc_ch1;
    int16_t rc_ch2;
    int16_t rc_ch3;
    uint8_t rc_sw0;
    uint8_t rc_sw1;
    uint8_t status;
    uint32_t time_us;
} proto2_rt_msg_communication_board_state_t;

#define PROTO2_RT_MSG_ID_COMMUNICATION_CHASSIS_STATE ((uint8_t)0x06u)
#define PROTO2_RT_WIRE_SIZE_COMMUNICATION_CHASSIS_STATE 19u
#define PROTO2_RT_FRAME_SIZE_COMMUNICATION_CHASSIS_STATE 26u
#define PROTO2_RT_DIRECTION_COMMUNICATION_CHASSIS_STATE PROTO2_RT_DIRECTION_BIDIRECTIONAL

/** @brief CommunicationChassisState 的固定 payload 结构。 */
typedef struct {
    float yaw_rad;
    float pitch_rad;
    float roll_rad;
    uint16_t robot_id;
    uint8_t status;
    uint32_t time_us;
} proto2_rt_msg_communication_chassis_state_t;

/** @brief 已通过 CRC 和长度检查的实时帧视图。 */
typedef struct {
    uint8_t msg_id;       /**< 消息 ID。 */
    uint16_t seq;         /**< 帧序号，由上层递增填入。 */
    const uint8_t *payload; /**< payload 视图，生命周期到下一次 feed 调用前。 */
    uint8_t payload_len;  /**< payload 字节数。 */
} proto2_rt_frame_t;

/** @brief 实时帧回调函数。 */
typedef void (*proto2_rt_frame_fn)(void *user, const proto2_rt_frame_t *frame);

/** @brief 实时核解析上下文和统计信息。 */
typedef struct {
    uint32_t rx_frames;     /**< 已接收合法帧数量。 */
    uint32_t crc_errors;    /**< CRC 错误数量。 */
    uint32_t id_errors;     /**< 未知 msg_id 数量。 */
    uint32_t seq_errors;    /**< 序号不连续数量。 */
    uint8_t state;          /**< 内部流式解析状态。 */
    uint8_t msg_id;         /**< 当前正在接收的 msg_id。 */
    uint8_t payload_len;    /**< 当前帧 payload 长度。 */
    uint8_t payload_pos;    /**< 当前 payload 写入位置。 */
    uint16_t seq;           /**< 当前帧序号。 */
    uint16_t rx_crc;        /**< 当前帧接收到的 CRC。 */
    uint16_t crc_calc;      /**< 当前帧增量计算 CRC。 */
    proto2_rt_role_t role;  /**< 当前 endpoint 角色，零初始化为 DEVICE。 */
    uint8_t payload[PROTO2_RT_PAYLOAD_STORAGE]; /**< 当前帧 payload 暂存。 */
    uint8_t have_host_aim; /**< HostAim 是否已有上一帧序号。 */
    uint16_t last_seq_host_aim; /**< HostAim 上一帧序号。 */
    uint8_t have_gimbal_state; /**< GimbalState 是否已有上一帧序号。 */
    uint16_t last_seq_gimbal_state; /**< GimbalState 上一帧序号。 */
    uint8_t have_rt_probe; /**< RtProbe 是否已有上一帧序号。 */
    uint16_t last_seq_rt_probe; /**< RtProbe 上一帧序号。 */
    uint8_t have_rt_probe_ack; /**< RtProbeAck 是否已有上一帧序号。 */
    uint16_t last_seq_rt_probe_ack; /**< RtProbeAck 上一帧序号。 */
    uint8_t have_communication_board_state; /**< CommunicationBoardState 是否已有上一帧序号。 */
    uint16_t last_seq_communication_board_state; /**< CommunicationBoardState 上一帧序号。 */
    uint8_t have_communication_chassis_state; /**< CommunicationChassisState 是否已有上一帧序号。 */
    uint16_t last_seq_communication_chassis_state; /**< CommunicationChassisState 上一帧序号。 */
} proto2_rt_t;

/**
 * @brief 初始化实时核上下文。
 * @param rt 实时核上下文。
 */
void proto2_rt_init(proto2_rt_t *rt);

/**
 * @brief 查询 msg_id 对应的固定 payload 长度。
 * @param msg_id 消息 ID。
 * @return uint16_t payload 长度；未知 ID 返回 0xFFFF。
 */
uint16_t proto2_rt_payload_len(uint8_t msg_id);

/**
 * @brief 查询指定角色是否允许发送 msg_id。
 * @param role endpoint 角色。
 * @param msg_id 消息 ID。
 * @return uint8_t 允许返回 1，否则返回 0。
 */
uint8_t proto2_rt_can_tx(proto2_rt_role_t role, uint8_t msg_id);

/**
 * @brief 查询指定角色是否允许接收 msg_id。
 * @param role endpoint 角色。
 * @param msg_id 消息 ID。
 * @return uint8_t 允许返回 1，否则返回 0。
 */
uint8_t proto2_rt_can_rx(proto2_rt_role_t role, uint8_t msg_id);

/**
 * @brief 喂入单字节并在完整合法帧到达时输出 frame view。
 * @param rt 实时核上下文。
 * @param byte 输入字节。
 * @param frame 输出 frame view；没有完整帧时 payload 会被置为 NULL。
 * @return proto2_rt_status_t 本字节解析结果。
 */
proto2_rt_status_t proto2_rt_feed_byte(proto2_rt_t *rt, uint8_t byte, proto2_rt_frame_t *frame);

/**
 * @brief 流式喂入字节并回调合法帧。
 * @param rt 实时核上下文。
 * @param data 输入字节流。
 * @param len 输入字节数。
 * @param on_frame 合法帧回调；可为 NULL。
 * @param user 用户上下文。
 * @return proto2_rt_status_t 本次 feed 遇到的最后一个非 OK 错误；全程合法返回 OK。
 */
proto2_rt_status_t proto2_rt_feed(proto2_rt_t *rt,
                                  const uint8_t *data,
                                  uint16_t len,
                                  proto2_rt_frame_fn on_frame,
                                  void *user);

/**
 * @brief 编码 HostAim 实时帧。
 * @param out 输出缓冲区。
 * @param out_cap 输出缓冲区容量。
 * @param seq 帧序号。
 * @param msg 待编码 payload。
 * @return uint16_t 实际帧长；参数非法或空间不足返回 0。
 */
uint16_t proto2_rt_encode_host_aim(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_host_aim_t *msg);

/**
 * @brief 解码 HostAim payload。
 * @param out 解码输出。
 * @param payload 输入 payload。
 * @param payload_len 输入 payload 字节数。
 * @return proto2_rt_status_t 解码结果。
 */
proto2_rt_status_t proto2_rt_decode_host_aim(proto2_rt_msg_host_aim_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len);

/**
 * @brief 编码 GimbalState 实时帧。
 * @param out 输出缓冲区。
 * @param out_cap 输出缓冲区容量。
 * @param seq 帧序号。
 * @param msg 待编码 payload。
 * @return uint16_t 实际帧长；参数非法或空间不足返回 0。
 */
uint16_t proto2_rt_encode_gimbal_state(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_gimbal_state_t *msg);

/**
 * @brief 解码 GimbalState payload。
 * @param out 解码输出。
 * @param payload 输入 payload。
 * @param payload_len 输入 payload 字节数。
 * @return proto2_rt_status_t 解码结果。
 */
proto2_rt_status_t proto2_rt_decode_gimbal_state(proto2_rt_msg_gimbal_state_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len);

/**
 * @brief 编码 RtProbe 实时帧。
 * @param out 输出缓冲区。
 * @param out_cap 输出缓冲区容量。
 * @param seq 帧序号。
 * @param msg 待编码 payload。
 * @return uint16_t 实际帧长；参数非法或空间不足返回 0。
 */
uint16_t proto2_rt_encode_rt_probe(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_rt_probe_t *msg);

/**
 * @brief 解码 RtProbe payload。
 * @param out 解码输出。
 * @param payload 输入 payload。
 * @param payload_len 输入 payload 字节数。
 * @return proto2_rt_status_t 解码结果。
 */
proto2_rt_status_t proto2_rt_decode_rt_probe(proto2_rt_msg_rt_probe_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len);

/**
 * @brief 编码 RtProbeAck 实时帧。
 * @param out 输出缓冲区。
 * @param out_cap 输出缓冲区容量。
 * @param seq 帧序号。
 * @param msg 待编码 payload。
 * @return uint16_t 实际帧长；参数非法或空间不足返回 0。
 */
uint16_t proto2_rt_encode_rt_probe_ack(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_rt_probe_ack_t *msg);

/**
 * @brief 解码 RtProbeAck payload。
 * @param out 解码输出。
 * @param payload 输入 payload。
 * @param payload_len 输入 payload 字节数。
 * @return proto2_rt_status_t 解码结果。
 */
proto2_rt_status_t proto2_rt_decode_rt_probe_ack(proto2_rt_msg_rt_probe_ack_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len);

/**
 * @brief 编码 CommunicationBoardState 实时帧。
 * @param out 输出缓冲区。
 * @param out_cap 输出缓冲区容量。
 * @param seq 帧序号。
 * @param msg 待编码 payload。
 * @return uint16_t 实际帧长；参数非法或空间不足返回 0。
 */
uint16_t proto2_rt_encode_communication_board_state(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_communication_board_state_t *msg);

/**
 * @brief 解码 CommunicationBoardState payload。
 * @param out 解码输出。
 * @param payload 输入 payload。
 * @param payload_len 输入 payload 字节数。
 * @return proto2_rt_status_t 解码结果。
 */
proto2_rt_status_t proto2_rt_decode_communication_board_state(proto2_rt_msg_communication_board_state_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len);

/**
 * @brief 编码 CommunicationChassisState 实时帧。
 * @param out 输出缓冲区。
 * @param out_cap 输出缓冲区容量。
 * @param seq 帧序号。
 * @param msg 待编码 payload。
 * @return uint16_t 实际帧长；参数非法或空间不足返回 0。
 */
uint16_t proto2_rt_encode_communication_chassis_state(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_communication_chassis_state_t *msg);

/**
 * @brief 解码 CommunicationChassisState payload。
 * @param out 解码输出。
 * @param payload 输入 payload。
 * @param payload_len 输入 payload 字节数。
 * @return proto2_rt_status_t 解码结果。
 */
proto2_rt_status_t proto2_rt_decode_communication_chassis_state(proto2_rt_msg_communication_chassis_state_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* GENERATED_PROTO2_RT_H */
