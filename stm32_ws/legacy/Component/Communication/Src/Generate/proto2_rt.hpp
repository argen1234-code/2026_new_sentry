/**
 * @file
 * @brief Proto2 RT 自动生成文件，请勿手动修改。
 *
 * @see communication_host_schema.hpp
 */

#ifndef GENERATED_PROTO2_RT_HPP
#define GENERATED_PROTO2_RT_HPP

#include "proto2_rt.h"

#include <cstdint>

namespace proto2 {
namespace rt {

using Status = proto2_rt_status_t;
using Direction = proto2_rt_direction_t;
using Role = proto2_rt_role_t;
using Context = proto2_rt_t;
using Frame = proto2_rt_frame_t;
using FrameFn = proto2_rt_frame_fn;

inline void Init(Context &ctx) {
    proto2_rt_init(&ctx);
}

inline uint16_t PayloadLen(uint8_t msg_id) {
    return proto2_rt_payload_len(msg_id);
}

inline bool CanTx(Role role, uint8_t msg_id) {
    return proto2_rt_can_tx(role, msg_id) != 0u;
}

inline bool CanRx(Role role, uint8_t msg_id) {
    return proto2_rt_can_rx(role, msg_id) != 0u;
}

inline Status FeedByte(Context &ctx, uint8_t byte, Frame &frame) {
    return proto2_rt_feed_byte(&ctx, byte, &frame);
}

inline Status Feed(Context &ctx, const uint8_t *data, uint16_t len, FrameFn on_frame, void *user) {
    return proto2_rt_feed(&ctx, data, len, on_frame, user);
}

using HostAim = proto2_rt_msg_host_aim_t;

inline uint16_t EncodeHostAim(uint8_t *out,
                                         uint16_t out_cap,
                                         uint16_t seq,
                                         const HostAim &msg) {
    return proto2_rt_encode_host_aim(out, out_cap, seq, &msg);
}

inline Status DecodeHostAim(HostAim &out, const uint8_t *payload, uint8_t payload_len) {
    return proto2_rt_decode_host_aim(&out, payload, payload_len);
}

using GimbalState = proto2_rt_msg_gimbal_state_t;

inline uint16_t EncodeGimbalState(uint8_t *out,
                                         uint16_t out_cap,
                                         uint16_t seq,
                                         const GimbalState &msg) {
    return proto2_rt_encode_gimbal_state(out, out_cap, seq, &msg);
}

inline Status DecodeGimbalState(GimbalState &out, const uint8_t *payload, uint8_t payload_len) {
    return proto2_rt_decode_gimbal_state(&out, payload, payload_len);
}

using RtProbe = proto2_rt_msg_rt_probe_t;

inline uint16_t EncodeRtProbe(uint8_t *out,
                                         uint16_t out_cap,
                                         uint16_t seq,
                                         const RtProbe &msg) {
    return proto2_rt_encode_rt_probe(out, out_cap, seq, &msg);
}

inline Status DecodeRtProbe(RtProbe &out, const uint8_t *payload, uint8_t payload_len) {
    return proto2_rt_decode_rt_probe(&out, payload, payload_len);
}

using RtProbeAck = proto2_rt_msg_rt_probe_ack_t;

inline uint16_t EncodeRtProbeAck(uint8_t *out,
                                         uint16_t out_cap,
                                         uint16_t seq,
                                         const RtProbeAck &msg) {
    return proto2_rt_encode_rt_probe_ack(out, out_cap, seq, &msg);
}

inline Status DecodeRtProbeAck(RtProbeAck &out, const uint8_t *payload, uint8_t payload_len) {
    return proto2_rt_decode_rt_probe_ack(&out, payload, payload_len);
}

using CommunicationBoardState = proto2_rt_msg_communication_board_state_t;

inline uint16_t EncodeCommunicationBoardState(uint8_t *out,
                                         uint16_t out_cap,
                                         uint16_t seq,
                                         const CommunicationBoardState &msg) {
    return proto2_rt_encode_communication_board_state(out, out_cap, seq, &msg);
}

inline Status DecodeCommunicationBoardState(CommunicationBoardState &out, const uint8_t *payload, uint8_t payload_len) {
    return proto2_rt_decode_communication_board_state(&out, payload, payload_len);
}

using CommunicationChassisState = proto2_rt_msg_communication_chassis_state_t;

inline uint16_t EncodeCommunicationChassisState(uint8_t *out,
                                         uint16_t out_cap,
                                         uint16_t seq,
                                         const CommunicationChassisState &msg) {
    return proto2_rt_encode_communication_chassis_state(out, out_cap, seq, &msg);
}

inline Status DecodeCommunicationChassisState(CommunicationChassisState &out, const uint8_t *payload, uint8_t payload_len) {
    return proto2_rt_decode_communication_chassis_state(&out, payload, payload_len);
}

}  // namespace rt
}  // namespace proto2

#endif /* GENERATED_PROTO2_RT_HPP */
