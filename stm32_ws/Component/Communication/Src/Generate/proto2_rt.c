/**
 * @file
 * @brief Proto2 RT 自动生成文件，请勿手动修改。
 *
 * @see communication_host_schema.hpp
 */

#include "proto2_rt.h"

#include <string.h>

#define PROTO2_RT_STATE_MAGIC_0 0u
#define PROTO2_RT_STATE_MAGIC_1 1u
#define PROTO2_RT_STATE_MSG_ID 2u
#define PROTO2_RT_STATE_SEQ_0 3u
#define PROTO2_RT_STATE_SEQ_1 4u
#define PROTO2_RT_STATE_PAYLOAD 5u
#define PROTO2_RT_STATE_CRC_0 6u
#define PROTO2_RT_STATE_CRC_1 7u

static uint16_t proto2_rt_read_u16_le(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t proto2_rt_read_u32_le(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static float proto2_rt_read_f32_le(const uint8_t *data) {
    uint32_t bits = proto2_rt_read_u32_le(data);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void proto2_rt_write_u16_le(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void proto2_rt_write_u32_le(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void proto2_rt_write_f32_le(uint8_t *data, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    proto2_rt_write_u32_le(data, bits);
}

static uint16_t proto2_rt_crc16_update(uint16_t crc, uint8_t byte) {
    uint8_t bit;
    crc ^= (uint16_t)byte << 8;
    for (bit = 0u; bit < 8u; ++bit) {
        if ((crc & 0x8000u) != 0u) {
            crc = (uint16_t)((crc << 1) ^ 0x1021u);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t proto2_rt_crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0u; i < len; ++i) {
        crc = proto2_rt_crc16_update(crc, data[i]);
    }
    return crc;
}

void proto2_rt_init(proto2_rt_t *rt) {
    if (!rt) {
        return;
    }
    memset(rt, 0, sizeof(*rt));
}

uint16_t proto2_rt_payload_len(uint8_t msg_id) {
    switch (msg_id) {
    case PROTO2_RT_MSG_ID_HOST_AIM:
        return PROTO2_RT_WIRE_SIZE_HOST_AIM;
    case PROTO2_RT_MSG_ID_GIMBAL_STATE:
        return PROTO2_RT_WIRE_SIZE_GIMBAL_STATE;
    case PROTO2_RT_MSG_ID_RT_PROBE:
        return PROTO2_RT_WIRE_SIZE_RT_PROBE;
    case PROTO2_RT_MSG_ID_RT_PROBE_ACK:
        return PROTO2_RT_WIRE_SIZE_RT_PROBE_ACK;
    case PROTO2_RT_MSG_ID_COMMUNICATION_BOARD_STATE:
        return PROTO2_RT_WIRE_SIZE_COMMUNICATION_BOARD_STATE;
    case PROTO2_RT_MSG_ID_COMMUNICATION_CHASSIS_STATE:
        return PROTO2_RT_WIRE_SIZE_COMMUNICATION_CHASSIS_STATE;
    default:
        return 0xFFFFu;
    }
}

static proto2_rt_direction_t proto2_rt_message_direction(uint8_t msg_id) {
    switch (msg_id) {
    case PROTO2_RT_MSG_ID_HOST_AIM:
        return PROTO2_RT_DIRECTION_HOST_AIM;
    case PROTO2_RT_MSG_ID_GIMBAL_STATE:
        return PROTO2_RT_DIRECTION_GIMBAL_STATE;
    case PROTO2_RT_MSG_ID_RT_PROBE:
        return PROTO2_RT_DIRECTION_RT_PROBE;
    case PROTO2_RT_MSG_ID_RT_PROBE_ACK:
        return PROTO2_RT_DIRECTION_RT_PROBE_ACK;
    case PROTO2_RT_MSG_ID_COMMUNICATION_BOARD_STATE:
        return PROTO2_RT_DIRECTION_COMMUNICATION_BOARD_STATE;
    case PROTO2_RT_MSG_ID_COMMUNICATION_CHASSIS_STATE:
        return PROTO2_RT_DIRECTION_COMMUNICATION_CHASSIS_STATE;
    default:
        return PROTO2_RT_DIRECTION_BIDIRECTIONAL;
    }
}

static uint8_t proto2_rt_direction_allowed(proto2_rt_role_t role, proto2_rt_direction_t direction, uint8_t tx) {
    if (role == PROTO2_RT_ROLE_ANY || direction == PROTO2_RT_DIRECTION_BIDIRECTIONAL) {
        return 1u;
    }
    if (tx != 0u) {
        if (role == PROTO2_RT_ROLE_DEVICE) {
            return direction == PROTO2_RT_DIRECTION_DEVICE_TO_HOST;
        }
        if (role == PROTO2_RT_ROLE_HOST) {
            return direction == PROTO2_RT_DIRECTION_HOST_TO_DEVICE;
        }
        return 0u;
    }
    if (role == PROTO2_RT_ROLE_DEVICE) {
        return direction == PROTO2_RT_DIRECTION_HOST_TO_DEVICE;
    }
    if (role == PROTO2_RT_ROLE_HOST) {
        return direction == PROTO2_RT_DIRECTION_DEVICE_TO_HOST;
    }
    return 0u;
}

uint8_t proto2_rt_can_tx(proto2_rt_role_t role, uint8_t msg_id) {
    if (proto2_rt_payload_len(msg_id) == 0xFFFFu) {
        return 0u;
    }
    return proto2_rt_direction_allowed(role, proto2_rt_message_direction(msg_id), 1u);
}

uint8_t proto2_rt_can_rx(proto2_rt_role_t role, uint8_t msg_id) {
    if (proto2_rt_payload_len(msg_id) == 0xFFFFu) {
        return 0u;
    }
    return proto2_rt_direction_allowed(role, proto2_rt_message_direction(msg_id), 0u);
}

static void proto2_rt_note_seq(proto2_rt_t *rt, uint8_t msg_id, uint16_t seq) {
    switch (msg_id) {
    case PROTO2_RT_MSG_ID_HOST_AIM:
        if (rt->have_host_aim != 0u && (uint16_t)(rt->last_seq_host_aim + 1u) != seq) {
            rt->seq_errors++;
        }
        rt->last_seq_host_aim = seq;
        rt->have_host_aim = 1u;
        break;
    case PROTO2_RT_MSG_ID_GIMBAL_STATE:
        if (rt->have_gimbal_state != 0u && (uint16_t)(rt->last_seq_gimbal_state + 1u) != seq) {
            rt->seq_errors++;
        }
        rt->last_seq_gimbal_state = seq;
        rt->have_gimbal_state = 1u;
        break;
    case PROTO2_RT_MSG_ID_RT_PROBE:
        if (rt->have_rt_probe != 0u && (uint16_t)(rt->last_seq_rt_probe + 1u) != seq) {
            rt->seq_errors++;
        }
        rt->last_seq_rt_probe = seq;
        rt->have_rt_probe = 1u;
        break;
    case PROTO2_RT_MSG_ID_RT_PROBE_ACK:
        if (rt->have_rt_probe_ack != 0u && (uint16_t)(rt->last_seq_rt_probe_ack + 1u) != seq) {
            rt->seq_errors++;
        }
        rt->last_seq_rt_probe_ack = seq;
        rt->have_rt_probe_ack = 1u;
        break;
    case PROTO2_RT_MSG_ID_COMMUNICATION_BOARD_STATE:
        if (rt->have_communication_board_state != 0u && (uint16_t)(rt->last_seq_communication_board_state + 1u) != seq) {
            rt->seq_errors++;
        }
        rt->last_seq_communication_board_state = seq;
        rt->have_communication_board_state = 1u;
        break;
    case PROTO2_RT_MSG_ID_COMMUNICATION_CHASSIS_STATE:
        if (rt->have_communication_chassis_state != 0u && (uint16_t)(rt->last_seq_communication_chassis_state + 1u) != seq) {
            rt->seq_errors++;
        }
        rt->last_seq_communication_chassis_state = seq;
        rt->have_communication_chassis_state = 1u;
        break;
    default:
        break;
    }
}

static void proto2_rt_reset_rx(proto2_rt_t *rt) {
    rt->state = PROTO2_RT_STATE_MAGIC_0;
    rt->msg_id = 0u;
    rt->payload_len = 0u;
    rt->payload_pos = 0u;
    rt->seq = 0u;
    rt->rx_crc = 0u;
    rt->crc_calc = 0xFFFFu;
}

static void proto2_rt_begin_rx(proto2_rt_t *rt) {
    rt->state = PROTO2_RT_STATE_MAGIC_1;
    rt->msg_id = 0u;
    rt->payload_len = 0u;
    rt->payload_pos = 0u;
    rt->seq = 0u;
    rt->rx_crc = 0u;
    rt->crc_calc = 0xFFFFu;
    rt->crc_calc = proto2_rt_crc16_update(rt->crc_calc, PROTO2_RT_MAGIC_0);
}

static void proto2_rt_clear_frame(proto2_rt_frame_t *frame) {
    if (frame) {
        frame->msg_id = 0u;
        frame->seq = 0u;
        frame->payload = NULL;
        frame->payload_len = 0u;
    }
}

static proto2_rt_status_t proto2_rt_finish_frame(proto2_rt_t *rt, proto2_rt_frame_t *frame) {
    if (rt->crc_calc != rt->rx_crc) {
        rt->crc_errors++;
        return PROTO2_RT_STATUS_CRC;
    }
    if (proto2_rt_can_rx(rt->role, rt->msg_id) == 0u) {
        return PROTO2_RT_STATUS_DIRECTION;
    }
    rt->rx_frames++;
    proto2_rt_note_seq(rt, rt->msg_id, rt->seq);
    if (frame) {
        frame->msg_id = rt->msg_id;
        frame->seq = rt->seq;
        frame->payload = rt->payload;
        frame->payload_len = rt->payload_len;
    }
    return PROTO2_RT_STATUS_OK;
}

proto2_rt_status_t proto2_rt_feed_byte(proto2_rt_t *rt, uint8_t byte, proto2_rt_frame_t *frame) {
    proto2_rt_status_t status = PROTO2_RT_STATUS_OK;
    proto2_rt_clear_frame(frame);
    if (!rt) {
        return PROTO2_RT_STATUS_NULL;
    }
    switch (rt->state) {
    case PROTO2_RT_STATE_MAGIC_0:
        if (byte == PROTO2_RT_MAGIC_0) {
            proto2_rt_begin_rx(rt);
        }
        break;
    case PROTO2_RT_STATE_MAGIC_1:
        if (byte == PROTO2_RT_MAGIC_1) {
            rt->crc_calc = proto2_rt_crc16_update(rt->crc_calc, byte);
            rt->state = PROTO2_RT_STATE_MSG_ID;
        } else if (byte == PROTO2_RT_MAGIC_0) {
            proto2_rt_begin_rx(rt);
        } else {
            proto2_rt_reset_rx(rt);
        }
        break;
    case PROTO2_RT_STATE_MSG_ID: {
        uint16_t payload_len = proto2_rt_payload_len(byte);
        rt->crc_calc = proto2_rt_crc16_update(rt->crc_calc, byte);
        if (payload_len == 0xFFFFu || payload_len > PROTO2_RT_MAX_PAYLOAD) {
            rt->id_errors++;
            status = PROTO2_RT_STATUS_ID;
            proto2_rt_reset_rx(rt);
            if (byte == PROTO2_RT_MAGIC_0) {
                proto2_rt_begin_rx(rt);
            }
            break;
        }
        rt->msg_id = byte;
        rt->payload_len = (uint8_t)payload_len;
        rt->payload_pos = 0u;
        rt->state = PROTO2_RT_STATE_SEQ_0;
        break;
    }
    case PROTO2_RT_STATE_SEQ_0:
        rt->crc_calc = proto2_rt_crc16_update(rt->crc_calc, byte);
        rt->seq = byte;
        rt->state = PROTO2_RT_STATE_SEQ_1;
        break;
    case PROTO2_RT_STATE_SEQ_1:
        rt->crc_calc = proto2_rt_crc16_update(rt->crc_calc, byte);
        rt->seq |= (uint16_t)byte << 8;
        rt->state = (rt->payload_len == 0u) ? PROTO2_RT_STATE_CRC_0 : PROTO2_RT_STATE_PAYLOAD;
        break;
    case PROTO2_RT_STATE_PAYLOAD:
        rt->crc_calc = proto2_rt_crc16_update(rt->crc_calc, byte);
        rt->payload[rt->payload_pos++] = byte;
        if (rt->payload_pos >= rt->payload_len) {
            rt->state = PROTO2_RT_STATE_CRC_0;
        }
        break;
    case PROTO2_RT_STATE_CRC_0:
        rt->rx_crc = byte;
        rt->state = PROTO2_RT_STATE_CRC_1;
        break;
    case PROTO2_RT_STATE_CRC_1:
        rt->rx_crc |= (uint16_t)byte << 8;
        status = proto2_rt_finish_frame(rt, frame);
        proto2_rt_reset_rx(rt);
        break;
    default:
        proto2_rt_reset_rx(rt);
        break;
    }
    return status;
}

proto2_rt_status_t proto2_rt_feed(proto2_rt_t *rt,
                                  const uint8_t *data,
                                  uint16_t len,
                                  proto2_rt_frame_fn on_frame,
                                  void *user) {
    proto2_rt_status_t status = PROTO2_RT_STATUS_OK;
    uint16_t index;
    if (!rt || (!data && len != 0u)) {
        return PROTO2_RT_STATUS_NULL;
    }
    for (index = 0u; index < len; ++index) {
        proto2_rt_frame_t frame;
        proto2_rt_status_t one = proto2_rt_feed_byte(rt, data[index], &frame);
        if (one != PROTO2_RT_STATUS_OK) {
            status = one;
        }
        if (one == PROTO2_RT_STATUS_OK && frame.payload != NULL && on_frame) {
            on_frame(user, &frame);
        }
    }
    return status;
}

uint16_t proto2_rt_encode_host_aim(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_host_aim_t *msg) {
    uint16_t crc;
    if (!out || !msg || out_cap < PROTO2_RT_FRAME_SIZE_HOST_AIM) {
        return 0u;
    }
    out[0] = PROTO2_RT_MAGIC_0;
    out[1] = PROTO2_RT_MAGIC_1;
    out[2] = PROTO2_RT_MSG_ID_HOST_AIM;
    proto2_rt_write_u16_le(&out[3], seq);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 0u], msg->yaw_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 4u], msg->pitch_rad);
out[PROTO2_RT_HEADER_SIZE + 8u] = (uint8_t)(msg->valid_mask);
out[PROTO2_RT_HEADER_SIZE + 9u] = (uint8_t)(msg->enable);
proto2_rt_write_u32_le(&out[PROTO2_RT_HEADER_SIZE + 10u], (uint32_t)(msg->host_time_us));
    crc = proto2_rt_crc16_ccitt(out, (uint16_t)(PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_HOST_AIM));
    proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_HOST_AIM], crc);
    return PROTO2_RT_FRAME_SIZE_HOST_AIM;
}

proto2_rt_status_t proto2_rt_decode_host_aim(proto2_rt_msg_host_aim_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len) {
    if (!out || (!payload && payload_len != 0u)) {
        return PROTO2_RT_STATUS_NULL;
    }
    if (payload_len != PROTO2_RT_WIRE_SIZE_HOST_AIM) {
        return PROTO2_RT_STATUS_LENGTH;
    }
out->yaw_rad = proto2_rt_read_f32_le(&payload[0u]);
out->pitch_rad = proto2_rt_read_f32_le(&payload[4u]);
out->valid_mask = (uint8_t)payload[8u];
out->enable = (uint8_t)payload[9u];
out->host_time_us = (uint32_t)proto2_rt_read_u32_le(&payload[10u]);
    return PROTO2_RT_STATUS_OK;
}

uint16_t proto2_rt_encode_gimbal_state(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_gimbal_state_t *msg) {
    uint16_t crc;
    if (!out || !msg || out_cap < PROTO2_RT_FRAME_SIZE_GIMBAL_STATE) {
        return 0u;
    }
    out[0] = PROTO2_RT_MAGIC_0;
    out[1] = PROTO2_RT_MAGIC_1;
    out[2] = PROTO2_RT_MSG_ID_GIMBAL_STATE;
    proto2_rt_write_u16_le(&out[3], seq);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 0u], msg->yaw_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 4u], msg->pitch_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 8u], msg->yaw_rate_rad_s);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 12u], msg->pitch_rate_rad_s);
out[PROTO2_RT_HEADER_SIZE + 16u] = (uint8_t)(msg->status);
proto2_rt_write_u32_le(&out[PROTO2_RT_HEADER_SIZE + 17u], (uint32_t)(msg->time_us));
    crc = proto2_rt_crc16_ccitt(out, (uint16_t)(PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_GIMBAL_STATE));
    proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_GIMBAL_STATE], crc);
    return PROTO2_RT_FRAME_SIZE_GIMBAL_STATE;
}

proto2_rt_status_t proto2_rt_decode_gimbal_state(proto2_rt_msg_gimbal_state_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len) {
    if (!out || (!payload && payload_len != 0u)) {
        return PROTO2_RT_STATUS_NULL;
    }
    if (payload_len != PROTO2_RT_WIRE_SIZE_GIMBAL_STATE) {
        return PROTO2_RT_STATUS_LENGTH;
    }
out->yaw_rad = proto2_rt_read_f32_le(&payload[0u]);
out->pitch_rad = proto2_rt_read_f32_le(&payload[4u]);
out->yaw_rate_rad_s = proto2_rt_read_f32_le(&payload[8u]);
out->pitch_rate_rad_s = proto2_rt_read_f32_le(&payload[12u]);
out->status = (uint8_t)payload[16u];
out->time_us = (uint32_t)proto2_rt_read_u32_le(&payload[17u]);
    return PROTO2_RT_STATUS_OK;
}

uint16_t proto2_rt_encode_rt_probe(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_rt_probe_t *msg) {
    uint16_t crc;
    if (!out || !msg || out_cap < PROTO2_RT_FRAME_SIZE_RT_PROBE) {
        return 0u;
    }
    out[0] = PROTO2_RT_MAGIC_0;
    out[1] = PROTO2_RT_MAGIC_1;
    out[2] = PROTO2_RT_MSG_ID_RT_PROBE;
    proto2_rt_write_u16_le(&out[3], seq);
proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + 0u], (uint16_t)(msg->token));
proto2_rt_write_u32_le(&out[PROTO2_RT_HEADER_SIZE + 2u], (uint32_t)(msg->time_us));
    crc = proto2_rt_crc16_ccitt(out, (uint16_t)(PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_RT_PROBE));
    proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_RT_PROBE], crc);
    return PROTO2_RT_FRAME_SIZE_RT_PROBE;
}

proto2_rt_status_t proto2_rt_decode_rt_probe(proto2_rt_msg_rt_probe_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len) {
    if (!out || (!payload && payload_len != 0u)) {
        return PROTO2_RT_STATUS_NULL;
    }
    if (payload_len != PROTO2_RT_WIRE_SIZE_RT_PROBE) {
        return PROTO2_RT_STATUS_LENGTH;
    }
out->token = (uint16_t)proto2_rt_read_u16_le(&payload[0u]);
out->time_us = (uint32_t)proto2_rt_read_u32_le(&payload[2u]);
    return PROTO2_RT_STATUS_OK;
}

uint16_t proto2_rt_encode_rt_probe_ack(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_rt_probe_ack_t *msg) {
    uint16_t crc;
    if (!out || !msg || out_cap < PROTO2_RT_FRAME_SIZE_RT_PROBE_ACK) {
        return 0u;
    }
    out[0] = PROTO2_RT_MAGIC_0;
    out[1] = PROTO2_RT_MAGIC_1;
    out[2] = PROTO2_RT_MSG_ID_RT_PROBE_ACK;
    proto2_rt_write_u16_le(&out[3], seq);
proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + 0u], (uint16_t)(msg->token));
proto2_rt_write_u32_le(&out[PROTO2_RT_HEADER_SIZE + 2u], (uint32_t)(msg->request_time_us));
proto2_rt_write_u32_le(&out[PROTO2_RT_HEADER_SIZE + 6u], (uint32_t)(msg->reply_time_us));
    crc = proto2_rt_crc16_ccitt(out, (uint16_t)(PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_RT_PROBE_ACK));
    proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_RT_PROBE_ACK], crc);
    return PROTO2_RT_FRAME_SIZE_RT_PROBE_ACK;
}

proto2_rt_status_t proto2_rt_decode_rt_probe_ack(proto2_rt_msg_rt_probe_ack_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len) {
    if (!out || (!payload && payload_len != 0u)) {
        return PROTO2_RT_STATUS_NULL;
    }
    if (payload_len != PROTO2_RT_WIRE_SIZE_RT_PROBE_ACK) {
        return PROTO2_RT_STATUS_LENGTH;
    }
out->token = (uint16_t)proto2_rt_read_u16_le(&payload[0u]);
out->request_time_us = (uint32_t)proto2_rt_read_u32_le(&payload[2u]);
out->reply_time_us = (uint32_t)proto2_rt_read_u32_le(&payload[6u]);
    return PROTO2_RT_STATUS_OK;
}

uint16_t proto2_rt_encode_communication_board_state(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_communication_board_state_t *msg) {
    uint16_t crc;
    if (!out || !msg || out_cap < PROTO2_RT_FRAME_SIZE_COMMUNICATION_BOARD_STATE) {
        return 0u;
    }
    out[0] = PROTO2_RT_MAGIC_0;
    out[1] = PROTO2_RT_MAGIC_1;
    out[2] = PROTO2_RT_MSG_ID_COMMUNICATION_BOARD_STATE;
    proto2_rt_write_u16_le(&out[3], seq);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 0u], msg->yaw_motor_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 4u], msg->ins_yaw_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 8u], msg->ins_pitch_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 12u], msg->ins_roll_rad);
out[PROTO2_RT_HEADER_SIZE + 16u] = (uint8_t)(msg->gimbal_mode);
out[PROTO2_RT_HEADER_SIZE + 17u] = (uint8_t)(msg->shoot_mode);
proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + 18u], (uint16_t)(msg->rc_ch0));
proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + 20u], (uint16_t)(msg->rc_ch1));
proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + 22u], (uint16_t)(msg->rc_ch2));
proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + 24u], (uint16_t)(msg->rc_ch3));
out[PROTO2_RT_HEADER_SIZE + 26u] = (uint8_t)(msg->rc_sw0);
out[PROTO2_RT_HEADER_SIZE + 27u] = (uint8_t)(msg->rc_sw1);
out[PROTO2_RT_HEADER_SIZE + 28u] = (uint8_t)(msg->status);
proto2_rt_write_u32_le(&out[PROTO2_RT_HEADER_SIZE + 29u], (uint32_t)(msg->time_us));
    crc = proto2_rt_crc16_ccitt(out, (uint16_t)(PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_COMMUNICATION_BOARD_STATE));
    proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_COMMUNICATION_BOARD_STATE], crc);
    return PROTO2_RT_FRAME_SIZE_COMMUNICATION_BOARD_STATE;
}

proto2_rt_status_t proto2_rt_decode_communication_board_state(proto2_rt_msg_communication_board_state_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len) {
    if (!out || (!payload && payload_len != 0u)) {
        return PROTO2_RT_STATUS_NULL;
    }
    if (payload_len != PROTO2_RT_WIRE_SIZE_COMMUNICATION_BOARD_STATE) {
        return PROTO2_RT_STATUS_LENGTH;
    }
out->yaw_motor_rad = proto2_rt_read_f32_le(&payload[0u]);
out->ins_yaw_rad = proto2_rt_read_f32_le(&payload[4u]);
out->ins_pitch_rad = proto2_rt_read_f32_le(&payload[8u]);
out->ins_roll_rad = proto2_rt_read_f32_le(&payload[12u]);
out->gimbal_mode = (uint8_t)payload[16u];
out->shoot_mode = (uint8_t)payload[17u];
out->rc_ch0 = (int16_t)proto2_rt_read_u16_le(&payload[18u]);
out->rc_ch1 = (int16_t)proto2_rt_read_u16_le(&payload[20u]);
out->rc_ch2 = (int16_t)proto2_rt_read_u16_le(&payload[22u]);
out->rc_ch3 = (int16_t)proto2_rt_read_u16_le(&payload[24u]);
out->rc_sw0 = (uint8_t)payload[26u];
out->rc_sw1 = (uint8_t)payload[27u];
out->status = (uint8_t)payload[28u];
out->time_us = (uint32_t)proto2_rt_read_u32_le(&payload[29u]);
    return PROTO2_RT_STATUS_OK;
}

uint16_t proto2_rt_encode_communication_chassis_state(uint8_t *out,
                                              uint16_t out_cap,
                                              uint16_t seq,
                                              const proto2_rt_msg_communication_chassis_state_t *msg) {
    uint16_t crc;
    if (!out || !msg || out_cap < PROTO2_RT_FRAME_SIZE_COMMUNICATION_CHASSIS_STATE) {
        return 0u;
    }
    out[0] = PROTO2_RT_MAGIC_0;
    out[1] = PROTO2_RT_MAGIC_1;
    out[2] = PROTO2_RT_MSG_ID_COMMUNICATION_CHASSIS_STATE;
    proto2_rt_write_u16_le(&out[3], seq);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 0u], msg->yaw_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 4u], msg->pitch_rad);
proto2_rt_write_f32_le(&out[PROTO2_RT_HEADER_SIZE + 8u], msg->roll_rad);
proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + 12u], (uint16_t)(msg->robot_id));
out[PROTO2_RT_HEADER_SIZE + 14u] = (uint8_t)(msg->status);
proto2_rt_write_u32_le(&out[PROTO2_RT_HEADER_SIZE + 15u], (uint32_t)(msg->time_us));
    crc = proto2_rt_crc16_ccitt(out, (uint16_t)(PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_COMMUNICATION_CHASSIS_STATE));
    proto2_rt_write_u16_le(&out[PROTO2_RT_HEADER_SIZE + PROTO2_RT_WIRE_SIZE_COMMUNICATION_CHASSIS_STATE], crc);
    return PROTO2_RT_FRAME_SIZE_COMMUNICATION_CHASSIS_STATE;
}

proto2_rt_status_t proto2_rt_decode_communication_chassis_state(proto2_rt_msg_communication_chassis_state_t *out,
                                                        const uint8_t *payload,
                                                        uint8_t payload_len) {
    if (!out || (!payload && payload_len != 0u)) {
        return PROTO2_RT_STATUS_NULL;
    }
    if (payload_len != PROTO2_RT_WIRE_SIZE_COMMUNICATION_CHASSIS_STATE) {
        return PROTO2_RT_STATUS_LENGTH;
    }
out->yaw_rad = proto2_rt_read_f32_le(&payload[0u]);
out->pitch_rad = proto2_rt_read_f32_le(&payload[4u]);
out->roll_rad = proto2_rt_read_f32_le(&payload[8u]);
out->robot_id = (uint16_t)proto2_rt_read_u16_le(&payload[12u]);
out->status = (uint8_t)payload[14u];
out->time_us = (uint32_t)proto2_rt_read_u32_le(&payload[15u]);
    return PROTO2_RT_STATUS_OK;
}

