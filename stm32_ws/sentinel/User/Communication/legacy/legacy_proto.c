/**
  * @file       legacy_proto.c
  * @brief      旧版 USB 桥协议编解码与命令映射实现
  */

#include "legacy_proto.h"

#include <string.h>

/* ========================= 小端读写 ========================= */

uint16_t legacy_read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t legacy_read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

int32_t legacy_read_i32(const uint8_t *p)
{
    return (int32_t)legacy_read_u32(p);
}

uint64_t legacy_read_u64(const uint8_t *p)
{
    return (uint64_t)legacy_read_u32(p) | ((uint64_t)legacy_read_u32(p + 4) << 32);
}

float legacy_read_f32(const uint8_t *p)
{
    uint32_t raw = legacy_read_u32(p);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

void legacy_write_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

void legacy_write_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

void legacy_write_i32(uint8_t *p, int32_t v)
{
    legacy_write_u32(p, (uint32_t)v);
}

void legacy_write_u64(uint8_t *p, uint64_t v)
{
    legacy_write_u32(p, (uint32_t)(v & 0xFFFFFFFFu));
    legacy_write_u32(p + 4, (uint32_t)(v >> 32));
}

void legacy_write_f32(uint8_t *p, float v)
{
    uint32_t raw;
    memcpy(&raw, &v, sizeof(raw));
    legacy_write_u32(p, raw);
}

/* ========================= 组帧与校验 ========================= */

static uint8_t legacy_frame_xor(const uint8_t *frame, uint16_t body_len)
{
    uint8_t value = frame[2] ^ frame[3] ^ frame[4]; /* id ^ seq ^ len */

    for (uint16_t i = 0u; i < body_len; i++)
    {
        value ^= frame[5u + i];
    }
    return value;
}

uint16_t legacy_encode_frame(uint8_t *out,
                             uint8_t msg_id,
                             uint8_t seq,
                             const uint8_t *payload,
                             uint8_t payload_len)
{
    if ((out == NULL) || (payload_len > LEGACY_MAX_PAYLOAD))
    {
        return 0u;
    }

    out[0] = LEGACY_SOF0;
    out[1] = LEGACY_SOF1;
    out[2] = msg_id;
    out[3] = seq;
    out[4] = payload_len;
    if ((payload != NULL) && (payload_len != 0u))
    {
        memcpy(&out[5], payload, payload_len);
    }
    out[5u + payload_len] = legacy_frame_xor(out, payload_len);

    return (uint16_t)(6u + payload_len);
}

/* ========================= 增量解析 ========================= */

/**
 * @brief 返回候选帧总长；0 表示不是本协议的主机帧
 * @details 只接受旧固件实际会发送的两种主机帧，防止把 uproto 帧误吞。
 */
static uint16_t legacy_expected_len(uint8_t msg_id, uint8_t len_field)
{
    if ((msg_id == LEGACY_ID_HOST_HEARTBEAT) && (len_field == 2u))
    {
        return (uint16_t)(5u + 2u + 1u);
    }
    if ((msg_id == LEGACY_ID_UNIFIED_CMD) && (len_field == (uint8_t)LEGACY_UNIFIED_CMD_LEN))
    {
        return (uint16_t)(5u + LEGACY_UNIFIED_CMD_LEN + 1u);
    }
    return 0u;
}

void legacy_parser_init(legacy_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }
    memset(parser, 0, sizeof(*parser));
}

void legacy_parser_feed(legacy_parser_t *parser,
                        const uint8_t *data,
                        uint32_t len,
                        legacy_pass_fn pass,
                        legacy_frame_fn on_frame,
                        void *user)
{
    if ((parser == NULL) || ((data == NULL) && (len != 0u)))
    {
        return;
    }

    for (uint32_t i = 0u; i < len; i++)
    {
        /* 单字节推进，缓冲区最多只保留一个候选帧 */
        parser->buf[parser->len++] = data[i];

        for (;;)
        {
            uint16_t expected;

            if (parser->len < 2u)
            {
                /* 只有一个字节且不是帧头时立即透传，避免无谓延迟 */
                if ((parser->len == 1u) && (parser->buf[0] != LEGACY_SOF0))
                {
                    if (pass != NULL)
                    {
                        pass(parser->buf[0], user);
                    }
                    parser->len = 0u;
                    continue;
                }
                break;
            }
            if ((parser->buf[0] != LEGACY_SOF0) || (parser->buf[1] != LEGACY_SOF1))
            {
                if (pass != NULL)
                {
                    pass(parser->buf[0], user);
                }
                parser->len--;
                memmove(&parser->buf[0], &parser->buf[1], parser->len);
                continue;
            }
            if (parser->len < 5u)
            {
                break;
            }

            expected = legacy_expected_len(parser->buf[2], parser->buf[4]);
            if (expected == 0u)
            {
                if (pass != NULL)
                {
                    pass(parser->buf[0], user);
                }
                parser->len--;
                memmove(&parser->buf[0], &parser->buf[1], parser->len);
                continue;
            }
            if (parser->len < expected)
            {
                break;
            }

            if (legacy_frame_xor(parser->buf, parser->buf[4]) == parser->buf[expected - 1u])
            {
                legacy_frame_t frame;

                frame.msg_id = parser->buf[2];
                frame.seq = parser->buf[3];
                frame.payload_len = parser->buf[4];
                if (frame.payload_len != 0u)
                {
                    memcpy(frame.payload, &parser->buf[5], frame.payload_len);
                }
                parser->rx_frames++;
                parser->len = (uint16_t)(parser->len - expected);
                memmove(&parser->buf[0], &parser->buf[expected], parser->len);
                if (on_frame != NULL)
                {
                    on_frame(&frame, user);
                }
            }
            else
            {
                /* 校验失败：丢弃起始字节后重新同步 */
                parser->errors++;
                if (pass != NULL)
                {
                    pass(parser->buf[0], user);
                }
                parser->len--;
                memmove(&parser->buf[0], &parser->buf[1], parser->len);
            }
        }
    }
}

/* ========================= UnifiedCmd ========================= */

uint8_t legacy_decode_unified_cmd(const legacy_frame_t *frame,
                                  legacy_unified_cmd_t *out)
{
    const uint8_t *p;

    if ((frame == NULL) || (out == NULL))
    {
        return 0u;
    }
    if ((frame->msg_id != LEGACY_ID_UNIFIED_CMD) ||
        (frame->payload_len != (uint8_t)LEGACY_UNIFIED_CMD_LEN))
    {
        return 0u;
    }

    p = frame->payload;
    out->delta_yaw_udeg = legacy_read_i32(&p[0]);
    out->delta_pitch_udeg = legacy_read_i32(&p[4]);
    out->gimbal_status = legacy_read_u16(&p[8]);
    out->vx_mm_s = legacy_read_i32(&p[10]);
    out->vy_mm_s = legacy_read_i32(&p[14]);
    out->wz_mdeg_s = legacy_read_i32(&p[18]);
    out->chassis_mode = legacy_read_i32(&p[22]);
    out->chassis_status = legacy_read_u16(&p[26]);
    out->fire_on = p[28];
    out->fire_mode = legacy_read_i32(&p[29]);
    out->burst_count = legacy_read_i32(&p[33]);
    out->host_ts_ns = legacy_read_u64(&p[37]);
    return 1u;
}

void legacy_map_chassis_cmd(const legacy_unified_cmd_t *in,
                            int32_t *vx_mm_s,
                            int32_t *vy_mm_s,
                            int32_t *wz_mdeg_s,
                            int32_t *upstream_mode)
{
    int32_t vx = 0;
    int32_t vy = 0;
    int32_t wz = 0;
    int32_t mode = SENTINEL_CHASSIS_MODE_NO_MOVE;

    if (in != NULL)
    {
        const uint8_t valid = ((in->chassis_status & LEGACY_CMD_VALID) != 0u) ? 1u : 0u;
        const uint8_t motion = ((in->chassis_status & LEGACY_CMD_MOTION_ENABLE) != 0u) ? 1u : 0u;

        if ((valid != 0u) && (motion != 0u))
        {
            switch (in->chassis_mode)
            {
            case LEGACY_CHASSIS_MODE_NO_FOLLOW:
                mode = SENTINEL_CHASSIS_MODE_NO_FOLLOW;
                vx = in->vx_mm_s;
                vy = in->vy_mm_s;
                wz = in->wz_mdeg_s;
                break;
            case LEGACY_CHASSIS_MODE_FOLLOW_YAW:
                mode = SENTINEL_CHASSIS_MODE_FOLLOW;
                vx = in->vx_mm_s;
                vy = in->vy_mm_s;
                wz = in->wz_mdeg_s;
                break;
            case LEGACY_CHASSIS_MODE_SPIN:
                mode = SENTINEL_CHASSIS_MODE_SPIN;
                vx = in->vx_mm_s;
                vy = in->vy_mm_s;
                wz = in->wz_mdeg_s;
                break;
            default:
                /* Stop / Raw / FollowChassisYaw / Balance 上游无对应模式 */
                mode = SENTINEL_CHASSIS_MODE_NO_MOVE;
                break;
            }
        }
    }

    if (vx_mm_s != NULL)
    {
        *vx_mm_s = vx;
    }
    if (vy_mm_s != NULL)
    {
        *vy_mm_s = vy;
    }
    if (wz_mdeg_s != NULL)
    {
        *wz_mdeg_s = wz;
    }
    if (upstream_mode != NULL)
    {
        *upstream_mode = mode;
    }
}
