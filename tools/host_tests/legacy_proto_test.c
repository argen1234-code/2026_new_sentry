/*
 * Host-side unit test for the sentinel legacy protocol compatibility layer.
 *
 * Build:
 *   cc -std=c11 -Wall -Wextra -I stm32_ws/sentinel/User/Communication/legacy \
 *      tools/host_tests/legacy_proto_test.c \
 *      stm32_ws/sentinel/User/Communication/legacy/legacy_proto.c \
 *      -o /tmp/legacy_proto_test
 */

#include "legacy_proto.h"

#include <stdio.h>
#include <string.h>

static int failures;

static void check(int cond, const char *name)
{
    if (cond) {
        printf("PASS %s\n", name);
    } else {
        printf("FAIL %s\n", name);
        failures++;
    }
}

typedef struct {
    uint8_t bytes[256];
    uint32_t len;
    uint32_t frames;
    legacy_frame_t last;
} capture_t;

static void on_pass(uint8_t byte, void *user)
{
    capture_t *cap = (capture_t *)user;
    if (cap->len < sizeof(cap->bytes)) {
        cap->bytes[cap->len++] = byte;
    }
}

static void on_frame(const legacy_frame_t *frame, void *user)
{
    capture_t *cap = (capture_t *)user;
    cap->frames++;
    cap->last = *frame;
}

static void test_reference_frame(void)
{
    /* Generated with ros_ws/.../usb_bridge/protocol.py encode_unified_cmd(). */
    static const uint8_t expected[] = {
        0xaa, 0x55, 0x10, 0x07, 0x2d, 0xe8, 0x03, 0x00, 0x00, 0x30, 0xf8, 0xff,
        0xff, 0x03, 0x00, 0xfa, 0x00, 0x00, 0x00, 0x88, 0xff, 0xff, 0xff, 0x94,
        0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x01, 0x00,
        0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xcb, 0x04, 0xfb, 0x71, 0x1f, 0x01,
        0x00, 0x00, 0x4a,
    };
    legacy_unified_cmd_t cmd;
    capture_t cap = {0};
    legacy_parser_t parser;

    legacy_parser_init(&parser);
    legacy_parser_feed(&parser, expected, sizeof(expected), on_pass, on_frame, &cap);

    check(cap.frames == 1u, "reference frame parsed");
    check(cap.len == 0u, "reference frame consumed (no passthrough)");
    check(parser.errors == 0u, "reference frame no errors");
    check(legacy_decode_unified_cmd(&cap.last, &cmd) == 1u, "reference frame decodes");
    check(cmd.vx_mm_s == 250, "vx 250");
    check(cmd.vy_mm_s == -120, "vy -120");
    check(cmd.wz_mdeg_s == 4500, "wz 4500");
    check(cmd.chassis_mode == 2, "mode no-follow");
    check(cmd.chassis_status == 3u, "status valid|motion");
    check(cmd.fire_on == 1u, "fire on");
    check(cmd.fire_mode == 1, "fire mode");
    check(cmd.burst_count == 2, "burst count");
    check(cmd.host_ts_ns == 1234567890123ULL, "host timestamp");
    check(cmd.delta_yaw_udeg == 1000, "delta yaw");
    check(cmd.delta_pitch_udeg == -2000, "delta pitch");
}

static void test_split_feed(void)
{
    static const uint8_t frame[] = {
        0xaa, 0x55, 0x10, 0x07, 0x2d, 0xe8, 0x03, 0x00, 0x00, 0x30, 0xf8, 0xff,
        0xff, 0x03, 0x00, 0xfa, 0x00, 0x00, 0x00, 0x88, 0xff, 0xff, 0xff, 0x94,
        0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x01, 0x00,
        0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xcb, 0x04, 0xfb, 0x71, 0x1f, 0x01,
        0x00, 0x00, 0x4a,
    };
    capture_t cap = {0};
    legacy_parser_t parser;
    size_t chunk;

    legacy_parser_init(&parser);
    for (chunk = 1; chunk <= sizeof(frame); chunk++) {
        capture_t partial = {0};
        size_t offset;
        legacy_parser_t split_parser;

        legacy_parser_init(&split_parser);
        for (offset = 0; offset < sizeof(frame); offset += chunk) {
            size_t n = sizeof(frame) - offset;
            if (n > chunk) {
                n = chunk;
            }
            legacy_parser_feed(&split_parser, &frame[offset], (uint32_t)n,
                               on_pass, on_frame, &partial);
        }
        if (partial.frames != 1u || partial.len != 0u) {
            break;
        }
    }
    check(chunk > sizeof(frame), "frame parsed across every chunk size");

    legacy_parser_feed(&parser, frame, sizeof(frame), on_pass, on_frame, &cap);
    check(cap.frames == 1u, "single-chunk parse");
}

static void test_heartbeat(void)
{
    static const uint8_t hb[] = {0xaa, 0x55, 0x01, 0x09, 0x02, 0x03, 0x00, 0x09};
    capture_t cap = {0};
    legacy_parser_t parser;

    legacy_parser_init(&parser);
    legacy_parser_feed(&parser, hb, sizeof(hb), on_pass, on_frame, &cap);
    check(cap.frames == 1u, "heartbeat parsed");
    check(cap.last.msg_id == LEGACY_ID_HOST_HEARTBEAT, "heartbeat id");
    check(cap.last.payload_len == 2u, "heartbeat payload len");
    check(cap.len == 0u, "heartbeat consumed");
}

static void test_uproto_passthrough(void)
{
    /* uproto header starts AA 55 01 ... but byte 4 is UPROTO_MSG_MUX (0x25). */
    static const uint8_t uproto[] = {
        0xaa, 0x55, 0x01, 0x00, 0x25, 0x00, 0x01, 0x00, 0x02, 0x00,
        0x00, 0x00, 0x08, 0x00, 0x12, 0x34, 0xa5, 0x01, 0x02, 0x03,
    };
    capture_t cap = {0};
    legacy_parser_t parser;

    legacy_parser_init(&parser);
    legacy_parser_feed(&parser, uproto, sizeof(uproto), on_pass, on_frame, &cap);
    check(cap.frames == 0u, "uproto not treated as legacy frame");
    check(cap.len == sizeof(uproto), "uproto bytes passed through");
    check(memcmp(cap.bytes, uproto, sizeof(uproto)) == 0, "uproto bytes intact");
}

static void test_bad_checksum(void)
{
    uint8_t frame[8] = {0xaa, 0x55, 0x01, 0x09, 0x02, 0x03, 0x00, 0x00};
    capture_t cap = {0};
    legacy_parser_t parser;

    legacy_parser_init(&parser);
    legacy_parser_feed(&parser, frame, sizeof(frame), on_pass, on_frame, &cap);
    check(cap.frames == 0u, "bad checksum rejected");
    check(parser.errors >= 1u, "bad checksum counted");
    check(cap.len == sizeof(frame), "bad checksum bytes passed to uproto");
}

static void test_multiple_frames(void)
{
    static const uint8_t hb[] = {0xaa, 0x55, 0x01, 0x09, 0x02, 0x03, 0x00, 0x09};
    static const uint8_t frame[] = {
        0xaa, 0x55, 0x10, 0x07, 0x2d, 0xe8, 0x03, 0x00, 0x00, 0x30, 0xf8, 0xff,
        0xff, 0x03, 0x00, 0xfa, 0x00, 0x00, 0x00, 0x88, 0xff, 0xff, 0xff, 0x94,
        0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x01, 0x00,
        0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xcb, 0x04, 0xfb, 0x71, 0x1f, 0x01,
        0x00, 0x00, 0x4a,
    };
    uint8_t stream[sizeof(hb) + sizeof(frame) + sizeof(hb)];
    capture_t cap = {0};
    legacy_parser_t parser;

    memcpy(&stream[0], hb, sizeof(hb));
    memcpy(&stream[sizeof(hb)], frame, sizeof(frame));
    memcpy(&stream[sizeof(hb) + sizeof(frame)], hb, sizeof(hb));

    legacy_parser_init(&parser);
    legacy_parser_feed(&parser, stream, sizeof(stream), on_pass, on_frame, &cap);
    check(cap.frames == 3u, "three frames in one chunk");
    check(cap.len == 0u, "no passthrough for valid stream");
}

static void test_garbage_resync(void)
{
    static const uint8_t garbage[] = {0x00, 0xaa, 0x11, 0xaa, 0x55, 0x99};
    static const uint8_t hb[] = {0xaa, 0x55, 0x01, 0x09, 0x02, 0x03, 0x00, 0x09};
    uint8_t stream[sizeof(garbage) + sizeof(hb)];
    capture_t cap = {0};
    legacy_parser_t parser;

    memcpy(stream, garbage, sizeof(garbage));
    memcpy(&stream[sizeof(garbage)], hb, sizeof(hb));

    legacy_parser_init(&parser);
    legacy_parser_feed(&parser, stream, sizeof(stream), on_pass, on_frame, &cap);
    check(cap.frames == 1u, "resync after garbage");
    check(cap.len == sizeof(garbage), "garbage passed through");
}

static void test_chassis_mapping(void)
{
    legacy_unified_cmd_t cmd = {0};
    int32_t vx;
    int32_t vy;
    int32_t wz;
    int32_t mode;

    cmd.chassis_status = LEGACY_CMD_VALID | LEGACY_CMD_MOTION_ENABLE;
    cmd.chassis_mode = LEGACY_CHASSIS_MODE_NO_FOLLOW;
    cmd.vx_mm_s = 300;
    cmd.vy_mm_s = -200;
    cmd.wz_mdeg_s = 9000;
    legacy_map_chassis_cmd(&cmd, &vx, &vy, &wz, &mode);
    check(mode == SENTINEL_CHASSIS_MODE_NO_FOLLOW, "map no-follow");
    check(vx == 300 && vy == -200 && wz == 9000, "map no-follow velocities");

    cmd.chassis_mode = LEGACY_CHASSIS_MODE_FOLLOW_YAW;
    legacy_map_chassis_cmd(&cmd, &vx, &vy, &wz, &mode);
    check(mode == SENTINEL_CHASSIS_MODE_FOLLOW, "map follow yaw");

    cmd.chassis_mode = LEGACY_CHASSIS_MODE_SPIN;
    legacy_map_chassis_cmd(&cmd, &vx, &vy, &wz, &mode);
    check(mode == SENTINEL_CHASSIS_MODE_SPIN, "map spin");

    cmd.chassis_mode = LEGACY_CHASSIS_MODE_STOP;
    legacy_map_chassis_cmd(&cmd, &vx, &vy, &wz, &mode);
    check(mode == SENTINEL_CHASSIS_MODE_NO_MOVE, "map stop to no-move");
    check(vx == 0 && vy == 0 && wz == 0, "stop zeroes velocities");

    cmd.chassis_mode = LEGACY_CHASSIS_MODE_NO_FOLLOW;
    cmd.chassis_status = LEGACY_CMD_VALID; /* motion disabled */
    legacy_map_chassis_cmd(&cmd, &vx, &vy, &wz, &mode);
    check(mode == SENTINEL_CHASSIS_MODE_NO_MOVE, "motion disabled -> no-move");
    check(vx == 0 && vy == 0 && wz == 0, "motion disabled zeroes velocities");

    cmd.chassis_status = LEGACY_CMD_MOTION_ENABLE; /* not valid */
    legacy_map_chassis_cmd(&cmd, &vx, &vy, &wz, &mode);
    check(mode == SENTINEL_CHASSIS_MODE_NO_MOVE, "invalid -> no-move");
}

static void test_encode_roundtrip(void)
{
    uint8_t payload[LEGACY_UNIFIED_CMD_LEN];
    uint8_t frame[LEGACY_MAX_FRAME];
    legacy_frame_t decoded;
    capture_t cap = {0};
    legacy_parser_t parser;
    uint16_t len;
    uint8_t i;

    for (i = 0; i < sizeof(payload); i++) {
        payload[i] = (uint8_t)(i * 7u + 3u);
    }
    len = legacy_encode_frame(frame, LEGACY_ID_UNIFIED_CMD, 0x42u, payload, sizeof(payload));
    check(len == 51u, "encoded length 51");

    decoded.msg_id = LEGACY_ID_UNIFIED_CMD;
    decoded.seq = 0x42u;
    decoded.payload_len = sizeof(payload);
    memcpy(decoded.payload, payload, sizeof(payload));

    legacy_parser_init(&parser);
    legacy_parser_feed(&parser, frame, len, on_pass, on_frame, &cap);
    check(cap.frames == 1u, "roundtrip frame parsed");
    check(cap.last.seq == 0x42u, "roundtrip seq");
    check(memcmp(cap.last.payload, payload, sizeof(payload)) == 0, "roundtrip payload");
}

int main(void)
{
    test_reference_frame();
    test_split_feed();
    test_heartbeat();
    test_uproto_passthrough();
    test_bad_checksum();
    test_multiple_frames();
    test_garbage_resync();
    test_chassis_mapping();
    test_encode_roundtrip();

    printf("failures=%d\n", failures);
    return (failures == 0) ? 0 : 1;
}
