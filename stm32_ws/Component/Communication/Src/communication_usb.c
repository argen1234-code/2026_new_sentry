/**
 ******************************************************************************
 * @file    communication_usb.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-19
 * @brief   Jetson <-> STM32 USB CDC 桥接实现。
 *
 * @note    接收在 USB 中断中只入环形缓冲，协议解析在 app 任务中完成。
 * @note    本模块不直接写电机输出，只缓存 Jetson 命令供上层 App 读取。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "communication_usb.h"

#include "bsp_cdc.h"
#include "main.h"

#include <string.h>

/* ==========================================================================
 * 私有配置与状态
 * ========================================================================== */

#define COMMUNICATION_USB_RX_RING_SIZE               512U

typedef enum
{
    COMMUNICATION_USB_RX_SOF0 = 0,
    COMMUNICATION_USB_RX_SOF1,
    COMMUNICATION_USB_RX_ID,
    COMMUNICATION_USB_RX_SEQ,
    COMMUNICATION_USB_RX_LEN,
    COMMUNICATION_USB_RX_PAYLOAD,
    COMMUNICATION_USB_RX_XOR
} CommunicationUsbRxState;

typedef struct
{
    CommunicationUsbRxState state;
    uint8_t id;
    uint8_t seq;
    uint8_t len;
    uint8_t index;
    uint8_t xor_sum;
    uint8_t payload[COMMUNICATION_USB_MAX_PAYLOAD];
} CommunicationUsbRxParser;

static uint8_t communication_usb_rx_ring[COMMUNICATION_USB_RX_RING_SIZE];
static volatile uint16_t communication_usb_rx_head;
static volatile uint16_t communication_usb_rx_tail;
static volatile uint32_t communication_usb_rx_overflow;

static CommunicationUsbRxParser communication_usb_parser;
static CommunicationUsbCmd communication_usb_cmd;
static CommunicationUsbDiag communication_usb_diag;
static uint32_t communication_usb_now_ms;
static uint32_t communication_usb_last_host_ms;
static uint8_t communication_usb_host_seen;
static uint8_t communication_usb_tx_seq;

/* ==========================================================================
 * 小端编解码
 * ========================================================================== */

static void communication_usb_put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void communication_usb_put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8) & 0xFFU);
    out[2] = (uint8_t)((value >> 16) & 0xFFU);
    out[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void communication_usb_put_i32(uint8_t *out, int32_t value)
{
    communication_usb_put_u32(out, (uint32_t)value);
}

static void communication_usb_put_u64(uint8_t *out, uint64_t value)
{
    communication_usb_put_u32(&out[0], (uint32_t)(value & 0xFFFFFFFFULL));
    communication_usb_put_u32(&out[4], (uint32_t)((value >> 32) & 0xFFFFFFFFULL));
}

static void communication_usb_put_f32(uint8_t *out, float value)
{
    uint32_t raw;

    (void)memcpy(&raw, &value, sizeof(raw));
    communication_usb_put_u32(out, raw);
}

static uint16_t communication_usb_get_u16(const uint8_t *in)
{
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static int32_t communication_usb_get_i32(const uint8_t *in)
{
    uint32_t raw = (uint32_t)in[0] |
                   ((uint32_t)in[1] << 8) |
                   ((uint32_t)in[2] << 16) |
                   ((uint32_t)in[3] << 24);
    return (int32_t)raw;
}

static uint64_t communication_usb_get_u64(const uint8_t *in)
{
    uint64_t low = (uint64_t)in[0] |
                   ((uint64_t)in[1] << 8) |
                   ((uint64_t)in[2] << 16) |
                   ((uint64_t)in[3] << 24);
    uint64_t high = (uint64_t)in[4] |
                    ((uint64_t)in[5] << 8) |
                    ((uint64_t)in[6] << 16) |
                    ((uint64_t)in[7] << 24);

    return low | (high << 32);
}

/* ==========================================================================
 * 接收环形缓冲（ISR 只入队，任务只出队）
 * ========================================================================== */

static void communication_usb_rx_push(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (length == 0U)) {
        return;
    }

    for (i = 0U; i < length; i++) {
        uint16_t head = communication_usb_rx_head;
        uint16_t next = (uint16_t)(head + 1U);

        if (next >= COMMUNICATION_USB_RX_RING_SIZE) {
            next = 0U;
        }
        if (next == communication_usb_rx_tail) {
            communication_usb_rx_overflow += (uint32_t)(length - i);
            break;
        }
        communication_usb_rx_ring[head] = data[i];
        __DMB();
        communication_usb_rx_head = next;
    }
}

static uint8_t communication_usb_rx_pop(uint8_t *out)
{
    uint16_t tail = communication_usb_rx_tail;

    if (tail == communication_usb_rx_head) {
        return 0U;
    }

    *out = communication_usb_rx_ring[tail];
    tail++;
    if (tail >= COMMUNICATION_USB_RX_RING_SIZE) {
        tail = 0U;
    }
    __DMB();
    communication_usb_rx_tail = tail;
    return 1U;
}

/* ==========================================================================
 * 帧解析
 * ========================================================================== */

static void communication_usb_reset_parser(void)
{
    (void)memset(&communication_usb_parser, 0, sizeof(communication_usb_parser));
    communication_usb_parser.state = COMMUNICATION_USB_RX_SOF0;
}

static void communication_usb_handle_frame(uint8_t id, uint8_t len, const uint8_t *payload)
{
    if ((id == COMMUNICATION_USB_ID_HOST_HEARTBEAT) && (len == 2U)) {
        communication_usb_host_seen = 1U;
        communication_usb_last_host_ms = communication_usb_now_ms;
        communication_usb_diag.rx_frames++;
        return;
    }

    if ((id == COMMUNICATION_USB_ID_UNIFIED_CMD) && (len == COMMUNICATION_USB_UNIFIED_CMD_LEN)) {
        CommunicationUsbCmd cmd;

        cmd.delta_yaw_udeg = communication_usb_get_i32(&payload[0]);
        cmd.delta_pitch_udeg = communication_usb_get_i32(&payload[4]);
        cmd.gimbal_status = communication_usb_get_u16(&payload[8]);
        cmd.vx_mm_s = communication_usb_get_i32(&payload[10]);
        cmd.vy_mm_s = communication_usb_get_i32(&payload[14]);
        cmd.wz_mdeg_s = communication_usb_get_i32(&payload[18]);
        cmd.chassis_mode = communication_usb_get_i32(&payload[22]);
        cmd.chassis_status = communication_usb_get_u16(&payload[26]);
        cmd.fire_on = payload[28];
        cmd.fire_mode = communication_usb_get_i32(&payload[29]);
        cmd.burst_count = communication_usb_get_i32(&payload[33]);
        cmd.host_ts_ns = communication_usb_get_u64(&payload[37]);
        cmd.rx_ms = communication_usb_now_ms;
        cmd.valid = 1U;

        communication_usb_cmd = cmd;
        communication_usb_host_seen = 1U;
        communication_usb_last_host_ms = communication_usb_now_ms;
        communication_usb_diag.rx_frames++;
        return;
    }

    communication_usb_diag.rx_errors++;
}

static void communication_usb_feed(uint8_t byte)
{
    CommunicationUsbRxParser *p = &communication_usb_parser;

    switch (p->state) {
        case COMMUNICATION_USB_RX_SOF0:
            if (byte == COMMUNICATION_USB_SOF0) {
                p->state = COMMUNICATION_USB_RX_SOF1;
            }
            break;

        case COMMUNICATION_USB_RX_SOF1:
            if (byte == COMMUNICATION_USB_SOF1) {
                p->state = COMMUNICATION_USB_RX_ID;
            } else if (byte != COMMUNICATION_USB_SOF0) {
                p->state = COMMUNICATION_USB_RX_SOF0;
            }
            break;

        case COMMUNICATION_USB_RX_ID:
            p->id = byte;
            p->xor_sum = byte;
            p->state = COMMUNICATION_USB_RX_SEQ;
            break;

        case COMMUNICATION_USB_RX_SEQ:
            p->seq = byte;
            p->xor_sum ^= byte;
            p->state = COMMUNICATION_USB_RX_LEN;
            break;

        case COMMUNICATION_USB_RX_LEN:
            p->len = byte;
            p->xor_sum ^= byte;
            p->index = 0U;
            if (p->len > COMMUNICATION_USB_MAX_PAYLOAD) {
                communication_usb_diag.rx_errors++;
                communication_usb_reset_parser();
            } else if (p->len == 0U) {
                p->state = COMMUNICATION_USB_RX_XOR;
            } else {
                p->state = COMMUNICATION_USB_RX_PAYLOAD;
            }
            break;

        case COMMUNICATION_USB_RX_PAYLOAD:
            p->payload[p->index++] = byte;
            p->xor_sum ^= byte;
            if (p->index >= p->len) {
                p->state = COMMUNICATION_USB_RX_XOR;
            }
            break;

        case COMMUNICATION_USB_RX_XOR:
            if (byte == p->xor_sum) {
                communication_usb_handle_frame(p->id, p->len, p->payload);
            } else {
                communication_usb_diag.rx_errors++;
            }
            communication_usb_reset_parser();
            break;

        default:
            communication_usb_reset_parser();
            break;
    }
}

/* ==========================================================================
 * 发送
 * ========================================================================== */

static void communication_usb_send_frame(uint8_t id, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[5U + COMMUNICATION_USB_MAX_PAYLOAD + 1U];
    uint8_t checksum;
    uint8_t i;
    uint16_t frame_len;

    if ((payload == NULL) || (len > COMMUNICATION_USB_MAX_PAYLOAD)) {
        communication_usb_diag.tx_errors++;
        return;
    }

    frame[0] = COMMUNICATION_USB_SOF0;
    frame[1] = COMMUNICATION_USB_SOF1;
    frame[2] = id;
    frame[3] = communication_usb_tx_seq++;
    frame[4] = len;
    (void)memcpy(&frame[5], payload, len);

    checksum = id ^ frame[3] ^ len;
    for (i = 0U; i < len; i++) {
        checksum ^= payload[i];
    }
    frame_len = (uint16_t)(5U + len);
    frame[frame_len] = checksum;
    frame_len++;

    if (bsp_cdc_write(frame, frame_len) == HAL_OK) {
        communication_usb_diag.tx_frames++;
    } else {
        communication_usb_diag.tx_errors++;
    }
}

/* ==========================================================================
 * 公开接口
 * ========================================================================== */

void communication_usb_init(void)
{
    communication_usb_rx_head = 0U;
    communication_usb_rx_tail = 0U;
    communication_usb_rx_overflow = 0U;
    communication_usb_reset_parser();
    (void)memset(&communication_usb_cmd, 0, sizeof(communication_usb_cmd));
    (void)memset(&communication_usb_diag, 0, sizeof(communication_usb_diag));
    communication_usb_last_host_ms = 0U;
    communication_usb_host_seen = 0U;
    communication_usb_tx_seq = 0U;
    communication_usb_diag.ready = 1U;

    bsp_cdc_on_rx(communication_usb_rx_push, NULL);
}

void communication_usb_poll(uint32_t now_ms)
{
    uint8_t byte;

    communication_usb_now_ms = now_ms;

    while (communication_usb_rx_pop(&byte) != 0U) {
        communication_usb_feed(byte);
    }

    communication_usb_diag.rx_overflow = communication_usb_rx_overflow;

    if ((communication_usb_host_seen != 0U) &&
        ((uint32_t)(now_ms - communication_usb_last_host_ms) >
         COMMUNICATION_USB_HOST_TIMEOUT_MS)) {
        communication_usb_host_seen = 0U;
        communication_usb_cmd.valid = 0U;
    }

    communication_usb_diag.link_online = communication_usb_host_seen;
}

void communication_usb_send_state(const CommunicationUsbState *state)
{
    uint8_t payload[COMMUNICATION_USB_MCU_STATE_LEN];

    if (state == NULL) {
        return;
    }

    communication_usb_put_u32(&payload[0], state->mcu_ts_ms);
    communication_usb_put_f32(&payload[4], state->imu_roll);
    communication_usb_put_f32(&payload[8], state->imu_pitch);
    communication_usb_put_f32(&payload[12], state->imu_yaw);
    communication_usb_put_f32(&payload[16], state->gimbal_yaw);
    communication_usb_put_f32(&payload[20], state->gimbal_pitch);
    communication_usb_put_f32(&payload[24], state->vx);
    communication_usb_put_f32(&payload[28], state->vy);
    communication_usb_put_f32(&payload[32], state->wz);
    payload[36] = state->mode;
    payload[37] = state->status;
    communication_usb_put_u16(&payload[38], state->error_flags);

    communication_usb_send_frame(COMMUNICATION_USB_ID_MCU_STATE,
                                 payload,
                                 (uint8_t)COMMUNICATION_USB_MCU_STATE_LEN);
}

void communication_usb_send_wheel_state(const CommunicationUsbWheelState *state)
{
    uint8_t payload[COMMUNICATION_USB_WHEEL_STATE_LEN];
    uint32_t i;

    if (state == NULL) {
        return;
    }

    communication_usb_put_u32(&payload[0], state->mcu_ts_ms);
    payload[4] = state->online_mask;
    payload[5] = state->mode;
    communication_usb_put_u16(&payload[6], state->chassis_status);
    communication_usb_put_u16(&payload[8], state->fault_flags);
    for (i = 0U; i < 4U; i++) {
        communication_usb_put_f32(&payload[10U + (4U * i)], state->speed_mps[i]);
        communication_usb_put_f32(&payload[26U + (4U * i)], state->position_rad[i]);
        payload[42U + i] = state->temp_mos_c[i];
        payload[46U + i] = state->temp_motor_c[i];
        communication_usb_put_u16(&payload[50U + (2U * i)], (uint16_t)state->torque_mnm[i]);
    }

    communication_usb_send_frame(COMMUNICATION_USB_ID_WHEEL_STATE,
                                 payload,
                                 (uint8_t)COMMUNICATION_USB_WHEEL_STATE_LEN);
}

void communication_usb_send_imu_state(const CommunicationUsbImuState *state)
{
    uint8_t payload[COMMUNICATION_USB_IMU_STATE_LEN];
    uint32_t i;

    if (state == NULL) {
        return;
    }

    communication_usb_put_u32(&payload[0], state->mcu_ts_ms);
    for (i = 0U; i < 3U; i++) {
        communication_usb_put_f32(&payload[4U + (4U * i)], state->gyro[i]);
        communication_usb_put_f32(&payload[16U + (4U * i)], state->accel[i]);
    }
    communication_usb_put_f32(&payload[28], state->temperature_c);
    for (i = 0U; i < 4U; i++) {
        communication_usb_put_f32(&payload[32U + (4U * i)], state->quat_wxyz[i]);
    }
    communication_usb_put_i32(&payload[48], state->imu_status);
    payload[52] = state->valid_mask;
    payload[53] = state->reserved;
    communication_usb_put_u32(&payload[54], state->last_ok_ms);

    communication_usb_send_frame(COMMUNICATION_USB_ID_IMU_STATE,
                                 payload,
                                 (uint8_t)COMMUNICATION_USB_IMU_STATE_LEN);
}

void communication_usb_send_heartbeat(uint32_t now_ms, uint8_t mode, uint8_t status)
{
    uint8_t payload[COMMUNICATION_USB_MCU_HEARTBEAT_LEN];

    communication_usb_put_u32(&payload[0], now_ms);
    payload[4] = mode;
    payload[5] = status;
    communication_usb_put_u16(&payload[6], 0U);

    communication_usb_send_frame(COMMUNICATION_USB_ID_MCU_HEARTBEAT,
                                 payload,
                                 (uint8_t)COMMUNICATION_USB_MCU_HEARTBEAT_LEN);
}

uint8_t communication_usb_get_cmd(CommunicationUsbCmd *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = communication_usb_cmd;
    __set_PRIMASK(primask);

    return out->valid;
}

uint8_t communication_usb_link_online(uint32_t now_ms)
{
    if (communication_usb_host_seen == 0U) {
        return 0U;
    }

    return ((uint32_t)(now_ms - communication_usb_last_host_ms) <=
            COMMUNICATION_USB_HOST_TIMEOUT_MS) ? 1U : 0U;
}

void communication_usb_get_diag(CommunicationUsbDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = communication_usb_diag;
    __set_PRIMASK(primask);
}
