/**
  ****************************(C) COPYRIGHT 2025****************************
  * @file       bsp_tfmini.c
  * @brief      TFmini Plus BSP driver (UART/IIC).
  *             TFmini Plus BSP驱动（UART/IIC）
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-21-2025     Codex           1. created
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025****************************
  */
#include "bsp_tfmini.h"
#include "bsp_dwt.h"
#include "bsp_tim24.h"
#include <string.h>

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    uint8_t frame[TFMINI_FRAME_LEN];
    uint8_t idx;
    uint8_t state;
    tfmini_measurement_t last;
    volatile uint32_t seq;
} tfmini_ctx_t;

static tfmini_ctx_t g_tfmini = {0};

/* ============ 内部工具函数 / Internal helpers ============ */
static uint64_t tfmini_now_us(void) {
#if (TFMINI_USE_TIM24_TIME == 1)
    return tim24_timebase_now_us();
#elif (TFMINI_USE_DWT_TIME == 1)
    return DWT_GetTimeline_us();
#else
    return ((uint64_t)HAL_GetTick()) * 1000ULL;
#endif
}

static uint8_t tfmini_checksum8(const uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    for(uint8_t i = 0; i < len; ++i) {
        sum = (uint8_t)(sum + buf[i]);
    }
    return sum;
}

static void tfmini_update_measurement(const uint8_t *frame) {
    tfmini_measurement_t m;
    uint16_t dist = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
    uint16_t strength = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
    uint16_t temp_raw = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
    int32_t temp_cdeg = ((int32_t)temp_raw * 100) / 8 - 25600;

    m.distance_cm = dist;
    m.strength = strength;
    m.temp_cdeg = (int16_t)temp_cdeg;
    m.valid = (strength >= 100u && strength != 0xFFFFu) ? 1u : 0u;
    m.ts_us = tfmini_now_us();

    g_tfmini.last = m;
    g_tfmini.seq++;
}

static void tfmini_parse_byte(uint8_t byte) {
    switch(g_tfmini.state) {
        case 0:
            if(byte == TFMINI_FRAME_HEAD) {
                g_tfmini.frame[0] = byte;
                g_tfmini.state = 1;
            }
            break;
        case 1:
            if(byte == TFMINI_FRAME_HEAD) {
                g_tfmini.frame[1] = byte;
                g_tfmini.idx = 2;
                g_tfmini.state = 2;
            } else {
                g_tfmini.state = 0;
            }
            break;
        case 2:
            g_tfmini.frame[g_tfmini.idx++] = byte;
            if(g_tfmini.idx >= TFMINI_FRAME_LEN) {
                uint8_t sum = tfmini_checksum8(g_tfmini.frame, 8);
                if(sum == g_tfmini.frame[8]) {
                    tfmini_update_measurement(g_tfmini.frame);
                }
                g_tfmini.state = 0;
            }
            break;
        default:
            g_tfmini.state = 0;
            break;
    }
}

/* ============ UART API ============ */
void tfmini_uart_init(UART_HandleTypeDef *huart) {
    if(!huart)
        return;
    memset(&g_tfmini, 0, sizeof(g_tfmini));
    g_tfmini.huart = huart;
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(huart, &g_tfmini.rx_byte, 1);
}

void tfmini_uart_rx_cplt_callback(UART_HandleTypeDef *huart) {
    if(!huart || g_tfmini.huart != huart)
        return;
    tfmini_parse_byte(g_tfmini.rx_byte);
    (void)HAL_UART_Receive_IT(huart, &g_tfmini.rx_byte, 1);
}

void tfmini_uart_error_callback(UART_HandleTypeDef *huart) {
    if(!huart || g_tfmini.huart != huart)
        return;
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(huart, &g_tfmini.rx_byte, 1);
}

bool tfmini_read(tfmini_measurement_t *out, uint32_t *io_seq) {
    if(!out || !io_seq)
        return false;
    uint32_t seq = g_tfmini.seq;
    if(seq == 0 || seq == *io_seq)
        return false;
    tfmini_measurement_t tmp = g_tfmini.last;
    if(seq != g_tfmini.seq) {
        seq = g_tfmini.seq;
        if(seq == 0 || seq == *io_seq)
            return false;
        tmp = g_tfmini.last;
    }
    *out = tmp;
    *io_seq = seq;
    return true;
}

bool tfmini_peek(tfmini_measurement_t *out) {
    if(!out)
        return false;
    if(g_tfmini.seq == 0)
        return false;
    *out = g_tfmini.last;
    return true;
}

void tfmini_reset_parser(void) {
    g_tfmini.idx = 0;
    g_tfmini.state = 0;
}

bool tfmini_uart_send_raw(const uint8_t *data, uint16_t len) {
    if(!data || len == 0 || !g_tfmini.huart)
        return false;
    return (HAL_UART_Transmit(g_tfmini.huart, (uint8_t *)data, len, 50) == HAL_OK);
}

#if (TFMINI_IIC_ENABLE == 1)
/* ============ Software IIC (minimal) ============ */
/* 用户需在工程中定义以下宏以匹配硬件引脚 / user should define these pins */
#ifndef TFMINI_IIC_SCL_PORT
#error "TFMINI_IIC_SCL_PORT not defined"
#endif
#ifndef TFMINI_IIC_SCL_PIN
#error "TFMINI_IIC_SCL_PIN not defined"
#endif
#ifndef TFMINI_IIC_SDA_PORT
#error "TFMINI_IIC_SDA_PORT not defined"
#endif
#ifndef TFMINI_IIC_SDA_PIN
#error "TFMINI_IIC_SDA_PIN not defined"
#endif

static void tfmini_iic_delay(void) {
    for(volatile uint32_t i = 0; i < 80; ++i) { __NOP(); }
}

static void tfmini_iic_scl(uint8_t level) {
    HAL_GPIO_WritePin(TFMINI_IIC_SCL_PORT, TFMINI_IIC_SCL_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void tfmini_iic_sda(uint8_t level) {
    HAL_GPIO_WritePin(TFMINI_IIC_SDA_PORT, TFMINI_IIC_SDA_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t tfmini_iic_sda_read(void) {
    return (uint8_t)(HAL_GPIO_ReadPin(TFMINI_IIC_SDA_PORT, TFMINI_IIC_SDA_PIN) == GPIO_PIN_SET);
}

static void tfmini_iic_start(void) {
    tfmini_iic_sda(1); tfmini_iic_scl(1); tfmini_iic_delay();
    tfmini_iic_sda(0); tfmini_iic_delay();
    tfmini_iic_scl(0); tfmini_iic_delay();
}

static void tfmini_iic_stop(void) {
    tfmini_iic_sda(0); tfmini_iic_scl(1); tfmini_iic_delay();
    tfmini_iic_sda(1); tfmini_iic_delay();
}

static uint8_t tfmini_iic_write_byte(uint8_t byte) {
    for(uint8_t i = 0; i < 8; ++i) {
        tfmini_iic_sda((byte & 0x80u) != 0u);
        tfmini_iic_delay();
        tfmini_iic_scl(1); tfmini_iic_delay();
        tfmini_iic_scl(0); tfmini_iic_delay();
        byte <<= 1;
    }
    tfmini_iic_sda(1); tfmini_iic_delay();
    tfmini_iic_scl(1); tfmini_iic_delay();
    uint8_t ack = (tfmini_iic_sda_read() == 0u);
    tfmini_iic_scl(0); tfmini_iic_delay();
    return ack;
}

static uint8_t tfmini_iic_read_byte(uint8_t ack) {
    uint8_t byte = 0;
    tfmini_iic_sda(1);
    for(uint8_t i = 0; i < 8; ++i) {
        tfmini_iic_scl(1); tfmini_iic_delay();
        byte = (uint8_t)((byte << 1) | (tfmini_iic_sda_read() & 0x01u));
        tfmini_iic_scl(0); tfmini_iic_delay();
    }
    tfmini_iic_sda(ack ? 0u : 1u);
    tfmini_iic_scl(1); tfmini_iic_delay();
    tfmini_iic_scl(0); tfmini_iic_delay();
    tfmini_iic_sda(1);
    return byte;
}

bool tfmini_iic_init(void) {
    /* 需要GPIO已配置为开漏输出+上拉 / GPIO should be open-drain with pull-up */
    return true;
}

bool tfmini_iic_read_measurement(tfmini_measurement_t *out) {
    if(!out)
        return false;
    uint8_t buf[TFMINI_FRAME_LEN];
    tfmini_iic_start();
    if(!tfmini_iic_write_byte((uint8_t)(TFMINI_IIC_ADDR << 1) | 0x01u)) {
        tfmini_iic_stop();
        return false;
    }
    for(uint8_t i = 0; i < TFMINI_FRAME_LEN; ++i) {
        buf[i] = tfmini_iic_read_byte(i < (TFMINI_FRAME_LEN - 1u));
    }
    tfmini_iic_stop();

    if(buf[0] != TFMINI_FRAME_HEAD || buf[1] != TFMINI_FRAME_HEAD)
        return false;
    if(tfmini_checksum8(buf, 8) != buf[8])
        return false;

    tfmini_update_measurement(buf);
    return tfmini_peek(out);
}
#endif
