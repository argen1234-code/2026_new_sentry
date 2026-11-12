/**
  * @file       bsp_odom_uart.c
  * @brief      外置里程计 UART7 接收（参考 工创底盘里程计例程）
  * @details    帧格式：A5 5A | x f32(mm) | y f32(mm) | yaw f32(deg) |
  *             gyro f32(dps) | body_vx f32(mm/s) | body_vy f32(mm/s) |
  *             crc16 LE（CRC-16/Modbus，覆盖字节 0..25）。
  *             DMA 缓冲与调试结构放 .odom_dma（RAM_D1）：H7 的 DMA 不能访问 DTCM。
  */

#include "bsp_odom_uart.h"

#include "usart.h"
#include "robot_param.h"

#include <math.h>
#include <string.h>

#define ODOM_DMA_ATTR __attribute__((section(".odom_dma"), aligned(32), used))

static uint8_t s_rx_buf[ODOM_RX_BUF_LEN] ODOM_DMA_ATTR;
volatile odom_dbg_t g_odom_dbg ODOM_DMA_ATTR;

static odom_state_t s_state;
static uint8_t s_origin_valid;
static float s_origin_x_mm;
static float s_origin_y_mm;
static float s_origin_yaw_rad;

/* 清零状态机：稳定等待 -> 下发 C3 -> 等数据稳定 -> 重置原点
 * 计时锚点由中断上下文（首帧解析）写入、任务上下文（tick）读取，必须 volatile。
 * s_zero_boot_ms == 0 表示「尚未收到首帧」，此时不清零。 */
static uint8_t s_zero_state;
static volatile uint32_t s_zero_boot_ms;
static volatile uint32_t s_zero_last_frame_ms;
static uint32_t s_zero_mark_ms;
static uint32_t s_zero_retry;
static uint32_t s_zero_last_seq;

static uint16_t odom_crc16(const uint8_t *p, uint16_t n)
{
    uint16_t c = 0xFFFFU;
    for (uint16_t i = 0U; i < n; i++)
    {
        c ^= p[i];
        for (uint8_t b = 0U; b < 8U; b++)
        {
            c = (c & 1U) ? (uint16_t)((c >> 1) ^ 0xA001U) : (uint16_t)(c >> 1);
        }
    }
    return c;
}

static float odom_read_float(const uint8_t *p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof(v));
    return v;
}

static float odom_angle_wrap(float a)
{
    while (a > PI)
    {
        a -= 2.0f * PI;
    }
    while (a < -PI)
    {
        a += 2.0f * PI;
    }
    return a;
}

/* 解析一段接收数据：取窗口内最后一个 CRC 通过的帧 */
static void odom_parse(const uint8_t *data, uint16_t len)
{
    if (len < ODOM_FRAME_LEN)
    {
        return;
    }

    uint16_t i = 0U;
    while ((uint16_t)(i + ODOM_FRAME_LEN) <= len)
    {
        if (data[i] != 0xA5U || data[i + 1U] != 0x5AU)
        {
            if (data[i] == 0xA5U)
            {
                g_odom_dbg.sync_miss++;
            }
            i++;
            continue;
        }

        const uint16_t rx = (uint16_t)data[i + 26U] | ((uint16_t)data[i + 27U] << 8);
        if (rx != odom_crc16(&data[i], 26U))
        {
            g_odom_dbg.crc_err++;
            i++;
            continue;
        }

        const float x_mm = odom_read_float(&data[i + 2U]);
        const float y_mm = odom_read_float(&data[i + 6U]);
        const float yaw_deg = odom_read_float(&data[i + 10U]);
        const float gyro_dps = odom_read_float(&data[i + 14U]);
        const float vx_mms = odom_read_float(&data[i + 18U]);
        const float vy_mms = odom_read_float(&data[i + 22U]);
        const float yaw_rad = yaw_deg * ODOM_DEG_TO_RAD;

        /* 安装点相对车体中心 +X 偏置，按清零后的相对航向旋转回中心 */
        if (s_origin_valid == 0U)
        {
            s_origin_yaw_rad = yaw_rad;
        }
        const float dyaw = odom_angle_wrap(yaw_rad - s_origin_yaw_rad);
        const float c = cosf(dyaw);
        const float s = sinf(dyaw);
        const float center_x_mm = x_mm - (c * ODOM_MOUNT_OFFSET_X_MM -
                                          s * ODOM_MOUNT_OFFSET_Y_MM);
        const float center_y_mm = y_mm - (s * ODOM_MOUNT_OFFSET_X_MM +
                                          c * ODOM_MOUNT_OFFSET_Y_MM);
        if (s_origin_valid == 0U)
        {
            s_origin_x_mm = center_x_mm;
            s_origin_y_mm = center_y_mm;
            s_origin_valid = 1U;
        }

        s_state.x_mm = center_x_mm - s_origin_x_mm;
        s_state.y_mm = center_y_mm - s_origin_y_mm;
        s_state.yaw_deg = yaw_deg;
        s_state.gyro_dps = gyro_dps;
        s_state.vx_mms = vx_mms;
        s_state.vy_mms = vy_mms;
        s_state.last_rx_ms = HAL_GetTick();
        /* 首帧到达才开始计时：模块比主控晚上电时不会被提前清零。 */
        if (s_zero_state == ODOM_ZERO_STATE_WAIT)
        {
            if (s_zero_boot_ms == 0U)
            {
                s_zero_boot_ms = s_state.last_rx_ms;
            }
            s_zero_last_frame_ms = s_state.last_rx_ms;
        }
        s_state.seq++;
        s_state.valid = true;

        g_odom_dbg.frames_ok++;
        g_odom_dbg.last_rx_ms = s_state.last_rx_ms;
        g_odom_dbg.last_x_mm = s_state.x_mm;
        g_odom_dbg.last_y_mm = s_state.y_mm;
        g_odom_dbg.last_yaw_deg = s_state.yaw_deg;
        g_odom_dbg.last_gyro_dps = s_state.gyro_dps;
        g_odom_dbg.last_vx_mms = s_state.vx_mms;
        g_odom_dbg.last_vy_mms = s_state.vy_mms;
        g_odom_dbg.valid = 1U;
        g_odom_dbg.seq = s_state.seq;

        i += ODOM_FRAME_LEN;
    }
}

static void odom_rearm(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, s_rx_buf, ODOM_RX_BUF_LEN) != HAL_OK)
    {
        g_odom_dbg.rearm_fail++;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART7)
    {
        return;
    }
    /* 半缓冲事件不是帧边界，DMA 仍在运行，不解析、不重挂 */
    if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    g_odom_dbg.rx_events++;
    g_odom_dbg.rx_bytes += Size;
    if (Size > 0U)
    {
        odom_parse(s_rx_buf, Size);
    }
    odom_rearm();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART7)
    {
        return;
    }
    g_odom_dbg.uart_errors++;
    odom_rearm();
}

/* 下发清零帧：A5 5A C3 00 | crc_lo crc_hi，CRC 覆盖前 4 字节 */
static void odom_send_zero(void)
{
    uint8_t frame[6] = {0xA5U, 0x5AU, 0xC3U, 0x00U, 0x00U, 0x00U};
    const uint16_t crc = odom_crc16(frame, 4U);

    frame[4] = (uint8_t)(crc & 0xFFU);
    frame[5] = (uint8_t)(crc >> 8);

    if (HAL_UART_Transmit(&huart7, frame, (uint16_t)sizeof(frame), 20U) == HAL_OK)
    {
        g_odom_dbg.zero_sent++;
    }
    else
    {
        g_odom_dbg.zero_fail++;
    }
}

void bsp_odom_uart_tick(void)
{
    const uint32_t now = HAL_GetTick();

    switch (s_zero_state)
    {
    case ODOM_ZERO_STATE_WAIT:
        /* 链路中断超过帧超时：把锚点后移，保证「连续 ODOM_ZERO_WAIT_MS 有数据」才清零，
         * 避免向刚好断掉的链路发清零帧。锚点为 0 表示还没收到首帧，继续等。 */
        if (s_zero_boot_ms != 0U &&
            (uint32_t)(now - s_zero_last_frame_ms) > ODOM_TIMEOUT_MS)
        {
            s_zero_boot_ms = now;
        }
        if (s_zero_boot_ms != 0U &&
            (uint32_t)(now - s_zero_boot_ms) >= ODOM_ZERO_WAIT_MS)
        {
            s_zero_last_seq = s_state.seq;
            odom_send_zero();
            s_zero_mark_ms = now;
            s_zero_state = ODOM_ZERO_STATE_SETTLE;
        }
        break;

    case ODOM_ZERO_STATE_SETTLE:
        if ((uint32_t)(now - s_zero_mark_ms) >= ODOM_ZERO_SETTLE_MS)
        {
            /* 清零只清 XY，不清航向：只失效原点，yaw 保持绝对角 */
            s_origin_valid = 0U;
            s_state.x_mm = 0.0f;
            s_state.y_mm = 0.0f;

            if (s_state.seq != s_zero_last_seq)
            {
                s_zero_state = ODOM_ZERO_STATE_DONE;
            }
            else if (s_zero_retry < ODOM_ZERO_MAX_RETRY)
            {
                s_zero_retry++;
                s_zero_last_seq = s_state.seq;
                odom_send_zero();
                s_zero_mark_ms = now;
            }
            else
            {
                g_odom_dbg.zero_fail++;
                s_zero_state = ODOM_ZERO_STATE_FAILED;
            }
        }
        break;

    default:
        break;
    }

    g_odom_dbg.zero_state = s_zero_state;
}

void bsp_odom_uart_init(void)
{
    memset((void *)&g_odom_dbg, 0, sizeof(g_odom_dbg));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    memset(&s_state, 0, sizeof(s_state));
    s_origin_valid = 0U;
    s_origin_x_mm = 0.0f;
    s_origin_y_mm = 0.0f;
    s_origin_yaw_rad = 0.0f;
    s_zero_state = ODOM_ZERO_STATE_WAIT;
    s_zero_boot_ms = 0U;
    s_zero_mark_ms = 0U;
    s_zero_retry = 0U;
    s_zero_last_seq = 0U;
    g_odom_dbg.magic = ODOM_DBG_MAGIC;

    /* 复用 sentinel 生成的 UART7 外设配置（GPIO/DMA/NVIC），再覆盖波特率 */
    MX_UART7_Init();
    huart7.Init.BaudRate = ODOM_UART_BAUD;
    if (HAL_UART_Init(&huart7) != HAL_OK)
    {
        g_odom_dbg.uart_errors++;
        return;
    }
    odom_rearm();
}

bool bsp_odom_uart_get(odom_state_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    *out = s_state;
    return s_state.valid;
}
