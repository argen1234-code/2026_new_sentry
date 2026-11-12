#ifndef __BSP_ODOM_UART_H__
#define __BSP_ODOM_UART_H__

#include "main.h"
#include "stdbool.h"

/* 里程计最新状态（原始单位：mm / deg / deg/s / mm/s） */
typedef struct
{
    float x_mm;       // 相对首帧原点的 X，单位 mm
    float y_mm;       // 相对首帧原点的 Y，单位 mm
    float yaw_deg;    // 绝对航向，单位 deg
    float gyro_dps;   // 角速度，单位 deg/s
    float vx_mms;     // 车体前向速度，单位 mm/s
    float vy_mms;     // 车体左向速度，单位 mm/s
    uint32_t last_rx_ms;
    uint32_t seq;
    bool valid;
} odom_state_t;

/* OpenOCD 可读的调试结构：非 static，放 RAM_D1，避免 D-Cache/ DTCM 问题 */
typedef struct
{
    volatile uint32_t magic;
    volatile uint32_t rx_events;   // 已处理的 IDLE/TC 事件数（HT 不计）
    volatile uint32_t rx_bytes;    // 累计接收字节数
    volatile uint32_t frames_ok;   // CRC 通过的帧数
    volatile uint32_t crc_err;     // 帧头对上但 CRC 错的帧数
    volatile uint32_t sync_miss;   // 看到 0xA5 但后一字节不是 0x5A 的次数
    volatile uint32_t uart_errors; // HAL 错误回调次数
    volatile uint32_t rearm_fail;  // 重新挂接 DMA 失败次数
    volatile uint32_t last_rx_ms;
    volatile float last_x_mm;
    volatile float last_y_mm;
    volatile float last_yaw_deg;
    volatile float last_gyro_dps;
    volatile float last_vx_mms;
    volatile float last_vy_mms;
    volatile uint32_t valid;
    volatile uint32_t seq;
    volatile uint32_t zero_state;  // 清零状态机当前状态
    volatile uint32_t zero_sent;   // 已成功下发的清零帧数
    volatile uint32_t zero_fail;   // 清零失败次数（含发送失败与重试超限）
} odom_dbg_t;

#define ODOM_DBG_MAGIC 0x4F444F4DU

/* 清零状态机状态（g_odom_dbg.zero_state） */
#define ODOM_ZERO_STATE_WAIT    0U // 上电稳定等待
#define ODOM_ZERO_STATE_SETTLE  1U // 已发清零帧，等待数据稳定
#define ODOM_ZERO_STATE_DONE    2U // 清零完成
#define ODOM_ZERO_STATE_FAILED  3U // 重试超限，已放弃

extern volatile odom_dbg_t g_odom_dbg;

void bsp_odom_uart_init(void);
void bsp_odom_uart_tick(void);
bool bsp_odom_uart_get(odom_state_t *out);

#endif
