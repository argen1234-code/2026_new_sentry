/**
 * @file       comm_app.c
 * @brief      设备侧通信应用入口（FreeRTOS 任务）
 * @details    对接 core 层 channel，新版采用“配置优先”的初始化风格：
 *             - time_sync 通道：提供设备/主机时间映射能力
 *             - camera 通道：相机触发事件上报（含可选 GPIO 轮询）
 *             - gimbal 通道：发布云台状态、接收主机增量命令
 *             并通过 ch_uproto_arbiter 实现统一的发送仲裁。
 */
#include "comm_app.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"

#include "usart.h"
#include "bsp_tfmini.h"
#include "../../channel/camera/camera_channel.h"
#include "../../channel/camera/camera_config.h"
#include "../../channel/gimbal/gimbal_channel.h"
#include "../../channel/gimbal/gimbal_config.h"
#include "../../channel/time_sync/time_sync_channel.h"
#include "../../channel/time_sync/time_sync_config.h"
#include "../../core/comm.h"
#include "../../core/platform.h"
#include "../../core/uproto.h"
#include "../shared/protocol_ids.h" /* 引入统一的 MUX 消息类型与通道号 */
#include "INS_task.h"
#include "bsp_dwt.h"
#include "bsp_tim24.h"
#include "gimbal_task.h"
#include "bsp_usart.h"   /* gimbal_data_t gimbal */
#include "bsp_usb.h"     /* usb_send_referee_data */

/* 任务配置与通道参数统一放在 comm_app_config.h 中（见 comm_app.h） */

#ifndef COMM_PROTO_ENABLE
#define COMM_PROTO_ENABLE 1
#endif

/* 复用公共协议 ID：UPROTO_MSG_MUX、CAM_CH_ID、TS_CH_ID、GIMBAL_CH_ID 等
 * 自定义演示流通道的 SID 在 comm_app_config.h 中定义 */

static osThreadId g_comm_app_tid = NULL;
extern uproto_context_t proto_ctx;

static channel_manager_t g_mgr;
static ch_uproto_bind_t g_bind;
static camera_channel_t g_camera;
static gimbal_channel_t g_gimbal;
static time_sync_channel_t g_tsync;

#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
static uint32_t g_tfmini_seq = 0;
static uint64_t g_tfmini_last_pub_us = 0;
#endif

/**
 * @brief          备用时间源（微秒）
 * @details        当 DWT 或高精度计时不可用时，用 HAL_GetTick 近似换算
 * @retval         当前时间戳（微秒）
 */
static uint64_t now_us_fallback(void) {
    return ((uint64_t)HAL_GetTick()) * 1000ULL;
}

/**
 * @brief          从时间同步通道获取当前设备时间（微秒）
 * @param[in]      user：保留（未使用）
 * @retval         当前设备时间戳（微秒）
 */
static uint64_t sync_now_us(void *user) {
    (void)user;
    return time_sync_channel_now_us(&g_tsync);
}

/* forward decls for gimbal channel hooks */
static bool gimbal_get_state(gimbal_state_t *out, void *user);
static void gimbal_on_delta(const gimbal_delta_t *d, void *user);
#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
static void tfmini_tick(void);
#endif

/**
 * @brief          自定义流通道的接收回调
 * @details        将收到的 payload 以相同 SID 回环到主机，用于串口/带宽测试
 * @param[in]      ch：通道对象
 * @param[in]      payload：数据指针
 * @param[in]      len：数据长度
 * @param[in]      user：用户上下文（未使用）
 */
static void stream_on_rx(channel_t *ch, const uint8_t *payload, uint32_t len, void *user) {
    (void)ch;
    (void)user;
    if(!payload || len == 0)
        return;
    (void)ch_uproto_send_notify(&g_bind, COMM_APP_STREAM_CH_ID, COMM_APP_STREAM_SID_DATA, 0, payload, (uint16_t)len);
}

/**
 * @brief          触发一次相机事件
 * @details        供外部 EXTI 中断或其他模块调用，通知 camera 通道上报一次事件
 */
void comm_camera_trigger_pulse(void) {
    camera_channel_trigger(&g_camera);
}

/**
 * @brief          初始化并注册所有通道
 * @details        使用“配置优先”的 EX 初始化接口，显式传入通道参数与时间源
 */
static void setup_channels(void) {
    chmgr_init(&g_mgr);

    /* time sync channel (config-first) */
    ts_time_ops_t ts_ops = {.now_us = (uint64_t (*)(void *))tim24_timebase_now_us, .user = NULL};
    ts_channel_cfg_t ts_cfg = {.ch_id = TS_CH_ID, .period_ms = TS_PERIOD_MS, .initiator = TS_INITIATOR, .priority = TS_PRIORITY, .max_rtt_us = TS_MAX_RTT_US};
    time_sync_channel_init_ex(&g_tsync, &g_bind, &g_mgr, &ts_cfg, &ts_ops);

    /* camera channel (config-first) */
    camera_channel_cfg_t cam_cfg = {.ch_id = CAM_CH_ID, .priority = CAM_PRIORITY};
    camera_channel_init_ex(&g_camera, &g_bind, &g_mgr, &cam_cfg, sync_now_us, NULL, NULL);

    /* gimbal channel (config-first) - 恢复云台状态发布 */
    gimbal_source_ops_t gsrc = {.get_state = gimbal_get_state, .user = NULL, .period_ms = GIMBAL_PUB_PERIOD_MS};
    gimbal_hooks_t ghk = {.on_delta = gimbal_on_delta, .user = NULL};
    gimbal_channel_cfg_t gcfg = {.ch_id = GIMBAL_CH_ID, .priority = GIMBAL_PRIORITY, .period_ms = 0u};
    gimbal_channel_init_ex(&g_gimbal, &g_bind, &g_mgr, &gcfg, &gsrc, &ghk, sync_now_us, NULL);

    /* stream channel: id=2, default priority=1 */
    static channel_t stream_ch;
    channel_cfg_t st_cfg = {.mode = CH_MODE_QA, .reliable = 0, .expect_reply = 0, .priority = 1, .period_ms = 0};
    channel_hooks_t st_hooks = {.on_rx = stream_on_rx, .on_tick = NULL, .on_cfg_change = NULL};
    channel_init(&stream_ch, COMM_APP_STREAM_CH_ID, &st_cfg, &st_hooks, NULL);
    channel_bind_transport(&stream_ch, &g_bind, UPROTO_MSG_MUX);
    (void)chmgr_register(&g_mgr, &stream_ch);
}

#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
/**
 * @brief          Publish TFmini range data (device -> host)
 * @details        Read UART parsed data and pack into gimbal channel
 */
/**
 * @brief          TFmini测距数据发布（设备 → 主机）
 * @details        读取UART解析结果并打包到gimbal通道
 */
static void tfmini_tick(void) {
    uint64_t now = sync_now_us(NULL);
    if(g_tfmini_last_pub_us != 0 &&
       (now - g_tfmini_last_pub_us) < (uint64_t)TFMINI_PUB_PERIOD_MS * 1000ULL) {
        return;
    }
    tfmini_measurement_t m;
    if(!tfmini_read(&m, &g_tfmini_seq))
        return;
    gimbal_tfmini_t msg;
    msg.distance_cm = m.distance_cm;
    msg.strength = m.strength;
    msg.temp_cdeg = m.temp_cdeg;
    msg.status = m.valid ? GIMBAL_TFMINI_STATUS_VALID : 0u;
    msg.ts_us = m.ts_us ? m.ts_us : now;
    (void)gimbal_channel_publish_tfmini(&g_gimbal, &msg);
    g_tfmini_last_pub_us = now;
}
#endif

/**
 * @brief          裁判系统数据发布（设备 → 主机）
 * @details        将 gimbal_data_t 中的裁判系统数据打包为 GIMBAL_SID_REFEREE 消息发送，周期 100ms
 * @note           已注释，改用简化版 usb_send_referee_data()
 */
/*
static void referee_tick(void)
{
    static uint64_t last_pub = 0;
    uint64_t now = sync_now_us(NULL);
    if (last_pub != 0 && (now - last_pub) < 100000ULL)
        return;
    last_pub = now;

    uint8_t buf[16];
    uint16_t pos = 0;
    uint64_t ts = now;
    int i;

    buf[pos++] = gimbal_data.game_progress;
    buf[pos++] = (uint8_t)(gimbal_data.stage_remain_time & 0xFFu);
    buf[pos++] = (uint8_t)(gimbal_data.stage_remain_time >> 8u);
    buf[pos++] = gimbal_data.game_result;
    buf[pos++] = (uint8_t)(gimbal_data.current_HP & 0xFFu);
    buf[pos++] = (uint8_t)(gimbal_data.current_HP >> 8u);
    buf[pos++] = (uint8_t)(gimbal_data.shooter_42mm_heat & 0xFFu);
    buf[pos++] = (uint8_t)(gimbal_data.shooter_42mm_heat >> 8u);
    for (i = 0; i < 8; i++) { buf[pos++] = (uint8_t)(ts & 0xFFu); ts >>= 8u; }

    (void)ch_uproto_send_notify(&g_bind, GIMBAL_CH_ID, GIMBAL_SID_REFEREE, 0, buf, pos);
}
*/

#if defined(CAM_TRIGGER_ENABLE) && (CAM_TRIGGER_ENABLE == 1)
/**
 * @brief          可选 GPIO 轮询触发
 * @details        当开启 CAM_TRIGGER_ENABLE 时，在任务循环内轮询 GPIO 边沿并触发事件
 */
void comm_camera_trigger_poll(void) {
    static uint8_t prev = 0xFFu;
    uint8_t state = 0u;
    GPIO_PinState s = HAL_GPIO_ReadPin(CAM_TRIGGER_GPIO_PORT, CAM_TRIGGER_GPIO_PIN);
    state = (s == GPIO_PIN_SET) ? 1u : 0u;
    if(prev == 0xFFu) {
        prev = state;
        return;
    }
#if (CAM_TRIGGER_ACTIVE_HIGH == 1)
    if(prev == 0u && state == 1u) {
        comm_camera_trigger_pulse();
    }
#else
    if(prev == 1u && state == 0u) {
        comm_camera_trigger_pulse();
    }
#endif
    prev = state;
}
#endif

/**
 * @brief          通信应用任务入口（FreeRTOS）
 * @param[in]      arg：任务参数（未使用）
 * @details        等待 USB CDC 枚举 → 绑定 uproto 与通道 → 周期性调度
 */
void comm_app_task(void const *arg) {
    (void)arg;
    extern USBD_HandleTypeDef hUsbDeviceHS;
		MX_USB_DEVICE_Init();
#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
    /* Init TFmini RX early; do not wait for USB enumeration. */
    tfmini_uart_init(&huart10);
#endif

    uint32_t t0 = HAL_GetTick();
    while((hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) && ((HAL_GetTick() - t0) < COMM_APP_USB_ENUM_TIMEOUT_MS)) {
        osDelay(1);
    }
    ch_uproto_bind(&g_bind, &proto_ctx, UPROTO_MSG_MUX, &g_mgr);
    usb_cdc_port_bind_uproto(&proto_ctx);
    ch_uproto_register_rx(&g_bind);
    setup_channels();
    for(;;) {
#if COMM_PROTO_ENABLE
        uproto_tick(&proto_ctx);
        camera_channel_tick(&g_camera);
        ch_uproto_arbiter_tick(&g_bind);
        chmgr_tick(&g_mgr);
#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
        tfmini_tick();
#endif
        // referee_tick();  /* 已改用简化版 usb_send_referee_data() */
#endif
#if defined(CAM_TRIGGER_ENABLE) && (CAM_TRIGGER_ENABLE == 1)
        comm_camera_trigger_poll();
#endif
        /* 裁判数据已通过 GIMBAL_STATE_PACK_EXT 宏附加到云台状态数据中，每 10ms 随云台数据一起发送 */
        osDelay(COMM_APP_LOOP_DELAY_MS);
    }
}

/**
 * @brief          启动通信应用任务
 * @details        创建并启动 FreeRTOS 任务（防止重复创建）
 */
void comm_app_start(void) {
    if(g_comm_app_tid != NULL)
        return;
    osThreadDef(COMM_APP, (os_pthread)comm_app_task, COMM_APP_PRIO, 0, COMM_APP_STACK);
    g_comm_app_tid = osThreadCreate(osThread(COMM_APP), NULL);
}

/* gimbal integration: real implementations */
/**
 * @brief          云台状态获取回调（设备 → 主机）
 * @param[out]     out：输出状态
 * @param[in]      user：用户上下文（未使用）
 * @retval         true：成功获取；false：不可用
 * @note           示例使用已有的 gimbal/INS 数据源进行单位换算到“微度”
 */
static bool gimbal_get_state(gimbal_state_t *out, void *user) {
    (void)user;
    if(!out)
        return false;
    const gimbal_motor_t *yaw = get_yaw_motor_point();
    const gimbal_motor_t *pit = get_pitch_motor_point();
    fp32 *imu = get_INS_angle_point();
    const float INV_PI = 0.31830988618379067154f; /* 1/pi */
    if(yaw && pit) {
        out->enc_yaw = (int32_t)(yaw->radian_of_ecd * 180000000.0f * INV_PI);
        out->enc_pitch = (int32_t)(pit->radian_of_ecd * 180000000.0f * INV_PI);
    } else {
        out->enc_yaw = 0;
        out->enc_pitch = 0;
    }
    if(imu) {
        out->yaw_udeg = (int32_t)(imu[0] * 180000000.0f * INV_PI);
        out->pitch_udeg = (int32_t)(imu[1] * 180000000.0f * INV_PI);
        out->roll_udeg = (int32_t)(imu[2] * 180000000.0f * INV_PI);
    } else {
        out->yaw_udeg = out->pitch_udeg = out->roll_udeg = 0;
    }
    out->ts_us = sync_now_us(NULL);
    return true;
}

/**
 * @brief          云台增量命令回调（主机 → 设备）
 * @param[in]      d：增量命令
 * @param[in]      user：用户上下文（未使用）
 * @details        将最新命令写入邮箱，供云台控制任务消费
 */
static void gimbal_on_delta(const gimbal_delta_t *d, void *user) {
    (void)user;
    if(!d)
        return;
    gimbal_cmd_t cmd;
    cmd.delta_yaw_udeg = d->delta_yaw_udeg;
    cmd.delta_pitch_udeg = d->delta_pitch_udeg;
    cmd.status = d->status;
    cmd.ts_us = d->ts_us;
    cmd.version = 0;
    gimbal_mailbox_set(&cmd);
}
