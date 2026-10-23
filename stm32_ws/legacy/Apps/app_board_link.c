/**
 ******************************************************************************
 * @file    app_board_link.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   板间通信应用层实现。
 *
 * @note    当前 target 默认不启用本任务；启用后必须在 Robot map 中配置独立
 *          UART，不能复用裁判系统或 host 通道。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_board_link.h"

#include "app_board_link_config.h"
#include "app_gimbal.h"
#include "app_observe.h"
#include "app_shoot.h"
#include "bsp_timebase.h"
#include "main.h"
#include "cmsis_os.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 毫秒转 OS tick，向上取整。 */
#define APP_BOARD_LINK_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/** @brief 毫秒转微秒。 */
#define APP_BOARD_LINK_MS_TO_US(ms) ((uint32_t)((uint32_t)(ms) * 1000U))

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 板间通信 glue 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief BoardLink 任务控制块。 */
static osThreadId_t app_board_link_thread;

/** @brief BoardLink 任务属性。 */
static const osThreadAttr_t app_board_link_attr = {
    .name = "app_board",
    .stack_size = APP_BOARD_LINK_STACK_SIZE,
    .priority = (osPriority_t)APP_BOARD_LINK_PRIORITY,
};

/** @brief BoardLink 诊断缓存。 */
static AppBoardLinkDiag app_board_link_diag;

/** @brief 最新底盘状态缓存。 */
static CommunicationChassisState app_board_link_chassis;

/** @brief 底盘状态缓存有效标志。 */
static uint8_t app_board_link_chassis_valid;

/* ==========================================================================
 * 临界区与时间工具
 * --------------------------------------------------------------------------
 * 短临界区只保护快照复制和轻量状态切换，避免任务读写同一缓存时撕裂。
 * ========================================================================== */
/**
 * @brief 进入临界区。
 *
 * @return 进入前 PRIMASK。
 */
static uint32_t app_board_link_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区。
 *
 * @param primask 进入前 PRIMASK。
 */
static void app_board_link_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 判断周期事件是否到期。
 *
 * @param now_us 当前时间戳低 32 位。
 * @param last_us 上次触发时间戳。
 * @param period_ms 周期，单位 ms。
 * @return 到期返回 1，否则返回 0。
 */
static uint8_t app_board_link_due(uint32_t now_us, uint32_t *last_us, uint32_t period_ms)
{
    uint32_t period_us;

    if (last_us == NULL) {
        return 0U;
    }

    period_us = APP_BOARD_LINK_MS_TO_US(period_ms);
    if ((*last_us == 0U) || ((uint32_t)(now_us - *last_us) >= period_us)) {
        *last_us = now_us;
        return 1U;
    }

    return 0U;
}

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/**
 * @brief 生成并发送云台板状态。
 *
 * @param now_us 当前时间戳低 32 位。
 */
static void app_board_link_send_state(uint32_t now_us)
{
    CommunicationBoardState state;
    ObserverState observe;
    GimbalInput gimbal_input;
    AppGimbalDiag gimbal_diag;
    AppShootDiag shoot_diag;
    CommunicationStatus status;
    uint32_t primask;

    (void)memset(&state, 0, sizeof(state));
    if ((app_observe_get_state(&observe) == 0U) ||
        (app_gimbal_get_input(&gimbal_input) == 0U) ||
        (app_gimbal_get_diag(&gimbal_diag) == 0U)) {
        status = CommunicationState;
    } else {
        state.yaw_motor_rad = gimbal_input.ins_yaw;
        state.ins_yaw_rad = observe.ins.yaw_total;
        state.ins_pitch_rad = observe.ins.euler.pitch;
        state.ins_roll_rad = observe.ins.euler.roll;
        state.gimbal_mode = (uint8_t)gimbal_diag.mode;
        if (app_shoot_get_diag(&shoot_diag) != 0U) {
            state.shoot_mode = (uint8_t)shoot_diag.mode;
        }
        state.status = (uint8_t)((gimbal_diag.input_online != 0U) ? 0x01U : 0U);
        if (gimbal_diag.observe_ready != 0U) {
            state.status |= 0x02U;
        }
        if (gimbal_diag.send_enable != 0U) {
            state.status |= 0x04U;
        }
        state.time_us = now_us;
        status = communication_send_board_state(&state);
    }

    primask = app_board_link_enter();
    app_board_link_diag.last_status = status;
    if (status == CommunicationOk) {
        app_board_link_diag.state_send_count++;
    } else {
        app_board_link_diag.state_fail_count++;
    }
    app_board_link_exit(primask);
}

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * BoardLink 负责把 App 快照转换为板间状态帧，Communication 仍只保存协议缓存。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/**
 * @brief 读取并缓存底盘板状态。
 */
static void app_board_link_poll_chassis(void)
{
    CommunicationChassisState state;
    uint32_t primask;

    if (communication_get_chassis_state(&state) != CommunicationOk) {
        return;
    }
    if ((state.seq == 0U) || (state.seq == app_board_link_diag.last_chassis_seq)) {
        return;
    }

    primask = app_board_link_enter();
    app_board_link_chassis = state;
    app_board_link_chassis_valid = 1U;
    app_board_link_diag.last_chassis_seq = state.seq;
    app_board_link_diag.chassis_rx_count++;
    app_board_link_exit(primask);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief BoardLink 任务入口。
 *
 * @param argument CMSIS-RTOS2 参数。
 */
static void app_board_link_entry(void *argument)
{
    uint32_t now_us;
    uint32_t last_state_us = 0U;
    uint32_t primask;

    (void)argument;

    while (1) {
        now_us = (uint32_t)bsp_timebase_now_us();
        app_board_link_poll_chassis();
        if (app_board_link_due(now_us, &last_state_us, APP_BOARD_LINK_STATE_PERIOD_MS) != 0U) {
            app_board_link_send_state(now_us);
        }

        primask = app_board_link_enter();
        app_board_link_diag.loop_count++;
        app_board_link_diag.last_time_us = now_us;
        app_board_link_exit(primask);
        osDelay(APP_BOARD_LINK_MS_TO_TICKS(APP_BOARD_LINK_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_board_link_init(void)
{
    (void)memset(&app_board_link_diag, 0, sizeof(app_board_link_diag));
    (void)memset(&app_board_link_chassis, 0, sizeof(app_board_link_chassis));
    app_board_link_chassis_valid = 0U;
    app_board_link_diag.ready = 1U;

    app_board_link_thread = osThreadNew(app_board_link_entry, NULL, &app_board_link_attr);
    return (app_board_link_thread != NULL) ? 1U : 0U;
}

uint8_t app_board_link_get_chassis(CommunicationChassisState *out)
{
    uint32_t primask;
    uint8_t valid;

    if (out == NULL) {
        return 0U;
    }

    primask = app_board_link_enter();
    valid = app_board_link_chassis_valid;
    if (valid != 0U) {
        *out = app_board_link_chassis;
    }
    app_board_link_exit(primask);
    return valid;
}

uint8_t app_board_link_get_diag(AppBoardLinkDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_board_link_enter();
    *out = app_board_link_diag;
    app_board_link_exit(primask);
    return 1U;
}
