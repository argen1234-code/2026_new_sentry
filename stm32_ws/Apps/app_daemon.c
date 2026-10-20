/**
 ******************************************************************************
 * @file    app_daemon.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Daemon 应用层守护线程实现。
 *
 * @note    本文件只维护健康表、在线离线边沿事件和诊断快照。
 * @note    本文件不修改任何控制输出，也不向 Controller 注入安全状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_daemon.h"

#include "app_daemon_config.h"
#if ROBOT_APP_ENABLE_CHASSIS != 0U
#include "app_chassis.h"
#endif
#if ROBOT_APP_ENABLE_COMMUNICATION != 0U
#include "app_communication.h"
#endif
#if ROBOT_APP_ENABLE_GIMBAL != 0U
#include "app_gimbal.h"
#endif
#if ROBOT_APP_ENABLE_INPUT != 0U
#include "app_input.h"
#endif
#if ROBOT_APP_ENABLE_OBSERVE != 0U
#include "app_observe.h"
#endif
#if ROBOT_APP_ENABLE_REFEREE != 0U
#include "app_referee.h"
#endif
#if ROBOT_APP_ENABLE_SHOOT != 0U
#include "app_shoot.h"
#endif
#include "bsp_timebase.h"
#include "main.h"
#include "rc.h"
#include "tfmini.h"

#include "cmsis_os.h"

#include <string.h>

#if APP_DAEMON_EVENT_MAX == 0U
#error "APP_DAEMON_EVENT_MAX must be greater than 0"
#endif

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_DAEMON_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/** @brief Shoot mono 构型需要的摩擦轮在线掩码。 */
#define APP_DAEMON_SHOOT_FRIC_MONO_MASK ((uint8_t)0x03U)

/** @brief Shoot double 构型需要的摩擦轮在线掩码。 */
#define APP_DAEMON_SHOOT_FRIC_DOUBLE_MASK ((uint8_t)0x0FU)

/** @brief Chassis 四轮反馈在线掩码。 */
#define APP_DAEMON_CHASSIS_4_MASK ((uint8_t)0x0FU)

/**
 * @brief 单项健康配置。
 */
typedef struct
{
    uint8_t enable;      /**< 非 0 表示参与健康检查。 */
    uint32_t offline_ms; /**< 离线超时时间，单位 ms。 */
    uint32_t online_ms;  /**< 恢复稳定时间，单位 ms。 */
    uint8_t priority;    /**< 错误优先级。 */
} AppDaemonItemCfg;

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 健康守护 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Daemon 线程控制块。 */
static osThreadId_t app_daemon_thread;

/** @brief Daemon 线程属性。 */
static const osThreadAttr_t app_daemon_attr = {
    .name = "app_daemon",
    .stack_size = APP_DAEMON_STACK_SIZE,
    .priority = (osPriority_t)APP_DAEMON_PRIORITY,
};

/** @brief Daemon 健康表和汇总状态。 */
static AppDaemonState app_daemon_state;

/** @brief Daemon 诊断快照。 */
static AppDaemonDiag app_daemon_diag;

/** @brief Daemon 事件 ring。 */
static AppDaemonEvent app_daemon_events[APP_DAEMON_EVENT_MAX];

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Daemon 集中维护在线/离线边沿和最高优先级错误，但不接管各 App 的安全清零。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/** @brief 事件 ring 读索引。 */
static uint16_t app_daemon_event_read;

/** @brief 事件 ring 写索引。 */
static uint16_t app_daemon_event_write;

/** @brief 事件 ring 当前元素数量。 */
static uint16_t app_daemon_event_count;

/**
 * @brief 获取当前毫秒时间戳。
 *
 * @return 当前时间低 32 位，单位 ms。
 */
static uint32_t app_daemon_now_ms(void)
{
    return (uint32_t)(bsp_timebase_now_us() / 1000ULL);
}

/* ==========================================================================
 * 临界区与时间工具
 * --------------------------------------------------------------------------
 * 短临界区只保护快照复制和轻量状态切换，避免任务读写同一缓存时撕裂。
 * ========================================================================== */
/**
 * @brief 保存并关闭全局中断。
 *
 * @return 进入临界区前的 PRIMASK。
 */
static uint32_t app_daemon_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 按保存的 PRIMASK 恢复中断状态。
 *
 * @param primask 进入临界区前的 PRIMASK。
 */
static void app_daemon_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 写入一条事件。
 *
 * 调用者必须已经进入临界区。
 *
 * @param item 健康项。
 * @param kind 事件类型。
 * @param tick_ms 事件时间戳，单位 ms。
 */
static void app_daemon_push_event(AppDaemonItem item, AppDaemonEventKind kind, uint32_t tick_ms)
{
    AppDaemonEvent *event;

    if ((item >= AppDaemonItemCount) || (kind == AppDaemonEventNone)) {
        return;
    }

    if (app_daemon_event_count >= (uint16_t)APP_DAEMON_EVENT_MAX) {
        app_daemon_diag.event_drop_count++;
        return;
    }

    event = &app_daemon_events[app_daemon_event_write];
    event->kind = kind;
    event->item = item;
    event->tick_ms = tick_ms;
    event->priority = app_daemon_state.item[item].priority;

    app_daemon_event_write++;
    if (app_daemon_event_write >= (uint16_t)APP_DAEMON_EVENT_MAX) {
        app_daemon_event_write = 0U;
    }
    app_daemon_event_count++;
    app_daemon_state.event_seq++;
    app_daemon_diag.event_write_count++;
}

/* ==========================================================================
 * 配置生成与参数展开
 * --------------------------------------------------------------------------
 * 本区把 Robot target 宏展开成 Component/Device 的 Cfg，运行期控制逻辑不写在配置头里。
 * ========================================================================== */
/**
 * @brief 填充默认健康项配置表。
 *
 * @param cfg 健康配置输出。
 */
static void app_daemon_make_cfg(AppDaemonItemCfg cfg[AppDaemonItemCount])
{
    if (cfg == NULL) {
        return;
    }

    (void)memset(cfg, 0, sizeof(AppDaemonItemCfg) * (uint32_t)AppDaemonItemCount);

    cfg[AppDaemonRcDbus] = (AppDaemonItemCfg){1U,
                                             ROBOT_APP_DAEMON_RC_OFFLINE_MS,
                                             ROBOT_APP_DAEMON_RC_ONLINE_MS,
                                             ROBOT_APP_DAEMON_RC_PRIORITY};
    cfg[AppDaemonReferee] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_REFEREE != 0U),
                                               ROBOT_APP_DAEMON_REFEREE_OFFLINE_MS,
                                               ROBOT_APP_DAEMON_REFEREE_ONLINE_MS,
                                               ROBOT_APP_DAEMON_REFEREE_PRIORITY};
    cfg[AppDaemonCommHost] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_COMMUNICATION != 0U),
                                                ROBOT_APP_DAEMON_COMM_OFFLINE_MS,
                                                ROBOT_APP_DAEMON_COMM_ONLINE_MS,
                                                ROBOT_APP_DAEMON_COMM_PRIORITY};
    cfg[AppDaemonCommVision] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_COMMUNICATION != 0U),
                                                  ROBOT_APP_DAEMON_VISION_OFFLINE_MS,
                                                  ROBOT_APP_DAEMON_VISION_ONLINE_MS,
                                                  ROBOT_APP_DAEMON_VISION_PRIORITY};
    cfg[AppDaemonObserveImu] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_OBSERVE != 0U),
                                                 ROBOT_APP_DAEMON_IMU_OFFLINE_MS,
                                                 ROBOT_APP_DAEMON_IMU_ONLINE_MS,
                                                 ROBOT_APP_DAEMON_IMU_PRIORITY};
    cfg[AppDaemonObserveIns] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_OBSERVE != 0U),
                                                 ROBOT_APP_DAEMON_IMU_OFFLINE_MS,
                                                 ROBOT_APP_DAEMON_IMU_ONLINE_MS,
                                                 ROBOT_APP_DAEMON_IMU_PRIORITY};
    cfg[AppDaemonGimbalYaw] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_GIMBAL != 0U),
                                                ROBOT_APP_DAEMON_GIMBAL_YAW_OFFLINE_MS,
                                                ROBOT_APP_DAEMON_GIMBAL_YAW_ONLINE_MS,
                                                ROBOT_APP_DAEMON_GIMBAL_YAW_PRIORITY};
    cfg[AppDaemonGimbalPitch] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_GIMBAL != 0U),
                                                  ROBOT_APP_DAEMON_GIMBAL_PITCH_OFFLINE_MS,
                                                  ROBOT_APP_DAEMON_GIMBAL_PITCH_ONLINE_MS,
                                                  ROBOT_APP_DAEMON_GIMBAL_PITCH_PRIORITY};
    cfg[AppDaemonShootTrigger] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_SHOOT != 0U),
                                                   ROBOT_APP_DAEMON_SHOOT_OFFLINE_MS,
                                                   ROBOT_APP_DAEMON_SHOOT_ONLINE_MS,
                                                   ROBOT_APP_DAEMON_SHOOT_PRIORITY};
    cfg[AppDaemonShootFric] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_SHOOT != 0U),
                                                ROBOT_APP_DAEMON_SHOOT_OFFLINE_MS,
                                                ROBOT_APP_DAEMON_SHOOT_ONLINE_MS,
                                                ROBOT_APP_DAEMON_SHOOT_PRIORITY};
    cfg[AppDaemonChassisDrive] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_CHASSIS != 0U),
                                                   ROBOT_APP_DAEMON_CHASSIS_OFFLINE_MS,
                                                   ROBOT_APP_DAEMON_CHASSIS_ONLINE_MS,
                                                   ROBOT_APP_DAEMON_CHASSIS_PRIORITY};
    cfg[AppDaemonChassisSteer] = (AppDaemonItemCfg){(uint8_t)(ROBOT_APP_ENABLE_CHASSIS != 0U),
                                                   ROBOT_APP_DAEMON_CHASSIS_OFFLINE_MS,
                                                   ROBOT_APP_DAEMON_CHASSIS_ONLINE_MS,
                                                   ROBOT_APP_DAEMON_CHASSIS_PRIORITY};
    cfg[AppDaemonTfmini] = (AppDaemonItemCfg){(uint8_t)(APP_DAEMON_USE_TFMINI != 0U),
                                             ROBOT_APP_DAEMON_TFMINI_OFFLINE_MS,
                                             ROBOT_APP_DAEMON_TFMINI_ONLINE_MS,
                                             ROBOT_APP_DAEMON_TFMINI_PRIORITY};
}

/**
 * @brief 依据配置初始化健康表。
 *
 * @param now_ms 当前时间戳，单位 ms。
 */
static void app_daemon_reset_table(uint32_t now_ms)
{
    AppDaemonItemCfg cfg[AppDaemonItemCount];
    uint32_t i;

    app_daemon_make_cfg(cfg);
    (void)memset(&app_daemon_state, 0, sizeof(app_daemon_state));
    app_daemon_state.highest_error = AppDaemonItemCount;

    for (i = 0U; i < (uint32_t)AppDaemonItemCount; i++) {
        app_daemon_state.item[i].new_time = now_ms;
        app_daemon_state.item[i].last_time = now_ms;
        app_daemon_state.item[i].lost_time = now_ms;
        app_daemon_state.item[i].work_time = now_ms;
        app_daemon_state.item[i].offline_ms = cfg[i].offline_ms;
        app_daemon_state.item[i].online_ms = cfg[i].online_ms;
        app_daemon_state.item[i].priority = cfg[i].priority;
        app_daemon_state.item[i].enable = cfg[i].enable;
        app_daemon_state.item[i].error = cfg[i].enable;
        app_daemon_state.item[i].lost = 0U;
        app_daemon_state.item[i].data_error = 0U;
        app_daemon_state.item[i].frequency = 0.0F;
    }
}

/**
 * @brief 在临界区内提交健康项心跳。
 *
 * @param item 健康项。
 * @param tick_ms 心跳时间戳，单位 ms。
 * @param data_error 非 0 表示本次心跳数据异常。
 * @return 返回 Daemon 状态码。
 */
static AppDaemonStatus app_daemon_report_locked(AppDaemonItem item,
                                                uint32_t tick_ms,
                                                uint8_t data_error)
{
    AppDaemonHealth *health;

    if (item >= AppDaemonItemCount) {
        return AppDaemonArg;
    }

    health = &app_daemon_state.item[item];
    if (health->enable == 0U) {
        return AppDaemonBusy;
    }

    if (tick_ms == 0U) {
        tick_ms = app_daemon_now_ms();
    }

    health->last_time = health->new_time;
    health->new_time = tick_ms;
    if (health->lost != 0U) {
        health->lost = 0U;
        health->error = 1U;
        health->work_time = tick_ms;
        app_daemon_push_event(item, AppDaemonEventRecovering, tick_ms);
    }
    if ((data_error != 0U) && (health->data_error == 0U)) {
        app_daemon_push_event(item, AppDaemonEventDataError, tick_ms);
    }
    health->data_error = (data_error != 0U) ? 1U : 0U;

    return AppDaemonOk;
}

/**
 * @brief 周期轮询提交健康项心跳。
 *
 * @param item 健康项。
 * @param tick_ms 心跳时间戳，单位 ms。
 * @param data_error 非 0 表示本次心跳数据异常。
 */
static void app_daemon_poll_report(AppDaemonItem item, uint32_t tick_ms, uint8_t data_error)
{
    uint32_t primask;

    primask = app_daemon_enter();
    if (app_daemon_report_locked(item, tick_ms, data_error) == AppDaemonOk) {
        app_daemon_diag.poll_count++;
    }
    app_daemon_exit(primask);
}

/**
 * @brief 更新全局错误汇总。
 *
 * 调用者必须已经进入临界区。
 *
 * @param item 健康项。
 */
static void app_daemon_raise_error(AppDaemonItem item)
{
    AppDaemonHealth *health;
    AppDaemonHealth *highest;

    if (item >= AppDaemonItemCount) {
        return;
    }

    health = &app_daemon_state.item[item];
    if ((health->enable == 0U) || (health->error == 0U)) {
        return;
    }

    if (app_daemon_state.any_error == 0U) {
        app_daemon_state.highest_error = item;
        app_daemon_state.any_error = 1U;
        return;
    }

    highest = &app_daemon_state.item[app_daemon_state.highest_error];
    if (health->priority > highest->priority) {
        app_daemon_state.highest_error = item;
    }
}

/**
 * @brief 根据时间戳判定所有健康项状态。
 *
 * @param now_ms 当前时间戳，单位 ms。
 */
static void app_daemon_eval(uint32_t now_ms)
{
    AppDaemonHealth *health;
    uint32_t primask;
    uint32_t i;
    uint32_t delta;
    uint8_t prev_error;
    uint8_t prev_lost;

    primask = app_daemon_enter();
    app_daemon_state.any_error = 0U;
    app_daemon_state.highest_error = AppDaemonItemCount;

    for (i = 0U; i < (uint32_t)AppDaemonItemCount; i++) {
        health = &app_daemon_state.item[i];
        if (health->enable == 0U) {
            continue;
        }

        prev_error = health->error;
        prev_lost = health->lost;

        if ((uint32_t)(now_ms - health->new_time) > health->offline_ms) {
            health->lost = 1U;
            health->error = 1U;
            if (prev_lost == 0U) {
                health->lost_time = now_ms;
                app_daemon_push_event((AppDaemonItem)i, AppDaemonEventOffline, now_ms);
            }
        } else if ((uint32_t)(now_ms - health->work_time) < health->online_ms) {
            health->lost = 0U;
            health->error = 1U;
        } else {
            health->lost = 0U;
            health->error = health->data_error;
            if ((prev_error != 0U) && (health->error == 0U)) {
                app_daemon_push_event((AppDaemonItem)i, AppDaemonEventOnline, now_ms);
            }
        }

        delta = health->new_time - health->last_time;
        if (delta > 0U) {
            health->frequency = 1000.0F / (float)delta;
        }

        app_daemon_raise_error((AppDaemonItem)i);
    }

    app_daemon_exit(primask);
}

/**
 * @brief 轮询 RC 设备状态。
 */
static void app_daemon_poll_rc(void)
{
    RcDiag diag;

    if (rc_diag(&diag) == 0U) {
        app_daemon_diag.poll_fail_count++;
        return;
    }

    if ((diag.ready != 0U) && (diag.last_valid_tick != 0U)) {
        app_daemon_poll_report(AppDaemonRcDbus, diag.last_valid_tick, 0U);
    }
}

/**
 * @brief 轮询 Referee 应用状态。
 */
static void app_daemon_poll_referee(void)
{
#if ROBOT_APP_ENABLE_REFEREE != 0U
    AppRefereeState state;

    if (app_referee_get_state(&state) == 0U) {
        app_daemon_diag.poll_fail_count++;
        return;
    }

    if ((state.online != 0U) && (state.last_rx_tick != 0U)) {
        app_daemon_poll_report(AppDaemonReferee,
                               state.last_rx_tick,
                               (uint8_t)((state.stale != 0U) || (state.valid_mask == 0U)));
    }
#endif
}

/**
 * @brief 轮询 Communication 应用状态。
 *
 * Host 通道目前只有计数器没有接收时间戳，因此使用接收计数边沿作为链路心跳。
 */
static void app_daemon_poll_comm(uint32_t now_ms)
{
#if ROBOT_APP_ENABLE_COMMUNICATION != 0U
    static uint32_t last_host_count;
    static uint32_t last_host_error_count;
    CommunicationHostDiag host;
    AppCommunicationDiag diag;
    uint32_t host_count;
    uint32_t host_error_count;
    uint8_t data_error;

    if (app_communication_get_host_diag(&host) != CommunicationOk) {
        app_daemon_diag.poll_fail_count++;
    } else {
        host_count = host.rx_frames + host.aim_count + host.probe_count + host.ack_count;
        host_error_count = host.crc_errors + host.id_errors + host.seq_errors +
                           host.direction_errors + host.decode_errors +
                           host.uart_errors + host.restart_fail_count;
        data_error = (host_error_count != last_host_error_count) ? 1U : 0U;
        if ((host.ready != 0U) &&
            ((host_count != last_host_count) || (data_error != 0U))) {
            last_host_count = host_count;
            last_host_error_count = host_error_count;
            app_daemon_poll_report(AppDaemonCommHost,
                                   now_ms,
                                   data_error);
        }
    }

    if (app_communication_get_diag(&diag) == 0U) {
        app_daemon_diag.poll_fail_count++;
        return;
    }

    if (diag.aim_online != 0U) {
        app_daemon_poll_report(AppDaemonCommVision,
                               now_ms,
                               (uint8_t)(diag.aim_status != CommunicationOk));
    }
#else
    (void)now_ms;
#endif
}

/**
 * @brief 轮询 Observe 应用状态。
 */
static void app_daemon_poll_observe(void)
{
#if ROBOT_APP_ENABLE_OBSERVE != 0U
    AppObserveDiag diag;
    uint32_t tick_ms;

    if (app_observe_get_diag(&diag) == 0U) {
        app_daemon_diag.poll_fail_count++;
        return;
    }

    tick_ms = diag.last_time_us / 1000U;
    if (diag.imu_online != 0U) {
        app_daemon_poll_report(AppDaemonObserveImu, tick_ms, 0U);
    }
    if (diag.ins_ready != 0U) {
        app_daemon_poll_report(AppDaemonObserveIns,
                               tick_ms,
                               (uint8_t)(diag.status != ObserverOk));
    }
#endif
}

/**
 * @brief 轮询 Gimbal 应用状态。
 */
static void app_daemon_poll_gimbal(void)
{
#if ROBOT_APP_ENABLE_GIMBAL != 0U
    AppGimbalDiag diag;

    if (app_gimbal_get_diag(&diag) == 0U) {
        app_daemon_diag.poll_fail_count++;
        return;
    }

    if ((diag.loop_count != 0U) && (diag.yaw_online != 0U)) {
        app_daemon_poll_report(AppDaemonGimbalYaw, diag.last_tick_ms, 0U);
    }
    if ((diag.loop_count != 0U) && (diag.pitch_online != 0U)) {
        app_daemon_poll_report(AppDaemonGimbalPitch, diag.last_tick_ms, 0U);
    }
#endif
}

/**
 * @brief 轮询 Shoot 应用状态。
 */
static void app_daemon_poll_shoot(void)
{
#if ROBOT_APP_ENABLE_SHOOT != 0U
    AppShootDiag diag;
    uint8_t fric_mask;

    if (app_shoot_get_diag(&diag) == 0U) {
        app_daemon_diag.poll_fail_count++;
        return;
    }

#if SHOOT_TYPE == SHOOT_TYPE_DOUBLE
    fric_mask = APP_DAEMON_SHOOT_FRIC_DOUBLE_MASK;
#elif SHOOT_TYPE == SHOOT_TYPE_NONE
    fric_mask = 0U;
#else
    fric_mask = APP_DAEMON_SHOOT_FRIC_MONO_MASK;
#endif

    if ((diag.loop_count != 0U) && (diag.trigger_online != 0U)) {
        app_daemon_poll_report(AppDaemonShootTrigger, diag.last_tick_ms, 0U);
    }
    if ((diag.loop_count != 0U) &&
        (fric_mask != 0U) &&
        ((diag.fric_online_mask & fric_mask) == fric_mask)) {
        app_daemon_poll_report(AppDaemonShootFric, diag.last_tick_ms, 0U);
    }
#endif
}

/**
 * @brief 轮询 Chassis 应用状态。
 */
static void app_daemon_poll_chassis(void)
{
#if ROBOT_APP_ENABLE_CHASSIS != 0U
    AppChassisDiag diag;

    if (app_chassis_heartbeat_alive(HAL_GetTick()) == 0U) {
        app_chassis_emergency_stop();
    }

    if (app_chassis_get_diag(&diag) == 0U) {
        app_daemon_diag.poll_fail_count++;
        return;
    }

    if ((diag.loop_count != 0U) &&
        ((diag.drive_online_mask & APP_DAEMON_CHASSIS_4_MASK) == APP_DAEMON_CHASSIS_4_MASK)) {
        app_daemon_poll_report(AppDaemonChassisDrive, diag.last_tick_ms, 0U);
    }

#if CHASSIS_TYPE == CHASSIS_TYPE_STEERING
    if ((diag.loop_count != 0U) &&
        ((diag.steer_online_mask & APP_DAEMON_CHASSIS_4_MASK) == APP_DAEMON_CHASSIS_4_MASK)) {
        app_daemon_poll_report(AppDaemonChassisSteer, diag.last_tick_ms, 0U);
    }
#endif
#endif
}

/**
 * @brief 轮询 TFmini 设备状态。
 */
static void app_daemon_poll_tfmini(uint32_t now_ms)
{
#if APP_DAEMON_USE_TFMINI != 0U
    if ((tfmini_ready() != false) &&
        (tfmini_online(ROBOT_APP_DAEMON_TFMINI_OFFLINE_MS) != false)) {
        app_daemon_poll_report(AppDaemonTfmini, now_ms, 0U);
    }
#else
    (void)now_ms;
#endif
}

/**
 * @brief 执行一轮 Daemon 健康轮询。
 *
 * @param now_ms 当前时间戳，单位 ms。
 */
static void app_daemon_poll(uint32_t now_ms)
{
    app_daemon_poll_rc();
    app_daemon_poll_referee();
    app_daemon_poll_comm(now_ms);
    app_daemon_poll_observe();
    app_daemon_poll_gimbal();
    app_daemon_poll_shoot();
    app_daemon_poll_chassis();
    app_daemon_poll_tfmini(now_ms);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Daemon 线程入口。
 *
 * @param argument CMSIS-RTOS2 输入参数。
 */
static void app_daemon_entry(void *argument)
{
    uint32_t now_ms;
    uint32_t primask;

    (void)argument;

    if (APP_DAEMON_INIT_DELAY_MS != 0U) {
        osDelay(APP_DAEMON_MS_TO_TICKS(APP_DAEMON_INIT_DELAY_MS));
    }

    while (1) {
        now_ms = app_daemon_now_ms();
        app_daemon_poll(now_ms);
        app_daemon_eval(now_ms);

        primask = app_daemon_enter();
        app_daemon_diag.loop_count++;
        app_daemon_diag.last_tick_ms = now_ms;
        app_daemon_exit(primask);

        osDelay(APP_DAEMON_MS_TO_TICKS(APP_DAEMON_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_daemon_init(void)
{
    uint32_t primask;

    (void)memset(&app_daemon_diag, 0, sizeof(app_daemon_diag));
    (void)memset(app_daemon_events, 0, sizeof(app_daemon_events));
    app_daemon_event_read = 0U;
    app_daemon_event_write = 0U;
    app_daemon_event_count = 0U;

    primask = app_daemon_enter();
    app_daemon_reset_table(app_daemon_now_ms());
    app_daemon_exit(primask);

    app_daemon_thread = osThreadNew(app_daemon_entry, NULL, &app_daemon_attr);
    primask = app_daemon_enter();
    app_daemon_diag.ready = (app_daemon_thread != NULL) ? 1U : 0U;
    app_daemon_exit(primask);

    return (app_daemon_thread != NULL) ? 1U : 0U;
}

AppDaemonStatus app_daemon_report(AppDaemonItem item, uint32_t tick_ms, uint8_t data_error)
{
    AppDaemonStatus status;
    uint32_t primask;

    primask = app_daemon_enter();
    status = app_daemon_report_locked(item, tick_ms, data_error);
    if (status == AppDaemonOk) {
        app_daemon_diag.report_count++;
    }
    app_daemon_exit(primask);

    return status;
}

uint8_t app_daemon_is_error(AppDaemonItem item)
{
    uint8_t error;
    uint32_t primask;

    if (item >= AppDaemonItemCount) {
        return 0U;
    }

    primask = app_daemon_enter();
    error = ((app_daemon_state.item[item].enable != 0U) &&
             (app_daemon_state.item[item].error != 0U)) ? 1U : 0U;
    app_daemon_exit(primask);

    return error;
}

AppDaemonStatus app_daemon_get_item(AppDaemonItem item, AppDaemonHealth *out)
{
    uint32_t primask;

    if (out == NULL) {
        return AppDaemonNull;
    }
    if (item >= AppDaemonItemCount) {
        return AppDaemonArg;
    }

    primask = app_daemon_enter();
    *out = app_daemon_state.item[item];
    app_daemon_exit(primask);

    return AppDaemonOk;
}

AppDaemonStatus app_daemon_get_state(AppDaemonState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return AppDaemonNull;
    }

    primask = app_daemon_enter();
    *out = app_daemon_state;
    app_daemon_exit(primask);

    return AppDaemonOk;
}

AppDaemonStatus app_daemon_get_diag(AppDaemonDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return AppDaemonNull;
    }

    primask = app_daemon_enter();
    *out = app_daemon_diag;
    app_daemon_exit(primask);

    return AppDaemonOk;
}

uint8_t app_daemon_read_event(AppDaemonEvent *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_daemon_enter();
    if (app_daemon_event_count == 0U) {
        app_daemon_exit(primask);
        return 0U;
    }

    *out = app_daemon_events[app_daemon_event_read];
    app_daemon_event_read++;
    if (app_daemon_event_read >= (uint16_t)APP_DAEMON_EVENT_MAX) {
        app_daemon_event_read = 0U;
    }
    app_daemon_event_count--;
    app_daemon_exit(primask);

    return 1U;
}

void app_daemon_clear_events(void)
{
    uint32_t primask;

    primask = app_daemon_enter();
    app_daemon_event_read = 0U;
    app_daemon_event_write = 0U;
    app_daemon_event_count = 0U;
    (void)memset(app_daemon_events, 0, sizeof(app_daemon_events));
    app_daemon_exit(primask);
}
