/**
 ******************************************************************************
 * @file    app_communication.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Communication 应用层实现。
 *
 * @note    本文件只做通讯组件初始化、云台状态上报、probe ack 和 Debug
 *          VOFA flush 的应用层调度。
 * @note    自瞄输入仍通过 Component/Input 仲裁后进入 GimbalCmd。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_communication.h"

#include "app_chassis.h"
#include "app_communication_config.h"
#include "app_gimbal.h"
#include "app_observe.h"
#include "bsp_timebase.h"
#include "communication_usb.h"
#include "imu.h"
#include "debug.h"
#include "main.h"
#include "cmsis_os.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_COMM_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/** @brief 将毫秒转换为微秒。 */
#define APP_COMM_MS_TO_US(ms) ((uint32_t)((uint32_t)(ms) * 1000U))

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 通信任务 glue 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Communication 任务控制块。 */
static osThreadId_t app_comm_thread;

/** @brief Communication 任务属性。 */
static const osThreadAttr_t app_comm_attr = {
    .name = "app_comm",
    .stack_size = APP_COMM_STACK_SIZE,
    .priority = (osPriority_t)APP_COMM_PRIORITY,
};

/** @brief Communication 应用层诊断缓存。 */
static AppCommunicationDiag app_comm_diag;

/** @brief 最近一次已处理的 host probe seq。 */
static uint32_t app_comm_last_probe_seq;

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/** @brief 最近一次已处理的 host probe ack seq。 */
static uint32_t app_comm_last_ack_seq;

#if ROBOT_APP_ENABLE_USB_BRIDGE != 0U
/** @brief 最近一次 USB 兼容状态帧发送时间，单位 us。 */
static uint32_t app_comm_last_usb_us;

/** @brief 最近一次 USB 四轮状态帧发送时间，单位 us。 */
static uint32_t app_comm_last_usb_wheel_us;

/** @brief 最近一次 USB IMU 状态帧发送时间，单位 us。 */
static uint32_t app_comm_last_usb_imu_us;

/** @brief 最近一次 USB 心跳发送时间，单位 us。 */
static uint32_t app_comm_last_usb_hb_us;
#endif

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
static uint32_t app_comm_enter(void)
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
static void app_comm_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 判断周期事件是否到期。
 *
 * @param now_us 当前时间戳低 32 位，单位 us。
 * @param last_us 上次触发时间戳。
 * @param period_ms 周期，单位 ms。
 * @return 到期返回 1，否则返回 0。
 */
static uint8_t app_comm_due(uint32_t now_us, uint32_t *last_us, uint32_t period_ms)
{
    uint32_t period_us;

    if (last_us == NULL) {
        return 0U;
    }

    period_us = APP_COMM_MS_TO_US(period_ms);
    if ((*last_us == 0U) || (period_us == 0U) || ((uint32_t)(now_us - *last_us) >= period_us)) {
        *last_us = now_us;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 更新最近状态。
 *
 * @param status 通讯组件状态。
 */
static void app_comm_set_last(CommunicationStatus status)
{
    uint32_t primask;

    primask = app_comm_enter();
    app_comm_diag.last_status = status;
    app_comm_exit(primask);
}

/**
 * @brief 根据 Gimbal 诊断生成状态 bit。
 *
 * @param diag Gimbal 诊断快照。
 * @return 状态 bit mask。
 */
static uint8_t app_comm_gimbal_status(const AppGimbalDiag *diag)
{
    uint8_t status = 0U;

    if (diag == NULL) {
        return 0U;
    }

    if (diag->input_online != 0U) {
        status |= APP_COMM_GIMBAL_STATUS_INPUT_ONLINE;
    }
    if (diag->observe_ready != 0U) {
        status |= APP_COMM_GIMBAL_STATUS_OBSERVE_READY;
    }
    if (diag->yaw_online != 0U) {
        status |= APP_COMM_GIMBAL_STATUS_YAW_ONLINE;
    }
    if (diag->pitch_online != 0U) {
        status |= APP_COMM_GIMBAL_STATUS_PITCH_ONLINE;
    }
    if (diag->send_enable != 0U) {
        status |= APP_COMM_GIMBAL_STATUS_SEND_ENABLE;
    }

    return status;
}

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Communication App 负责周期上报、probe/ack 和 Debug flush，协议细节留在 Component。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/**
 * @brief 周期读取自瞄缓存并更新诊断。
 */
static void app_comm_poll_aim(void)
{
    CommunicationAimState aim;

    (void)app_communication_get_aim(&aim);
}

/**
 * @brief 发送一帧云台状态。
 *
 * @param now_us 当前时间戳低 32 位，单位 us。
 */
static void app_comm_send_gimbal_state(uint32_t now_us)
{
    CommunicationGimbalState state;
    GimbalInput input;
    AppGimbalDiag diag;
    CommunicationStatus status;
    uint32_t primask;

    if ((app_gimbal_get_input(&input) == 0U) ||
        (app_gimbal_get_diag(&diag) == 0U) ||
        (diag.loop_count == 0U)) {
        primask = app_comm_enter();
        app_comm_diag.gimbal_ready = 0U;
        app_comm_diag.state_status = CommunicationState;
        app_comm_diag.last_status = CommunicationState;
        app_comm_diag.state_fail_count++;
        app_comm_exit(primask);
        return;
    }

    state.yaw_rad = input.ins_yaw;
    state.pitch_rad = input.ins_pitch;
    state.yaw_rate_rad_s = input.gyro_z;
    state.pitch_rate_rad_s = input.gyro_y;
    state.status = app_comm_gimbal_status(&diag);
    state.time_us = now_us;

    status = communication_send_state(&state);

    primask = app_comm_enter();
    app_comm_diag.gimbal_ready = 1U;
    app_comm_diag.state_status = status;
    app_comm_diag.last_status = status;
    if (status == CommunicationOk) {
        app_comm_diag.state_send_count++;
    } else {
        app_comm_diag.state_fail_count++;
    }
    app_comm_exit(primask);
}

#if ROBOT_APP_ENABLE_USB_BRIDGE != 0U
/**
 * @brief 汇总底盘故障位，供 0x81/0x83 共用。
 *
 * @return 16 位故障位。
 */
static uint16_t app_comm_chassis_fault_flags(void)
{
    uint16_t flags = 0U;

#if ROBOT_APP_ENABLE_CHASSIS != 0U
    static AppChassisDiag diag;

    if (app_chassis_get_diag(&diag) == 0U) {
        return 0U;
    }

    if (diag.dm_fault_latched != 0U) {
        flags |= 0x0001U;
    }
    if (diag.dm_mode_fail_count != 0U) {
        flags |= 0x0002U;
    }
    if (diag.dm_tx_drop_count != 0U) {
        flags |= 0x0004U;
    }
    if (diag.imu_status != (int32_t)IMU_OK) {
        flags |= 0x0008U;
    }
    if (diag.send_enable == 0U) {
        flags |= 0x0010U;
    }
    if (diag.drive_online_mask != 0x0FU) {
        flags |= 0x0020U;
    }
    {
        static uint32_t last_overrun;
        static uint8_t overrun_init;

        if (overrun_init == 0U) {
            last_overrun = diag.cycle_overrun_count;
            overrun_init = 1U;
        } else if (diag.cycle_overrun_count != last_overrun) {
            flags |= 0x0040U;
            last_overrun = diag.cycle_overrun_count;
        }
    }
    if (diag.chassis_status != ChassisOk) {
        flags |= 0x0080U;
    }
    flags |= (uint16_t)((uint16_t)(diag.dm_fault_mask & 0x0FU) << 8);
#else
    (void)flags;
#endif

    return flags;
}

/**
 * @brief 发送四轮动态状态帧（0x83）。
 *
 * @param now_us 当前时间戳低 32 位，单位 us。
 */
static void app_comm_send_usb_wheel(uint32_t now_us)
{
#if ROBOT_APP_ENABLE_CHASSIS != 0U
    static AppChassisDiag diag;
    CommunicationUsbWheelState state;
    uint32_t i;

    if ((app_chassis_get_diag(&diag) == 0U) || (diag.loop_count == 0U)) {
        return;
    }

    (void)memset(&state, 0, sizeof(state));
    state.mcu_ts_ms = (uint32_t)(now_us / 1000U);
    state.online_mask = diag.drive_online_mask;
    state.mode = (uint8_t)diag.mode;
    state.chassis_status = (uint16_t)diag.chassis_status;
    state.fault_flags = app_comm_chassis_fault_flags();

    for (i = 0U; i < 4U; i++) {
        float torque_mnm = diag.drive_torque_nm[i] * 1000.0F;

        if (torque_mnm > 32767.0F) {
            torque_mnm = 32767.0F;
        } else if (torque_mnm < -32768.0F) {
            torque_mnm = -32768.0F;
        }

        state.speed_mps[i] = diag.drive_speed_fb[i];
        state.position_rad[i] = diag.drive_position_rad[i];
        state.temp_mos_c[i] = diag.drive_temp_mos[i];
        state.temp_motor_c[i] = diag.drive_temp_motor[i];
        state.torque_mnm[i] = (int16_t)torque_mnm;
    }

    communication_usb_send_wheel_state(&state);
#else
    (void)now_us;
#endif
}

/**
 * @brief 发送 IMU 动态状态帧（0x84）。
 *
 * @param now_us 当前时间戳低 32 位，单位 us。
 */
static void app_comm_send_usb_imu(uint32_t now_us)
{
    CommunicationUsbImuState state;
    imu_health_t health;

    (void)memset(&state, 0, sizeof(state));
    state.mcu_ts_ms = (uint32_t)(now_us / 1000U);
    state.quat_wxyz[0] = 1.0F;
    state.imu_status = (int32_t)IMU_ERR_NOT_READY;

    (void)imu_get_health(&health);
    state.last_ok_ms = health.last_ok_ms;

#if ROBOT_APP_ENABLE_OBSERVE != 0U
    {
        ObserverState observer;

        if ((app_observe_get_state(&observer) != 0U) &&
            (observer.ins.valid != 0U)) {
            state.gyro[0] = observer.ins.gyro.x;
            state.gyro[1] = observer.ins.gyro.y;
            state.gyro[2] = observer.ins.gyro.z;
            state.accel[0] = observer.ins.accel.x;
            state.accel[1] = observer.ins.accel.y;
            state.accel[2] = observer.ins.accel.z;
            state.temperature_c = observer.ins.temperature;
            state.quat_wxyz[0] = observer.ins.quat.w;
            state.quat_wxyz[1] = observer.ins.quat.x;
            state.quat_wxyz[2] = observer.ins.quat.y;
            state.quat_wxyz[3] = observer.ins.quat.z;
            state.valid_mask = 0x01U | 0x04U;
            if (observer.ins.ready != 0U) {
                state.valid_mask |= 0x02U;
            }
            state.imu_status = (int32_t)health.last_status;
        }
    }
#elif ROBOT_APP_ENABLE_CHASSIS != 0U
    {
        static AppChassisDiag diag;

        if ((app_chassis_get_diag(&diag) != 0U) && (diag.loop_count != 0U)) {
            state.gyro[0] = diag.imu_gyro[0];
            state.gyro[1] = diag.imu_gyro[1];
            state.gyro[2] = diag.imu_gyro[2];
            state.accel[0] = diag.imu_accel[0];
            state.accel[1] = diag.imu_accel[1];
            state.accel[2] = diag.imu_accel[2];
            state.temperature_c = diag.imu_temperature;
            state.valid_mask = 0x01U | 0x04U;
            state.imu_status = diag.imu_status;
        }
    }
#else
    (void)now_us;
#endif

    communication_usb_send_imu_state(&state);
}

/**
 * @brief 把当前机器人状态封装成 USB 桥接帧并发送。
 *
 * @param now_us 当前时间戳低 32 位，单位 us。
 */
static void app_comm_send_usb_state(uint32_t now_us)
{
    CommunicationUsbState state;
    uint32_t now_ms = (uint32_t)(now_us / 1000U);

    (void)memset(&state, 0, sizeof(state));
    state.mcu_ts_ms = now_ms;
    state.mode = 3U;
    state.error_flags = app_comm_chassis_fault_flags();

#if ROBOT_APP_ENABLE_GIMBAL != 0U
    {
        GimbalInput input;

        if (app_gimbal_get_input(&input) != 0U) {
            state.imu_yaw = input.ins_yaw;
            state.imu_pitch = input.ins_pitch;
            state.gimbal_yaw = input.ins_yaw;
            state.gimbal_pitch = input.ins_pitch;
            state.wz = input.gyro_z;
            state.status |= 0x04U;
            if (input.ins_ready != 0U) {
                state.status |= 0x02U;
            }
        }
    }
#endif

#if ROBOT_APP_ENABLE_OBSERVE != 0U
    {
        ObserverState observer;

        if (app_observe_get_state(&observer) != 0U) {
            state.imu_roll = observer.ins.euler.roll;
            state.imu_pitch = observer.ins.euler.pitch;
            state.imu_yaw = observer.ins.euler.yaw;
            state.vx = observer.chassis.vel.x;
            state.vy = observer.chassis.vel.y;
            if (observer.ins.ready != 0U) {
                state.status |= 0x02U;
            }
        }
    }
#endif

    if (communication_usb_link_online(HAL_GetTick()) != 0U) {
        state.status |= 0x01U;
    }

    communication_usb_send_state(&state);
}
#endif

/**
 * @brief 处理 host probe 并按需发送 ack。
 *
 * @param now_us 当前时间戳低 32 位，单位 us。
 */
static void app_comm_handle_probe(uint32_t now_us)
{
#if APP_COMM_AUTO_ACK_ENABLE != 0U
    CommunicationProbe probe;
    CommunicationProbeAck ack;
    CommunicationStatus status;
    uint32_t primask;

    status = communication_get_probe(&probe);
    if ((status != CommunicationOk) || (probe.seq == 0U) || (probe.seq == app_comm_last_probe_seq)) {
        return;
    }

    app_comm_last_probe_seq = probe.seq;
    status = communication_time_make_ack(&probe, now_us, &ack);
    if (status == CommunicationOk) {
        status = communication_send_probe_ack(ack.token, ack.request_time_us, ack.reply_time_us);
    }

    primask = app_comm_enter();
    app_comm_diag.last_probe_seq = probe.seq;
    app_comm_diag.ack_status = status;
    app_comm_diag.last_status = status;
    if (status == CommunicationOk) {
        app_comm_diag.probe_ack_count++;
    } else {
        app_comm_diag.probe_ack_fail_count++;
    }
    app_comm_exit(primask);
#else
    (void)now_us;
#endif
}

/**
 * @brief 处理 host probe ack 并更新时间同步状态。
 *
 * @param now_us 当前时间戳低 32 位，单位 us。
 */
static void app_comm_handle_ack(uint32_t now_us)
{
    CommunicationProbeAck ack;
    CommunicationStatus status;
    uint32_t primask;

    status = communication_get_probe_ack(&ack);
    if ((status != CommunicationOk) || (ack.seq == 0U) || (ack.seq == app_comm_last_ack_seq)) {
        return;
    }

    app_comm_last_ack_seq = ack.seq;
    status = communication_time_on_ack(&ack, now_us);

    primask = app_comm_enter();
    app_comm_diag.last_ack_seq = ack.seq;
    if (status != CommunicationOk) {
        app_comm_diag.probe_fail_count++;
    }
    app_comm_exit(primask);
}

/**
 * @brief 输出 profiler 快照并更新诊断。
 */
static void app_comm_flush_profiler(void)
{
    CommunicationStatus status;
    uint32_t primask;

    status = communication_debug_profiler_flush();

    primask = app_comm_enter();
    if (status == CommunicationOk) {
        app_comm_diag.profiler_flush_count++;
    } else if (status == CommunicationBusy) {
        app_comm_diag.profiler_busy_count++;
    } else {
        app_comm_diag.profiler_error_count++;
    }
    app_comm_exit(primask);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Communication 任务入口。
 *
 * @param argument CMSIS-RTOS2 输入参数。
 */
static void app_comm_entry(void *argument)
{
    uint32_t now_us;
    uint32_t last_state_us = 0U;
    uint32_t last_debug_us = 0U;
    uint32_t last_profiler_us = 0U;
    uint32_t primask;
#if ROBOT_APP_ENABLE_USB_BRIDGE != 0U
    uint8_t usb_frame_sent;
#endif

    (void)argument;

    while (1) {
        now_us = (uint32_t)bsp_timebase_now_us();
#if ROBOT_APP_ENABLE_USB_BRIDGE != 0U
        usb_frame_sent = 0U;
#endif

        app_comm_poll_aim();
        app_comm_handle_probe(now_us);
        app_comm_handle_ack(now_us);

        if (app_comm_due(now_us, &last_state_us, APP_COMM_STATE_PERIOD_MS) != 0U) {
            app_comm_send_gimbal_state(now_us);
        }

#if ROBOT_APP_ENABLE_USB_BRIDGE != 0U
        /* 命令新鲜度使用 HAL tick，和 app_chassis 的时间域一致。 */
        communication_usb_poll(HAL_GetTick());

        /* CDC 没有 TX 队列：每 5 ms tick 只发一帧，按优先级轮转。 */
#if ROBOT_APP_ENABLE_CHASSIS != 0U
        if (app_comm_due(now_us, &app_comm_last_usb_wheel_us, APP_COMM_USB_PERIOD_MS) != 0U) {
            app_comm_send_usb_wheel(now_us);
            usb_frame_sent = 1U;
        } else
#endif
#if (ROBOT_APP_ENABLE_OBSERVE != 0U) || (ROBOT_APP_ENABLE_CHASSIS != 0U)
        if (app_comm_due(now_us, &app_comm_last_usb_imu_us, APP_COMM_USB_PERIOD_MS) != 0U) {
            app_comm_send_usb_imu(now_us);
            usb_frame_sent = 1U;
        } else
#endif
        if (app_comm_due(now_us, &app_comm_last_usb_us, APP_COMM_USB_PERIOD_MS) != 0U) {
            app_comm_send_usb_state(now_us);
            usb_frame_sent = 1U;
        } else if (app_comm_due(now_us, &app_comm_last_usb_hb_us, APP_COMM_USB_HEARTBEAT_MS) != 0U) {
            uint32_t now_tick = HAL_GetTick();
            communication_usb_send_heartbeat(
                now_tick,
                3U,
                (communication_usb_link_online(now_tick) != 0U) ? 0x01U : 0x00U);
            usb_frame_sent = 1U;
        }
#endif

#if APP_COMM_DEBUG_FLUSH_ENABLE != 0U
#if ROBOT_APP_ENABLE_USB_BRIDGE != 0U
        if ((usb_frame_sent == 0U) &&
            (app_comm_due(now_us, &last_debug_us, APP_COMM_DEBUG_PERIOD_MS) != 0U)) {
#else
        if (app_comm_due(now_us, &last_debug_us, APP_COMM_DEBUG_PERIOD_MS) != 0U) {
#endif
            (void)app_communication_flush_debug();
        }
#else
        (void)last_debug_us;
#endif
#if APP_COMM_DEBUG_PROFILER_ENABLE != 0U
        if (app_comm_due(now_us, &last_profiler_us, APP_COMM_DEBUG_PERIOD_MS) != 0U) {
            app_comm_flush_profiler();
        }
#else
        (void)last_profiler_us;
#endif

        primask = app_comm_enter();
        app_comm_diag.loop_count++;
        app_comm_diag.last_time_us = now_us;
        app_comm_exit(primask);

        osDelay(APP_COMM_MS_TO_TICKS(APP_COMM_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_communication_init(void)
{
    CommunicationCfg cfg;
    CommunicationStatus status;
    uint32_t primask;

    (void)memset(&app_comm_diag, 0, sizeof(app_comm_diag));
    app_comm_last_probe_seq = 0U;
    app_comm_last_ack_seq = 0U;

#if ROBOT_APP_ENABLE_USB_BRIDGE != 0U
    app_comm_last_usb_us = 0U;
    app_comm_last_usb_wheel_us = 0U;
    app_comm_last_usb_imu_us = 0U;
    app_comm_last_usb_hb_us = 0U;
    communication_usb_init();
#endif

    if (debug_init() != DebugOk) {
        app_comm_diag.init_status = CommunicationState;
        app_comm_diag.last_status = CommunicationState;
        return 0U;
    }

    status = communication_default_cfg(&cfg);
    if (status != CommunicationOk) {
        app_comm_diag.init_status = status;
        app_comm_diag.last_status = status;
        return 0U;
    }
    cfg.host.aim_priority = (uint8_t)APP_COMM_AIM_PRIORITY;
    cfg.time.initiator = (uint8_t)APP_COMM_TIME_INITIATOR;
    cfg.time.max_rtt_us = (uint32_t)APP_COMM_TIME_MAX_RTT_US;

    status = communication_init(&cfg);
    if (status != CommunicationOk) {
        app_comm_diag.init_status = status;
        app_comm_diag.last_status = status;
        return 0U;
    }

    primask = app_comm_enter();
    app_comm_diag.ready = 1U;
    app_comm_diag.init_status = CommunicationOk;
    app_comm_diag.last_status = CommunicationOk;
    app_comm_exit(primask);

    app_comm_thread = osThreadNew(app_comm_entry, NULL, &app_comm_attr);
    return (app_comm_thread != NULL) ? 1U : 0U;
}

CommunicationStatus app_communication_get_aim(CommunicationAimState *out)
{
    CommunicationStatus status;
    uint32_t primask;

    status = communication_get_aim(out);

    primask = app_comm_enter();
    app_comm_diag.aim_status = status;
    app_comm_diag.last_status = status;
    app_comm_diag.aim_online = ((status == CommunicationOk) &&
                                (out != NULL) &&
                                (out->enable != 0U)) ? 1U : 0U;
    if (app_comm_diag.aim_online != 0U) {
        app_comm_diag.aim_count++;
    }
    app_comm_exit(primask);

    return status;
}

uint8_t app_communication_get_diag(AppCommunicationDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_comm_enter();
    *out = app_comm_diag;
    app_comm_exit(primask);
    return 1U;
}

CommunicationStatus app_communication_get_host_diag(CommunicationHostDiag *out)
{
    return communication_host_diag(out);
}

CommunicationStatus app_communication_get_debug_diag(CommunicationDebugDiag *out)
{
    return communication_debug_diag(out);
}

CommunicationStatus app_communication_get_probe(CommunicationProbe *out)
{
    return communication_get_probe(out);
}

CommunicationStatus app_communication_get_probe_ack(CommunicationProbeAck *out)
{
    CommunicationStatus status;
    uint32_t primask;

    status = communication_get_probe_ack(out);
    if ((status == CommunicationOk) && (out != NULL)) {
        (void)communication_time_on_ack(out, (uint32_t)bsp_timebase_now_us());
        primask = app_comm_enter();
        app_comm_diag.last_ack_seq = out->seq;
        app_comm_exit(primask);
    }

    return status;
}

CommunicationStatus app_communication_send_probe(uint16_t token, uint32_t time_us)
{
    CommunicationStatus status;
    uint32_t primask;

    status = communication_send_probe(token, time_us);

    primask = app_comm_enter();
    app_comm_diag.probe_status = status;
    app_comm_diag.last_status = status;
    if (status == CommunicationOk) {
        app_comm_diag.probe_send_count++;
    } else {
        app_comm_diag.probe_fail_count++;
    }
    app_comm_exit(primask);

    return status;
}

CommunicationStatus app_communication_get_time(CommunicationTimeState *out)
{
    return communication_get_time(out);
}

CommunicationStatus app_communication_flush_debug(void)
{
    CommunicationStatus status;
    uint32_t primask;

    status = communication_debug_flush();

    primask = app_comm_enter();
    app_comm_diag.debug_status = status;
    app_comm_diag.last_status = status;
    if (status == CommunicationOk) {
        app_comm_diag.debug_flush_count++;
    } else if (status == CommunicationBusy) {
        app_comm_diag.debug_busy_count++;
    } else {
        app_comm_diag.debug_error_count++;
    }
    app_comm_exit(primask);

    return status;
}
