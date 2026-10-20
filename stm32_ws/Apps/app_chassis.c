/**
 ******************************************************************************
 * @file    app_chassis.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Chassis 应用层实现。
 *
 * @note    本文件只做 Input/Observe/电机反馈到 Chassis controller 的数据接线。
 * @note    底盘行为参考已经在 Input 链路中转换为 ChassisCmd，本文件不读取遥控器。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_chassis.h"

#include "app_chassis_config.h"
#include "app_gimbal.h"
#include "app_input.h"
#include "app_observe.h"
#if ROBOT_APP_ENABLE_CALIBRATION != 0U
#include "app_calibration.h"
#endif
#include "bsp_timebase.h"
#include "debug.h"
#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#if CHASSIS_TYPE == CHASSIS_TYPE_MECANUM
#include "chassis_mecanum.h"
#elif CHASSIS_TYPE == CHASSIS_TYPE_OMNI
#include "chassis_omni.h"
#elif CHASSIS_TYPE == CHASSIS_TYPE_STEERING
#include "chassis_steering.h"
#elif CHASSIS_TYPE == CHASSIS_TYPE_BALANCE
#include "chassis_balance.h"
#else
#error "Unsupported CHASSIS_TYPE"
#endif

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 四路底盘电机全部在线的 bit mask。 */
#define APP_CHASSIS_MOTOR_ALL_MASK ((uint8_t)0x0FU)

/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_CHASSIS_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/**
 * @brief 底盘功率测量缓存。
 */
typedef struct
{
    float32_t power;     /**< 当前底盘功率，单位 W。 */
    float32_t limit;     /**< 当前底盘功率上限，单位 W。 */
    float32_t buffer;    /**< 当前底盘缓冲能量。 */
    uint8_t valid;       /**< 非 0 表示缓存有效。 */
} AppChassisPower;

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 底盘应用 glue 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Chassis 任务控制块。 */
static osThreadId_t app_chassis_thread;

/** @brief Chassis 任务属性。 */
static const osThreadAttr_t app_chassis_attr = {
    .name = "app_chassis",
    .stack_size = APP_CHASSIS_STACK_SIZE,
    .priority = (osPriority_t)APP_CHASSIS_PRIORITY,
};

/** @brief Chassis 公共上下文。 */
static Chassis app_chassis_ctx;

/** @brief Chassis 公共上下文快照。 */
static Chassis app_chassis_snapshot;

/** @brief 最近一次 Chassis 输入快照。 */
static ChassisInput app_chassis_input;

/** @brief 最近一次 Chassis 输出快照。 */
static ChassisOutput app_chassis_output;

/** @brief Chassis 应用层诊断快照。 */
static AppChassisDiag app_chassis_diag;

/** @brief 最近一次功率注入缓存。 */
static AppChassisPower app_chassis_power;

/** @brief 任务心跳时间戳，单位 ms。 */
static volatile uint32_t app_chassis_heartbeat_ms;

/** @brief 任务周期 overrun 计数。 */
static volatile uint32_t app_chassis_overrun_count;

/** @brief 外部急停标志。 */
static volatile uint8_t app_chassis_emergency;

#if defined(APP_CHASSIS_BENCH_TEST) && (APP_CHASSIS_BENCH_TEST != 0U)
/** @brief 台架注入起始时间，单位 ms。 */
static uint32_t app_chassis_bench_start_ms;
#endif

#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
/** @brief DM 速度模式是否已完成启动序列。 */
static uint8_t app_chassis_dm_ready;

/** @brief DM 故障计时起点，单位 ms。 */
static uint32_t app_chassis_dm_fault_since_ms;

/** @brief DM 故障锁存后最近一次停止帧时间，单位 ms。 */
static uint32_t app_chassis_dm_last_stop_ms;

/** @brief DM 故障锁存标志。 */
static uint8_t app_chassis_dm_fault_latched;

/** @brief DM 启动完成时间，用于故障锁存宽限，单位 ms。 */
static uint32_t app_chassis_dm_start_ms;

/** @brief DM 故障锁存次数。 */
static uint32_t app_chassis_dm_fault_count;

/** @brief DM 速度帧发送失败次数。 */
static uint32_t app_chassis_dm_tx_drop_count;

/** @brief 最近一次重发使能的时间，单位 ms（HAL tick）。 */
static uint32_t app_chassis_dm_last_enable_ms;

/** @brief 使能帧发送成功次数。 */
static uint32_t app_chassis_dm_enable_count;

/** @brief 使能帧发送失败次数。 */
static uint32_t app_chassis_dm_enable_fail_count;

/** @brief 每个驱动轮运行期间反馈速度最小值，m/s。 */
static float32_t app_chassis_dm_fb_min[CHASSIS_DRIVE_MAX];

/** @brief 每个驱动轮运行期间反馈速度最大值，m/s。 */
static float32_t app_chassis_dm_fb_max[CHASSIS_DRIVE_MAX];

/** @brief 每个驱动轮是否已经记录过反馈极值。 */
static uint8_t app_chassis_dm_fb_seen[CHASSIS_DRIVE_MAX];

/** @brief CTRL_MODE 写入并读回成功的电机数量。 */
static uint32_t app_chassis_dm_mode_ok_count;

/** @brief CTRL_MODE 写入或读回失败的电机数量。 */
static uint32_t app_chassis_dm_mode_fail_count;

/** @brief 每个驱动轮读回的 CTRL_MODE，0xFFFFFFFF 表示未读到。 */
static uint32_t app_chassis_dm_mode_value[CHASSIS_DRIVE_MAX];

/** @brief 运行期间每个驱动轮的反馈离线周期计数。 */
static uint32_t app_chassis_drive_offline_count[CHASSIS_DRIVE_MAX];

/** @brief 每个驱动轮首次起转、首次达到目标 80%、首次使能的时间。 */
static uint32_t app_chassis_drive_first_move_ms[CHASSIS_DRIVE_MAX];
static uint32_t app_chassis_drive_first_full_ms[CHASSIS_DRIVE_MAX];
static uint32_t app_chassis_drive_first_enabled_ms[CHASSIS_DRIVE_MAX];
static uint8_t app_chassis_drive_first_move_seen[CHASSIS_DRIVE_MAX];
static uint8_t app_chassis_drive_first_full_seen[CHASSIS_DRIVE_MAX];
static uint8_t app_chassis_drive_first_enabled_seen[CHASSIS_DRIVE_MAX];

/** @brief 首次进入故障状态时的驱动轮在线掩码。 */
static uint8_t app_chassis_dm_fault_mask;

static void app_chassis_dm_start(void);
static void app_chassis_dm_set_speed_mode(void);
static void app_chassis_dm_enable_all(uint32_t now_ms);
static void app_chassis_dm_force_stop(uint32_t now_ms);
#endif

#if CHASSIS_TYPE == CHASSIS_TYPE_MECANUM
/** @brief 麦克纳姆轮底盘私有上下文。 */
static ChassisMecanumCtx app_chassis_impl;
#elif CHASSIS_TYPE == CHASSIS_TYPE_OMNI
/** @brief 全向轮底盘私有上下文。 */
static ChassisOmniCtx app_chassis_impl;
#elif CHASSIS_TYPE == CHASSIS_TYPE_STEERING
/** @brief 舵轮底盘私有上下文。 */
static ChassisSteeringCtx app_chassis_impl;
#elif CHASSIS_TYPE == CHASSIS_TYPE_BALANCE
/** @brief 平衡底盘私有上下文。 */
static ChassisBalanceCtx app_chassis_impl;
#endif

/** @brief 底盘驱动电机反馈 ID 表。 */
static const uint16_t app_chassis_drive_id[CHASSIS_DRIVE_MAX] = {
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_0,
    ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_1,
    ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_2,
    ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_3,
#else
    MOTOR_CHASSIS_DRIVE_0_ID,
    MOTOR_CHASSIS_DRIVE_1_ID,
    MOTOR_CHASSIS_DRIVE_2_ID,
    MOTOR_CHASSIS_DRIVE_3_ID,
#endif
};

/** @brief 底盘舵向电机反馈 ID 表。 */
static const uint16_t app_chassis_steer_id[CHASSIS_STEER_MAX] = {
    MOTOR_CHASSIS_STEER_0_ID,
    MOTOR_CHASSIS_STEER_1_ID,
    MOTOR_CHASSIS_STEER_2_ID,
    MOTOR_CHASSIS_STEER_3_ID,
};

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
static uint32_t app_chassis_enter(void)
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
static void app_chassis_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 填充一个 PID 配置结构。
 *
 * @param cfg PID 配置输出。
 * @param kp 比例增益。
 * @param ki 积分增益。
 * @param kd 微分增益。
 * @param max_out 输出限幅。
 * @param max_iout 积分限幅。
 */
static void app_chassis_pid(ChassisPidCfg *cfg,
                            float32_t kp,
                            float32_t ki,
                            float32_t kd,
                            float32_t max_out,
                            float32_t max_iout)
{
    if (cfg == NULL) {
        return;
    }

    cfg->kp = kp;
    cfg->ki = ki;
    cfg->kd = kd;
    cfg->max_out = max_out;
    cfg->max_iout = max_iout;
}

/* ==========================================================================
 * 配置生成与参数展开
 * --------------------------------------------------------------------------
 * 本区把 Robot target 宏展开成 Component/Device 的 Cfg，运行期控制逻辑不写在配置头里。
 * ========================================================================== */
/**
 * @brief 生成 Chassis controller 配置。
 *
 * @param cfg 输出配置。
 */
static void app_chassis_make_cfg(ChassisCfg *cfg)
{
    uint32_t i;

    if (cfg == NULL) {
        return;
    }

    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->type = (ChassisType)CHASSIS_TYPE;
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    cfg->drive_mode = ChassisDriveModeSpeed;
#else
    cfg->drive_mode = ChassisDriveModeCurrent;
#endif
#if defined(ROBOT_CHASSIS_MIN_ONLINE_MOTORS)
    cfg->min_online_motors = (uint8_t)ROBOT_CHASSIS_MIN_ONLINE_MOTORS;
#else
    cfg->min_online_motors = (uint8_t)CHASSIS_DRIVE_MAX;
#endif
    app_chassis_pid(&cfg->drive_pid,
                    ROBOT_CHASSIS_DRIVE_PID_KP,
                    ROBOT_CHASSIS_DRIVE_PID_KI,
                    ROBOT_CHASSIS_DRIVE_PID_KD,
                    ROBOT_CHASSIS_DRIVE_PID_MAX_OUT,
                    ROBOT_CHASSIS_DRIVE_PID_MAX_IOUT);
    app_chassis_pid(&cfg->steer_angle_pid,
                    ROBOT_CHASSIS_STEER_ANGLE_PID_KP,
                    ROBOT_CHASSIS_STEER_ANGLE_PID_KI,
                    ROBOT_CHASSIS_STEER_ANGLE_PID_KD,
                    ROBOT_CHASSIS_STEER_ANGLE_PID_MAX_OUT,
                    ROBOT_CHASSIS_STEER_ANGLE_PID_MAX_IOUT);
    app_chassis_pid(&cfg->steer_speed_pid,
                    ROBOT_CHASSIS_STEER_SPEED_PID_KP,
                    ROBOT_CHASSIS_STEER_SPEED_PID_KI,
                    ROBOT_CHASSIS_STEER_SPEED_PID_KD,
                    ROBOT_CHASSIS_STEER_SPEED_PID_MAX_OUT,
                    ROBOT_CHASSIS_STEER_SPEED_PID_MAX_IOUT);
    app_chassis_pid(&cfg->yaw_pid,
                    ROBOT_CHASSIS_FOLLOW_PID_KP,
                    ROBOT_CHASSIS_FOLLOW_PID_KI,
                    ROBOT_CHASSIS_FOLLOW_PID_KD,
                    ROBOT_CHASSIS_FOLLOW_PID_MAX_OUT,
                    ROBOT_CHASSIS_FOLLOW_PID_MAX_IOUT);
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    cfg->drive_rpm_to_speed = 0.0F;
    cfg->max_drive_speed = ROBOT_CHASSIS_DM_MAX_WHEEL_SPEED;
#else
    cfg->drive_rpm_to_speed = ROBOT_CHASSIS_RPM_TO_VECTOR;
    cfg->max_drive_speed = ROBOT_CHASSIS_MAX_WHEEL_SPEED;
#endif
    cfg->steer_rpm_to_speed = 1.0F;
    cfg->steer_ecd_to_rad = 1.0F / ROBOT_CHASSIS_STEER_ANGLE_RATIO;
    cfg->max_drive_current = ROBOT_CHASSIS_MAX_CURRENT;
    cfg->max_steer_current = ROBOT_CHASSIS_STEER_CURRENT_MAX;
    cfg->power_limit = ROBOT_CHASSIS_POWER_LIMIT;
    cfg->power_cfg.k1 = ROBOT_POWER_CHASSIS_K1;
    cfg->power_cfg.k2 = ROBOT_POWER_CHASSIS_K2;
    cfg->power_cfg.offset = ROBOT_POWER_CHASSIS_OFFSET;
    cfg->power_cfg.torque_coeff = ROBOT_POWER_CHASSIS_TORQUE;
    cfg->power_cfg.rpm_to_rad = ROBOT_POWER_CHASSIS_RPM_RAD;
    cfg->power_cfg.reserve_power = ROBOT_POWER_CHASSIS_RESERVE;
    cfg->power_cfg.min_scale = ROBOT_POWER_CHASSIS_SCALE_MIN;
    cfg->power_cfg.fallback_scale = ROBOT_POWER_CHASSIS_SCALE_FALLBACK;
    cfg->mecanum_center = ROBOT_CHASSIS_CENTER_DIST;
    cfg->omni_center = ROBOT_CHASSIS_CENTER_DIST;
    cfg->wz_scale = ROBOT_CHASSIS_WZ_SET_SCALE;
    cfg->half_length = ROBOT_CHASSIS_HALF_LENGTH;
    cfg->half_width = ROBOT_CHASSIS_HALF_WIDTH;
    cfg->steer_zero[0] = ROBOT_CHASSIS_STEER_ZERO_0;
    cfg->steer_zero[1] = ROBOT_CHASSIS_STEER_ZERO_1;
    cfg->steer_zero[2] = ROBOT_CHASSIS_STEER_ZERO_2;
    cfg->steer_zero[3] = ROBOT_CHASSIS_STEER_ZERO_3;
    cfg->drive_polarity[0] = (int8_t)ROBOT_CHASSIS_DRIVE_POLARITY_0;
    cfg->drive_polarity[1] = (int8_t)ROBOT_CHASSIS_DRIVE_POLARITY_1;
    cfg->drive_polarity[2] = (int8_t)ROBOT_CHASSIS_DRIVE_POLARITY_2;
    cfg->drive_polarity[3] = (int8_t)ROBOT_CHASSIS_DRIVE_POLARITY_3;
    cfg->steer_polarity[0] = (int8_t)ROBOT_CHASSIS_STEER_POLARITY_0;
    cfg->steer_polarity[1] = (int8_t)ROBOT_CHASSIS_STEER_POLARITY_1;
    cfg->steer_polarity[2] = (int8_t)ROBOT_CHASSIS_STEER_POLARITY_2;
    cfg->steer_polarity[3] = (int8_t)ROBOT_CHASSIS_STEER_POLARITY_3;

    for (i = 0U; i < CHASSIS_JOINT_MAX; i++) {
        cfg->joint_polarity[i] = 1;
    }
    for (i = 0U; i < CHASSIS_WHEEL_MAX; i++) {
        cfg->wheel_polarity[i] = 1;
    }
}

/**
 * @brief 判断电机反馈是否在超时时间内更新。
 *
 * @param fb DJI 电机反馈。
 * @param now_ms 当前时间，单位 ms。
 * @return 在线返回 1，否则返回 0。
 */
static uint8_t app_chassis_motor_recent(const DjiMotorFb *fb, uint32_t now_ms)
{
    uint32_t now_tick;

    if (fb == NULL) {
        return 0U;
    }

#if APP_CHASSIS_MOTOR_LOST_MS == 0U
    (void)now_ms;
    return 1U;
#else
    /* motor.c 的反馈时间戳来自 HAL_GetTick()，年龄判断必须使用同一时间源。 */
    (void)now_ms;
    now_tick = HAL_GetTick();
    return (((uint32_t)(now_tick - fb->last_time)) <= APP_CHASSIS_MOTOR_LOST_MS) ? 1U : 0U;
#endif
}

/**
 * @brief 读取 DJI 电机反馈基础字段。
 *
 * @param bus FDCAN 总线。
 * @param rx_id 电机反馈 ID。
 * @param now_ms 当前时间，单位 ms。
 * @param out 底盘电机反馈输出。
 * @return Devices/Motor 读取状态。
 */
static motor_status_t app_chassis_motor_fb(bsp_fdcan_id_t bus,
                                           uint16_t rx_id,
                                           uint32_t now_ms,
                                           ChassisMotorFb *out)
{
    DjiMotorFb fb;
    motor_status_t status;

    if (out == NULL) {
        return MotorNull;
    }

    (void)memset(out, 0, sizeof(*out));
    status = motor_dji_get(bus, rx_id, &fb);
    if (status != MotorOk) {
        return status;
    }
    if (app_chassis_motor_recent(&fb, now_ms) == 0U) {
        return MotorNoData;
    }

    out->ecd = fb.ecd;
    out->speed_rpm = fb.speed_rpm;
    out->current = fb.given_current;
    out->online = 1U;
    return MotorOk;
}

/**
 * @brief 填充驱动轮反馈，并保持旧舵轮底盘的 rpm 到 m/s 换算语义。
 *
 * @param now_ms 当前时间，单位 ms。
 * @param index 电机序号。
 * @param out 反馈输出。
 * @return Devices/Motor 读取状态。
 */
static motor_status_t app_chassis_drive_fb(uint32_t now_ms, uint32_t index, ChassisMotorFb *out)
{
    motor_status_t status;

    if ((index >= CHASSIS_DRIVE_MAX) || (out == NULL)) {
        return MotorArg;
    }

#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    {
        motor_mit_fb_t fb;

        (void)memset(out, 0, sizeof(*out));
        status = motor_mit_get(MOTOR_CHASSIS_BUS, app_chassis_drive_id[index], &fb);
        if (status != MotorOk) {
            return status;
        }
        if ((uint32_t)(HAL_GetTick() - fb.last_time) > ROBOT_CHASSIS_DM_FB_LOST_MS) {
            return MotorNoData;
        }
        (void)now_ms;

        /* DM 反馈是电机正方向下的 rad/s，换算成底盘 m/s；符号已包含在运动学里。 */
        out->speed = fb.fb.vel * ROBOT_CHASSIS_WHEEL_RADIUS_M * ROBOT_CHASSIS_DM_GEAR_RATIO;
        out->speed_rpm = 0;
        out->current = 0;
        out->online = 1U;
        return MotorOk;
    }
#else
    status = app_chassis_motor_fb(MOTOR_CHASSIS_BUS, app_chassis_drive_id[index], now_ms, out);
    if ((status == MotorOk) && (ROBOT_CHASSIS_MPS_TO_RPM > 0.0F)) {
        out->speed = (float32_t)out->speed_rpm / ROBOT_CHASSIS_MPS_TO_RPM;
    }

    return status;
#endif
}

/**
 * @brief 填充舵向电机反馈，并保持旧 6020 角度和速度环单位语义。
 *
 * @param now_ms 当前时间，单位 ms。
 * @param index 电机序号。
 * @param out 反馈输出。
 * @return Devices/Motor 读取状态。
 */
static motor_status_t app_chassis_steer_fb(uint32_t now_ms, uint32_t index, ChassisMotorFb *out)
{
    motor_status_t status;

    if ((index >= CHASSIS_STEER_MAX) || (out == NULL)) {
        return MotorArg;
    }

    status = app_chassis_motor_fb(MOTOR_CHASSIS_STEER_BUS, app_chassis_steer_id[index], now_ms, out);
    if (status == MotorOk) {
        out->angle = (float32_t)out->ecd / ROBOT_CHASSIS_STEER_ANGLE_RATIO;
        out->speed = (float32_t)out->speed_rpm;
    }

    return status;
}

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Chassis App 只采集输入、观测和电机反馈，运动学与 PID 留在 Controller。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/**
 * @brief 填充上层输入命令。
 *
 * @param input ChassisInput 输出。
 */
static void app_chassis_fill_cmd(ChassisInput *input)
{
    if (input == NULL) {
        return;
    }

    if (app_input_get_chassis_cmd(&input->cmd) == 0U) {
        (void)memset(&input->cmd, 0, sizeof(input->cmd));
    }
#if APP_CHASSIS_REQUIRE_GIMBAL_READY != 0U
    if (app_gimbal_ready() == 0U) {
        input->cmd.force_stop = 1U;
    }
#endif
    if (app_chassis_emergency != 0U) {
        input->cmd.force_stop = 1U;
    }
#if ROBOT_APP_ENABLE_CALIBRATION != 0U
    if (app_calibration_is_busy() != 0U) {
        input->cmd.force_stop = 1U;
    }
#endif
#if defined(APP_CHASSIS_BENCH_TEST) && (APP_CHASSIS_BENCH_TEST != 0U)
    if (app_chassis_bench_start_ms == 0U) {
        app_chassis_bench_start_ms = HAL_GetTick();
    }
    {
        uint32_t elapsed_ms = (uint32_t)(HAL_GetTick() - app_chassis_bench_start_ms);
        float32_t bench_vx = APP_CHASSIS_BENCH_VX_MPS;

        if (elapsed_ms < APP_CHASSIS_BENCH_MS) {
#if APP_CHASSIS_BENCH_RAMP_MS > 0U
            if (elapsed_ms < APP_CHASSIS_BENCH_RAMP_MS) {
                bench_vx = APP_CHASSIS_BENCH_VX_MPS *
                           ((float32_t)elapsed_ms / (float32_t)APP_CHASSIS_BENCH_RAMP_MS);
            }
#endif
            input->cmd.online = 1U;
            input->cmd.mode = ChassisModeNoFollow;
            input->cmd.vx = bench_vx;
            input->cmd.vy = 0.0F;
            input->cmd.wz = 0.0F;
        } else {
            input->cmd.force_stop = 1U;
        }
    }
#endif
}

/**
 * @brief 填充 Observe 姿态、角速度和加速度。
 *
 * @param input ChassisInput 输出。
 * @param observe_ready INS ready 标志输出。
 */
static void app_chassis_fill_observe(ChassisInput *input, uint8_t *observe_ready)
{
    ObserverState state;

    if (observe_ready != NULL) {
        *observe_ready = 0U;
    }

    if (input == NULL) {
        return;
    }

    if ((app_observe_get_state(&state) != 0U) &&
        ((state.valid_mask & OBSERVER_VALID_INS) != 0U) &&
        (state.ins.valid != 0U)) {
        input->yaw = state.ins.yaw_total;
        input->pitch = state.ins.euler.pitch;
        input->roll = state.ins.euler.roll;
        input->gyro[0] = state.ins.gyro.x;
        input->gyro[1] = state.ins.gyro.y;
        input->gyro[2] = state.ins.gyro.z;
        input->acc[0] = state.ins.accel.x;
        input->acc[1] = state.ins.accel.y;
        input->acc[2] = state.ins.accel.z;
        if (observe_ready != NULL) {
            *observe_ready = state.ins.ready;
        }
    }
}

/**
 * @brief 复制当前功率缓存。
 *
 * @param power 输出功率缓存。
 */
static void app_chassis_get_power(AppChassisPower *power)
{
    uint32_t primask;

    if (power == NULL) {
        return;
    }

    primask = app_chassis_enter();
    *power = app_chassis_power;
    app_chassis_exit(primask);
}

/**
 * @brief 填充功率预算输入。
 *
 * @param input ChassisInput 输出。
 */
static void app_chassis_fill_power(ChassisInput *input)
{
    AppChassisPower power;

    if (input == NULL) {
        return;
    }

    (void)memset(&power, 0, sizeof(power));
    app_chassis_get_power(&power);
    if (power.valid != 0U) {
        input->power = power.power;
        input->power_limit = (power.limit > 0.0F) ? power.limit : ROBOT_CHASSIS_POWER_LIMIT;
        input->buffer_energy = power.buffer;
    } else {
        input->power = 0.0F;
        input->power_limit = ROBOT_CHASSIS_POWER_LIMIT;
        input->buffer_energy = 0.0F;
    }
}

/**
 * @brief 填充完整 ChassisInput。
 *
 * @param now_us 当前时间，单位 us。
 * @param input ChassisInput 输出。
 * @param drive_status 四路驱动反馈状态。
 * @param steer_status 四路舵向反馈状态。
 * @param observe_ready INS ready 标志输出。
 */
static void app_chassis_fill_input(uint64_t now_us,
                                   ChassisInput *input,
                                   motor_status_t drive_status[CHASSIS_DRIVE_MAX],
                                   motor_status_t steer_status[CHASSIS_STEER_MAX],
                                   uint8_t *observe_ready)
{
    uint32_t i;
    uint32_t now_ms = (uint32_t)(now_us / 1000ULL);

    if ((input == NULL) || (drive_status == NULL) || (steer_status == NULL)) {
        return;
    }

    (void)memset(input, 0, sizeof(*input));
    input->tick_ms = now_ms;

    app_chassis_fill_cmd(input);
    app_chassis_fill_observe(input, observe_ready);
    app_chassis_fill_power(input);

    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        drive_status[i] = app_chassis_drive_fb(now_ms, i, &input->drive_fb[i]);
    }
    for (i = 0U; i < CHASSIS_STEER_MAX; i++) {
        steer_status[i] = app_chassis_steer_fb(now_ms, i, &input->steer_fb[i]);
    }
}

/**
 * @brief 生成驱动反馈在线 bit mask。
 *
 * @param input 周期输入。
 * @return 在线 bit mask。
 */
static uint8_t app_chassis_drive_mask(const ChassisInput *input)
{
    uint8_t mask = 0U;
    uint32_t i;

    if (input == NULL) {
        return 0U;
    }

    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        if (input->drive_fb[i].online != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

/**
 * @brief 统计驱动轮反馈在线数量。
 *
 * @param input 周期输入。
 * @return 在线轮数。
 */
static uint8_t app_chassis_drive_count(const ChassisInput *input)
{
    uint8_t count = 0U;
    uint32_t i;

    if (input == NULL) {
        return 0U;
    }

    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        if (input->drive_fb[i].online != 0U) {
            count++;
        }
    }

    return count;
}

/**
 * @brief 生成舵向反馈在线 bit mask。
 *
 * @param input 周期输入。
 * @return 在线 bit mask。
 */
static uint8_t app_chassis_steer_mask(const ChassisInput *input)
{
    uint8_t mask = 0U;
    uint32_t i;

    if (input == NULL) {
        return 0U;
    }

    for (i = 0U; i < CHASSIS_STEER_MAX; i++) {
        if (input->steer_fb[i].online != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/**
 * @brief 发送底盘输出，失败或禁发时发送零电流。
 *
 * @param output Chassis 输出。
 * @param status Chassis controller 状态。
 * @param drive_send 驱动电机发送状态输出。
 * @param steer_send 舵向电机发送状态输出。
 * @param zero_out 非 0 输出是否被清零。
 */
static void app_chassis_send(const ChassisOutput *output,
                             ChassisStatus status,
                             motor_status_t *drive_send,
                             motor_status_t *steer_send,
                             uint8_t *zero_out)
{
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    const float32_t rad_per_mps =
        1.0F / (ROBOT_CHASSIS_WHEEL_RADIUS_M * ROBOT_CHASSIS_DM_GEAR_RATIO);
    motor_status_t result = MotorOk;
    uint8_t send_ok;
    uint32_t i;

    send_ok = ((output != NULL) &&
               (status == ChassisOk) &&
               (output->send_enable != 0U)) ? 1U : 0U;
    if (zero_out != NULL) {
        *zero_out = (send_ok != 0U) ? 0U : 1U;
    }

    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        float32_t vel = 0.0F;
        motor_status_t status_i;

        if (send_ok != 0U) {
            vel = output->drive_speed_set[i] * rad_per_mps;
            if (vel > ROBOT_CHASSIS_DM_MAX_RAD_S) {
                vel = ROBOT_CHASSIS_DM_MAX_RAD_S;
            } else if (vel < -ROBOT_CHASSIS_DM_MAX_RAD_S) {
                vel = -ROBOT_CHASSIS_DM_MAX_RAD_S;
            }
        }

        status_i = motor_dm_speed(MOTOR_CHASSIS_BUS,
                                  (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0 + i),
                                  vel);
        if (status_i != MotorOk) {
            result = status_i;
        }
    }

    if (drive_send != NULL) {
        *drive_send = result;
    }
    if (steer_send != NULL) {
        *steer_send = MotorOk;
    }
#else
    int16_t drive[4] = {0, 0, 0, 0};
    int16_t steer[4] = {0, 0, 0, 0};

    if (zero_out != NULL) {
        *zero_out = 1U;
    }

    if ((output != NULL) &&
        (status == ChassisOk) &&
        (output->send_enable != 0U)) {
        uint32_t i;
        for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
            drive[i] = output->drive_current[i];
        }
        for (i = 0U; i < CHASSIS_STEER_MAX; i++) {
            steer[i] = output->steer_current[i];
        }
        if (zero_out != NULL) {
            *zero_out = 0U;
        }
    }

    if (drive_send != NULL) {
        *drive_send = motor_chassis(drive[0], drive[1], drive[2], drive[3]);
    } else {
        (void)motor_chassis(drive[0], drive[1], drive[2], drive[3]);
    }

#if CHASSIS_TYPE == CHASSIS_TYPE_STEERING
    if (steer_send != NULL) {
        *steer_send = motor_chassis_steer(steer[0], steer[1], steer[2], steer[3]);
    } else {
        (void)motor_chassis_steer(steer[0], steer[1], steer[2], steer[3]);
    }
#else
    if (steer_send != NULL) {
        *steer_send = MotorOk;
    }
#endif
#endif
}

/* ==========================================================================
 * 诊断快照
 * --------------------------------------------------------------------------
 * 周期末统一提交快照和诊断，其他模块只能读取稳定副本。
 * ========================================================================== */
/**
 * @brief 提交输出和诊断快照。
 *
 * @param input 本周期输入。
 * @param output 本周期输出。
 * @param chassis_status controller 状态。
 * @param drive_status 驱动反馈读取状态。
 * @param steer_status 舵向反馈读取状态。
 * @param drive_send 驱动发送状态。
 * @param steer_send 舵向发送状态。
 * @param observe_ready INS ready 标志。
 * @param zero_out 非 0 表示本周期发送零电流。
 * @param now_us 当前时间戳，单位 us。
 */
static void app_chassis_commit(const ChassisInput *input,
                               const ChassisOutput *output,
                               ChassisStatus chassis_status,
                               const motor_status_t drive_status[CHASSIS_DRIVE_MAX],
                               const motor_status_t steer_status[CHASSIS_STEER_MAX],
                               motor_status_t drive_send,
                               motor_status_t steer_send,
                               uint8_t observe_ready,
                               uint8_t zero_out,
                               uint64_t now_us)
{
    uint32_t primask;
    uint32_t i;
    uint8_t drive_mask;
    uint8_t steer_mask;
    AppChassisDiag diag;

    if ((input == NULL) || (output == NULL) || (drive_status == NULL) || (steer_status == NULL)) {
        return;
    }

    drive_mask = app_chassis_drive_mask(input);
    steer_mask = app_chassis_steer_mask(input);

    primask = app_chassis_enter();
    app_chassis_input = *input;
    app_chassis_output = *output;
    app_chassis_snapshot = app_chassis_ctx;
    app_chassis_diag.loop_count++;
    if (input->cmd.online == 0U) {
        app_chassis_diag.input_lost_count++;
    }
    if (observe_ready == 0U) {
        app_chassis_diag.observe_lost_count++;
    }
    if (drive_mask != APP_CHASSIS_MOTOR_ALL_MASK) {
        app_chassis_diag.drive_fb_lost_count++;
    }
#if CHASSIS_TYPE == CHASSIS_TYPE_STEERING
    if (steer_mask != APP_CHASSIS_MOTOR_ALL_MASK) {
        app_chassis_diag.steer_fb_lost_count++;
    }
#endif
    if (chassis_status != ChassisOk) {
        app_chassis_diag.chassis_fail_count++;
    }
    if ((drive_send == MotorOk) && (steer_send == MotorOk)) {
        app_chassis_diag.send_count++;
    } else {
        app_chassis_diag.send_fail_count++;
    }
    if (zero_out != 0U) {
        app_chassis_diag.zero_count++;
    }
    app_chassis_diag.cycle_overrun_count = app_chassis_overrun_count;
    app_chassis_diag.last_tick_ms = input->tick_ms;
    app_chassis_diag.last_time_us = (uint32_t)now_us;
    app_chassis_diag.chassis_status = chassis_status;
    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        app_chassis_diag.drive_status[i] = drive_status[i];
        app_chassis_diag.drive_current[i] = output->drive_current[i];
        app_chassis_diag.drive_speed_cmd[i] = output->drive_speed_set[i];
        app_chassis_diag.drive_speed_fb[i] = input->drive_fb[i].speed;
    }
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    app_chassis_diag.dm_fault_latched = app_chassis_dm_fault_latched;
    app_chassis_diag.dm_fault_count = app_chassis_dm_fault_count;
    app_chassis_diag.dm_tx_drop_count = app_chassis_dm_tx_drop_count;
    app_chassis_diag.dm_enable_count = app_chassis_dm_enable_count;
    app_chassis_diag.dm_enable_fail_count = app_chassis_dm_enable_fail_count;
    app_chassis_diag.dm_mode_ok_count = app_chassis_dm_mode_ok_count;
    app_chassis_diag.dm_mode_fail_count = app_chassis_dm_mode_fail_count;
    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        app_chassis_diag.dm_mode_value[i] = app_chassis_dm_mode_value[i];
    }
    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        app_chassis_diag.drive_speed_fb_min[i] = app_chassis_dm_fb_min[i];
        app_chassis_diag.drive_speed_fb_max[i] = app_chassis_dm_fb_max[i];
        app_chassis_diag.drive_offline_count[i] = app_chassis_drive_offline_count[i];
        app_chassis_diag.drive_first_move_ms[i] = app_chassis_drive_first_move_ms[i];
        app_chassis_diag.drive_first_full_ms[i] = app_chassis_drive_first_full_ms[i];
        app_chassis_diag.drive_first_enabled_ms[i] = app_chassis_drive_first_enabled_ms[i];
    }
    app_chassis_diag.dm_fault_mask = app_chassis_dm_fault_mask;
#endif
    for (i = 0U; i < CHASSIS_STEER_MAX; i++) {
        app_chassis_diag.steer_status[i] = steer_status[i];
        app_chassis_diag.steer_current[i] = output->steer_current[i];
    }
    app_chassis_diag.drive_send_status = drive_send;
    app_chassis_diag.steer_send_status = steer_send;
    app_chassis_diag.mode = output->mode;
    app_chassis_diag.input_online = input->cmd.online;
    app_chassis_diag.observe_ready = observe_ready;
    app_chassis_diag.drive_online_mask = drive_mask;
    app_chassis_diag.steer_online_mask = steer_mask;
    app_chassis_diag.send_enable = output->send_enable;
    diag = app_chassis_diag;
    app_chassis_exit(primask);

    DBG_BOOL("app.chassis.in", diag.input_online != 0U);
    DBG_U32("app.chassis.st", (uint32_t)diag.chassis_status);
    DBG_F32("app.chassis.d0", (float32_t)diag.drive_current[0]);
    DBG_F32("app.chassis.s0", (float32_t)diag.steer_current[0]);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Chassis 任务入口。
 *
 * @param argument CMSIS-RTOS2 输入参数。
 */
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
/**
 * @brief 读取一个电机的 CTRL_MODE 并等待新应答。
 *
 * @param id 电机 ID。
 * @param value 读回值输出。
 * @param timeout_ms 等待时间，单位 ms。
 * @return 1 表示读到新应答，0 表示超时。
 */
static uint8_t app_chassis_dm_read_mode(uint16_t id, uint32_t *value, uint32_t timeout_ms)
{
    uint32_t seq = motor_dm_param_seq();
    uint32_t waited = 0U;
    uint32_t read_value = 0xFFFFFFFFU;

    (void)motor_dm_param_cmd(MOTOR_CHASSIS_BUS, id, 0x33U, 0x0AU, 0U);
    while (waited < timeout_ms) {
        osDelay(5U);
        waited += 5U;
        if ((motor_dm_param_seq() != seq) &&
            (motor_dm_param_get(id, 0x0AU, &read_value) != 0U)) {
            if (value != NULL) {
                *value = read_value;
            }
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 将四个驱动轮切换到达妙原生速度模式并读回确认。
 *
 * 6c5c78b 基线在跑速度前会写 CTRL_MODE=3（0x55 / RID 0x0A）、读回确认，
 * 再保存参数。缺少这一步时，模式不是 3 的电机仍会发反馈，但会忽略
 * ID+0x200 的使能和速度帧，表现为“在线但不动”。
 *
 * 这里先读后写：已经是 3 的电机不再写参数，避免不必要的模式写入导致
 * ID 重排或启动期反馈短暂中断。
 */
static void app_chassis_dm_set_speed_mode(void)
{
    uint32_t i;
    uint32_t attempt;
    uint32_t ok_count = 0U;
    uint32_t written_count = 0U;

    app_chassis_dm_mode_ok_count = 0U;
    app_chassis_dm_mode_fail_count = 0U;

    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        uint16_t id = (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0 + i);
        uint32_t value = 0xFFFFFFFFU;

        app_chassis_dm_mode_value[i] = 0xFFFFFFFFU;

        /* 先读两次，避免单次应答丢失就误判模式不对。 */
        for (attempt = 0U; attempt < 2U; attempt++) {
            if ((app_chassis_dm_read_mode(id, &value, 300U) != 0U) && (value == 3U)) {
                app_chassis_dm_mode_value[i] = value;
                break;
            }
        }

        if (app_chassis_dm_mode_value[i] != 3U) {
            /* 不是 3 或读不到：写入后再读回，最多两轮。 */
            for (attempt = 0U; attempt < 2U; attempt++) {
                (void)motor_dm_param_cmd(MOTOR_CHASSIS_BUS, id, 0x55U, 0x0AU, 3U);
                osDelay(50U);
                value = 0xFFFFFFFFU;
                if ((app_chassis_dm_read_mode(id, &value, 300U) != 0U) &&
                    (value == 3U)) {
                    app_chassis_dm_mode_value[i] = value;
                    written_count++;
                    break;
                }
                app_chassis_dm_mode_value[i] = value;
            }
        }

        if (app_chassis_dm_mode_value[i] == 3U) {
            ok_count++;
        }
    }

    app_chassis_dm_mode_ok_count = ok_count;
    app_chassis_dm_mode_fail_count = (uint32_t)CHASSIS_DRIVE_MAX - ok_count;

    if ((ok_count == (uint32_t)CHASSIS_DRIVE_MAX) && (written_count > 0U)) {
        for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
            (void)motor_dm_param_cmd(MOTOR_CHASSIS_BUS,
                                     (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0 + i),
                                     0xAAU, 0x01U, 0U);
        }
        osDelay(100U);
    }
}

/**
 * @brief 向四个驱动轮重发一次使能帧。
 *
 * 达妙速度模式在漏收使能或通讯中断后会保持失能；基线 6c5c78b 在运行中
 * 每 500 ms 重发一次使能，这里保持同样的行为，避免单帧丢失导致某一轮不转。
 *
 * @param now_ms 当前 HAL tick，单位 ms。
 */
static void app_chassis_dm_enable_all(uint32_t now_ms)
{
    uint32_t i;

    app_chassis_dm_last_enable_ms = now_ms;
    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        if (motor_dm_speed_cmd(MOTOR_CHASSIS_BUS,
                               (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0 + i),
                               DmMotorEnable) == MotorOk) {
            app_chassis_dm_enable_count++;
        } else {
            app_chassis_dm_enable_fail_count++;
        }
    }
}

static void app_chassis_dm_start(void)
{
    motor_mit_fb_t fb;
    uint32_t i;
    uint32_t round;
    uint32_t online;
    uint8_t ready;

    app_chassis_dm_ready = 0U;
    app_chassis_dm_fault_latched = 0U;
    app_chassis_dm_fault_since_ms = 0U;
    app_chassis_dm_last_stop_ms = 0U;
    app_chassis_dm_last_enable_ms = 0U;
    app_chassis_dm_enable_count = 0U;
    app_chassis_dm_enable_fail_count = 0U;
    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        app_chassis_dm_fb_min[i] = 0.0F;
        app_chassis_dm_fb_max[i] = 0.0F;
        app_chassis_dm_fb_seen[i] = 0U;
    }

    for (round = 0U; round < 5U; round++) {
        for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
            (void)motor_dm_speed_stop(MOTOR_CHASSIS_BUS,
                                      (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0 + i));
        }
        osDelay(20U);
    }
    osDelay(200U);

    /* 先确认四轮都在原生速度模式，再使能；否则 ID+0x200 会被忽略。 */
    app_chassis_dm_set_speed_mode();

    /* 与基线一致：使能连发多轮，每轮同时把速度目标清零。 */
    for (round = 0U; round < 3U; round++) {
        app_chassis_dm_enable_all(HAL_GetTick());
        for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
            (void)motor_dm_speed(MOTOR_CHASSIS_BUS,
                                 (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0 + i), 0.0F);
        }
        osDelay(100U);
    }

    /* 等待四路反馈有效后再允许底盘进入运行态；期间周期性重发使能。 */
    ready = 0U;
    for (round = 0U; round < 200U; round++) {
        if ((round % 50U) == 0U) {
            app_chassis_dm_enable_all(HAL_GetTick());
        }
        online = 0U;
        for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
            if ((motor_mit_get(MOTOR_CHASSIS_BUS,
                               (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_0 + i),
                               &fb) == MotorOk) &&
                ((uint32_t)(HAL_GetTick() - fb.last_time) <= ROBOT_CHASSIS_DM_FB_LOST_MS)) {
                online++;
            }
        }
        app_chassis_heartbeat_ms = HAL_GetTick();
        if (online >= (uint32_t)app_chassis_ctx.cfg.min_online_motors) {
            ready = 1U;
            break;
        }
        osDelay(10U);
    }

    app_chassis_dm_start_ms = HAL_GetTick();
    app_chassis_dm_last_enable_ms = HAL_GetTick();
    app_chassis_dm_ready = ready;
}

static void app_chassis_dm_force_stop(uint32_t now_ms)
{
    uint32_t i;

    if ((uint32_t)(now_ms - app_chassis_dm_last_stop_ms) < ROBOT_CHASSIS_DM_STOP_PERIOD_MS) {
        return;
    }
    app_chassis_dm_last_stop_ms = now_ms;

    if (app_chassis_dm_fault_latched == 0U) {
        app_chassis_dm_fault_latched = 1U;
        app_chassis_dm_fault_count++;
    }

    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        (void)motor_dm_speed_stop(MOTOR_CHASSIS_BUS,
                                  (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0 + i));
    }
}
#endif

static void app_chassis_entry(void *argument)
{
    ChassisInput input;
    ChassisOutput output;
    ChassisStatus chassis_status;
    motor_status_t drive_status[CHASSIS_DRIVE_MAX];
    motor_status_t steer_status[CHASSIS_STEER_MAX];
    motor_status_t drive_send;
    motor_status_t steer_send;
    uint64_t now_us;
    uint32_t now_ms;
    uint8_t observe_ready;
    uint8_t zero_out;
    uint8_t dm_fault = 0U;
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    uint32_t dm_now_ms = 0U;
    uint32_t i;
#endif
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;

#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    app_chassis_dm_start();
#endif

    while (1) {
        now_us = bsp_timebase_now_us();
        now_ms = (uint32_t)(now_us / 1000ULL);
        app_chassis_heartbeat_ms = now_ms;
        app_chassis_fill_input(now_us, &input, drive_status, steer_status, &observe_ready);

#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
        /* DM 的反馈时间戳来自 HAL_GetTick()，所有 DM 计时统一用同一时间源。 */
        dm_now_ms = HAL_GetTick();
        for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
            if (input.drive_fb[i].online != 0U) {
                float32_t fb_speed = input.drive_fb[i].speed;

                if (app_chassis_dm_fb_seen[i] == 0U) {
                    app_chassis_dm_fb_seen[i] = 1U;
                    app_chassis_dm_fb_min[i] = fb_speed;
                    app_chassis_dm_fb_max[i] = fb_speed;
                } else {
                    if (fb_speed < app_chassis_dm_fb_min[i]) {
                        app_chassis_dm_fb_min[i] = fb_speed;
                    }
                    if (fb_speed > app_chassis_dm_fb_max[i]) {
                        app_chassis_dm_fb_max[i] = fb_speed;
                    }
                }
            } else {
                app_chassis_drive_offline_count[i]++;
            }
        }
        dm_fault = ((app_chassis_dm_ready == 0U) ||
                    (app_chassis_dm_fault_latched != 0U) ||
                    (app_chassis_emergency != 0U) ||
                    (app_chassis_drive_count(&input) <
                     app_chassis_ctx.cfg.min_online_motors))
                       ? 1U
                       : 0U;
        if ((dm_fault != 0U) && (app_chassis_dm_fault_since_ms == 0U)) {
            app_chassis_dm_fault_mask = app_chassis_drive_mask(&input);
        }
#else
        dm_fault = 0U;
#endif

        (void)memset(&output, 0, sizeof(output));
        chassis_status = chassis_step(&app_chassis_ctx, &input, &output);
        if ((chassis_status != ChassisOk) || (output.send_enable == 0U) || (dm_fault != 0U)) {
            (void)memset(&output, 0, sizeof(output));
        }

#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
        /* 运行中周期性重发使能，补偿单帧丢失；故障锁存后不再使能。 */
        if ((dm_fault == 0U) &&
            (app_chassis_dm_fault_latched == 0U) &&
            ((uint32_t)(dm_now_ms - app_chassis_dm_last_enable_ms) >=
             ROBOT_CHASSIS_DM_ENABLE_PERIOD_MS)) {
            app_chassis_dm_enable_all(dm_now_ms);
        }

        /* 记录每轮首次使能、首次起转和首次达到目标 80% 的时刻。 */
        for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
            if (app_chassis_drive_first_enabled_seen[i] == 0U) {
                motor_mit_fb_t fb_state;

                if ((motor_mit_get(MOTOR_CHASSIS_BUS,
                                   (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_0 + i),
                                   &fb_state) == MotorOk) &&
                    (fb_state.fb.state == 1U)) {
                    app_chassis_drive_first_enabled_seen[i] = 1U;
                    app_chassis_drive_first_enabled_ms[i] =
                        (uint32_t)(dm_now_ms - app_chassis_dm_start_ms);
                }
            }

            if (input.drive_fb[i].online != 0U) {
                float32_t fb_speed = input.drive_fb[i].speed;
                float32_t cmd_speed = output.drive_speed_set[i];

                if ((app_chassis_drive_first_move_seen[i] == 0U) &&
                    (fabsf(fb_speed) > 0.01F)) {
                    app_chassis_drive_first_move_seen[i] = 1U;
                    app_chassis_drive_first_move_ms[i] =
                        (uint32_t)(dm_now_ms - app_chassis_dm_start_ms);
                }
                if ((app_chassis_drive_first_full_seen[i] == 0U) &&
                    (fabsf(cmd_speed) > 0.01F) &&
                    (fabsf(fb_speed) >= (0.8F * fabsf(cmd_speed)))) {
                    app_chassis_drive_first_full_seen[i] = 1U;
                    app_chassis_drive_first_full_ms[i] =
                        (uint32_t)(dm_now_ms - app_chassis_dm_start_ms);
                }
            }
        }
#endif

        app_chassis_send(&output, chassis_status, &drive_send, &steer_send, &zero_out);

#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
        if (drive_send != MotorOk) {
            app_chassis_dm_tx_drop_count++;
        }
        if (dm_fault != 0U) {
            if (app_chassis_dm_fault_since_ms == 0U) {
                app_chassis_dm_fault_since_ms = dm_now_ms;
            }
            /* 启动后 2000 ms 内允许反馈短暂不完整，不锁存故障。 */
            if (((uint32_t)(dm_now_ms - app_chassis_dm_start_ms) >= 2000U) &&
                ((uint32_t)(dm_now_ms - app_chassis_dm_fault_since_ms) >=
                 ROBOT_CHASSIS_DM_DISABLE_AFTER_MS)) {
                app_chassis_dm_force_stop(dm_now_ms);
            }
        } else {
            app_chassis_dm_fault_since_ms = 0U;
        }
        if (app_chassis_dm_fault_latched != 0U) {
            app_chassis_dm_force_stop(dm_now_ms);
        }
#endif

        app_chassis_commit(&input,
                           &output,
                           chassis_status,
                           drive_status,
                           steer_status,
                           drive_send,
                           steer_send,
                           observe_ready,
                           zero_out,
                           now_us);

        wake_tick += APP_CHASSIS_MS_TO_TICKS(APP_CHASSIS_PERIOD_MS);
        if ((int32_t)(osKernelGetTickCount() - wake_tick) > 0) {
            app_chassis_overrun_count++;
            wake_tick = osKernelGetTickCount();
        }
        (void)osDelayUntil(wake_tick);
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_chassis_init(void)
{
    ChassisCfg cfg;
    ChassisStatus status;

    (void)memset(&app_chassis_ctx, 0, sizeof(app_chassis_ctx));
    (void)memset(&app_chassis_snapshot, 0, sizeof(app_chassis_snapshot));
    (void)memset(&app_chassis_input, 0, sizeof(app_chassis_input));
    (void)memset(&app_chassis_output, 0, sizeof(app_chassis_output));
    (void)memset(&app_chassis_diag, 0, sizeof(app_chassis_diag));
    (void)memset(&app_chassis_power, 0, sizeof(app_chassis_power));
    (void)memset(&app_chassis_impl, 0, sizeof(app_chassis_impl));

    app_chassis_heartbeat_ms = HAL_GetTick();
    app_chassis_overrun_count = 0U;
    app_chassis_emergency = 0U;
#if defined(ROBOT_CHASSIS_DRIVE_MODE_DM_SPEED)
    app_chassis_dm_ready = 0U;
    app_chassis_dm_fault_since_ms = 0U;
    app_chassis_dm_last_stop_ms = 0U;
    app_chassis_dm_fault_latched = 0U;
    app_chassis_dm_fault_count = 0U;
    app_chassis_dm_tx_drop_count = 0U;
    app_chassis_dm_last_enable_ms = 0U;
    app_chassis_dm_enable_count = 0U;
    app_chassis_dm_enable_fail_count = 0U;
    app_chassis_dm_mode_ok_count = 0U;
    app_chassis_dm_mode_fail_count = 0U;
    app_chassis_dm_fault_mask = 0U;
    for (uint32_t i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        app_chassis_dm_fb_min[i] = 0.0F;
        app_chassis_dm_fb_max[i] = 0.0F;
        app_chassis_dm_fb_seen[i] = 0U;
        app_chassis_dm_mode_value[i] = 0xFFFFFFFFU;
        app_chassis_drive_offline_count[i] = 0U;
        app_chassis_drive_first_move_ms[i] = 0xFFFFFFFFU;
        app_chassis_drive_first_full_ms[i] = 0xFFFFFFFFU;
        app_chassis_drive_first_enabled_ms[i] = 0xFFFFFFFFU;
        app_chassis_drive_first_move_seen[i] = 0U;
        app_chassis_drive_first_full_seen[i] = 0U;
        app_chassis_drive_first_enabled_seen[i] = 0U;
    }
#endif

    app_chassis_make_cfg(&cfg);
    status = chassis_init(&app_chassis_ctx, &cfg, &app_chassis_impl);
    app_chassis_diag.chassis_status = status;
    if (status != ChassisOk) {
        return 0U;
    }

    app_chassis_snapshot = app_chassis_ctx;
    app_chassis_thread = osThreadNew(app_chassis_entry, NULL, &app_chassis_attr);
    return (app_chassis_thread != NULL) ? 1U : 0U;
}

uint8_t app_chassis_get_state(Chassis *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_chassis_enter();
    *out = app_chassis_snapshot;
    app_chassis_exit(primask);
    return 1U;
}

uint8_t app_chassis_get_input(ChassisInput *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_chassis_enter();
    *out = app_chassis_input;
    app_chassis_exit(primask);
    return 1U;
}

uint8_t app_chassis_get_output(ChassisOutput *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_chassis_enter();
    *out = app_chassis_output;
    app_chassis_exit(primask);
    return 1U;
}

uint8_t app_chassis_get_diag(AppChassisDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_chassis_enter();
    *out = app_chassis_diag;
    app_chassis_exit(primask);
    return 1U;
}

uint8_t app_chassis_set_power(float32_t power, float32_t limit, float32_t buffer)
{
    uint32_t primask;

    if ((isfinite(power) == 0) || (isfinite(limit) == 0) || (isfinite(buffer) == 0)) {
        return 0U;
    }

    primask = app_chassis_enter();
    app_chassis_power.power = power;
    app_chassis_power.limit = limit;
    app_chassis_power.buffer = buffer;
    app_chassis_power.valid = 1U;
    app_chassis_exit(primask);

    return 1U;
}

uint8_t app_chassis_heartbeat_alive(uint32_t now_ms)
{
    return (((uint32_t)(now_ms - app_chassis_heartbeat_ms) <= APP_CHASSIS_CMD_STALE_MS)
                ? 1U
                : 0U);
}

void app_chassis_emergency_stop(void)
{
    app_chassis_emergency = 1U;
}
