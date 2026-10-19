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
#include "bsp_timebase.h"
#include "debug.h"
#include "main.h"
#include "cmsis_os.h"

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
    MOTOR_CHASSIS_DRIVE_0_ID,
    MOTOR_CHASSIS_DRIVE_1_ID,
    MOTOR_CHASSIS_DRIVE_2_ID,
    MOTOR_CHASSIS_DRIVE_3_ID,
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
    cfg->drive_rpm_to_speed = ROBOT_CHASSIS_RPM_TO_VECTOR;
    cfg->steer_rpm_to_speed = 1.0F;
    cfg->steer_ecd_to_rad = 1.0F / ROBOT_CHASSIS_STEER_ANGLE_RATIO;
    cfg->max_drive_speed = ROBOT_CHASSIS_MAX_WHEEL_SPEED;
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
    if (fb == NULL) {
        return 0U;
    }

#if APP_CHASSIS_MOTOR_LOST_MS == 0U
    (void)now_ms;
    return 1U;
#else
    return (((uint32_t)(now_ms - fb->last_time)) <= APP_CHASSIS_MOTOR_LOST_MS) ? 1U : 0U;
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

    status = app_chassis_motor_fb(MOTOR_CHASSIS_BUS, app_chassis_drive_id[index], now_ms, out);
    if ((status == MotorOk) && (ROBOT_CHASSIS_MPS_TO_RPM > 0.0F)) {
        out->speed = (float32_t)out->speed_rpm / ROBOT_CHASSIS_MPS_TO_RPM;
    }

    return status;
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
    if (app_gimbal_ready() == 0U) {
        input->cmd.force_stop = 1U;
    }
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
    app_chassis_diag.last_tick_ms = input->tick_ms;
    app_chassis_diag.last_time_us = (uint32_t)now_us;
    app_chassis_diag.chassis_status = chassis_status;
    for (i = 0U; i < CHASSIS_DRIVE_MAX; i++) {
        app_chassis_diag.drive_status[i] = drive_status[i];
        app_chassis_diag.drive_current[i] = output->drive_current[i];
    }
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
    uint8_t observe_ready;
    uint8_t zero_out;

    (void)argument;

    while (1) {
        now_us = bsp_timebase_now_us();
        app_chassis_fill_input(now_us, &input, drive_status, steer_status, &observe_ready);

        (void)memset(&output, 0, sizeof(output));
        chassis_status = chassis_step(&app_chassis_ctx, &input, &output);
        if ((chassis_status != ChassisOk) || (output.send_enable == 0U)) {
            (void)memset(&output, 0, sizeof(output));
        }

        app_chassis_send(&output, chassis_status, &drive_send, &steer_send, &zero_out);
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

        osDelay(APP_CHASSIS_MS_TO_TICKS(APP_CHASSIS_PERIOD_MS));
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
