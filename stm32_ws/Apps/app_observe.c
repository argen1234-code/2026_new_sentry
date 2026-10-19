/**
 ******************************************************************************
 * @file    app_observe.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Observe 应用层实现。
 *
 * @note    本文件只做 IMU 设备读取、Observer 输入注入、温控输出写入和
 *          状态快照缓存，不实现姿态或速度估计算法。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_observe.h"

#include "app_observe_config.h"
#if ROBOT_APP_ENABLE_CALIBRATION != 0U
#include "app_calibration.h"
#endif
#include "bsp_dwt.h"
#include "bsp_timebase.h"
#include "debug.h"
#include "imu.h"
#include "imu_heat.h"
#include "main.h"
#include "thermal.h"
#include "cmsis_os.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 三轴全部有效的掩码。 */
#define APP_OBSERVE_AXIS_MASK \
    ((uint8_t)(OBSERVER_AXIS_X | OBSERVER_AXIS_Y | OBSERVER_AXIS_Z))

/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_OBSERVE_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/**
 * @brief 底盘观测测量缓存。
 */
typedef struct
{
    Vec3 vel;              /**< 底盘速度测量。 */
    Vec3 acc;              /**< 底盘加速度测量。 */
    uint32_t timestamp_us; /**< 测量时间戳低 32 位，单位 us。 */
    uint8_t vel_mask;      /**< 速度有效轴掩码。 */
    uint8_t acc_mask;      /**< 加速度有效轴掩码。 */
} AppObserveChassisMeasure;

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 观测与温控 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Observe 任务控制块。 */
static osThreadId_t app_observe_thread;

/** @brief Observe 任务属性。 */
static const osThreadAttr_t app_observe_attr = {
    .name = "app_observe",
    .stack_size = APP_OBSERVE_STACK_SIZE,
    .priority = (osPriority_t)APP_OBSERVE_PRIORITY,
};

/** @brief Observer 组件上下文。 */
static Observer app_observe_ctx;

/** @brief Thermal 组件上下文。 */
static Thermal app_observe_thermal;

/** @brief DWT 周期记录。 */
static uint32_t app_observe_dwt_count;

/** @brief 最新 Observer 状态快照。 */
static ObserverState app_observe_state;

/** @brief Observe 诊断快照。 */
static AppObserveDiag app_observe_diag;

/** @brief 最新底盘测量注入缓存。 */
static AppObserveChassisMeasure app_observe_chassis;

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Observe App 读取 IMU、执行 Observer step，并把 IMU 温控拆到 Thermal + imu_heat。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/** @brief 非 0 表示 IMU 加热设备可用。 */
static uint8_t app_observe_heat_ready;

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
static uint32_t app_observe_enter(void)
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
static void app_observe_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* ==========================================================================
 * 配置生成与参数展开
 * --------------------------------------------------------------------------
 * 本区把 Robot target 宏展开成 Component/Device 的 Cfg，运行期控制逻辑不写在配置头里。
 * ========================================================================== */
/**
 * @brief 生成 Observer 默认配置。
 *
 * @param cfg 输出配置。
 */
static void app_observe_make_cfg(ObserverCfg *cfg)
{
    if (cfg == NULL) {
        return;
    }

    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->mahony.kp = APP_OBSERVE_MAHONY_KP;
    cfg->mahony.ki = APP_OBSERVE_MAHONY_KI;
    cfg->mahony.dt = APP_OBSERVE_DEFAULT_DT;
    cfg->gravity = APP_OBSERVE_GRAVITY;
    cfg->ready_time = APP_OBSERVE_READY_TIME;
    cfg->stable_gyro = APP_OBSERVE_STABLE_GYRO;
    cfg->stable_accel = APP_OBSERVE_STABLE_ACCEL;
    cfg->accel_lpf = APP_OBSERVE_ACCEL_LPF;
    cfg->accel_dead_xy = APP_OBSERVE_ACCEL_DEAD_XY;
    cfg->accel_dead_z = APP_OBSERVE_ACCEL_DEAD_Z;
    cfg->gyro_cali_samples = APP_OBSERVE_GYRO_CALI_SAMPLES;
    cfg->cali_gyro_th = APP_OBSERVE_CALI_GYRO_TH;
    cfg->cali_accel_th = APP_OBSERVE_CALI_ACCEL_TH;
    cfg->kf_p = APP_OBSERVE_KF_P;
    cfg->kf_q = APP_OBSERVE_KF_Q;
    cfg->kf_r = APP_OBSERVE_KF_R;
    cfg->kf_dt = APP_OBSERVE_KF_DT;
}

/**
 * @brief 生成 Thermal 默认配置。
 *
 * @param heat_max 加热 compare 上限。
 * @param target 温控目标温度，单位摄氏度。
 * @param cfg 输出配置。
 */
static void app_observe_make_thermal_cfg(uint32_t heat_max, float32_t target, ThermalCfg *cfg)
{
    if (cfg == NULL) {
        return;
    }

    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->pid.kp = APP_OBSERVE_TEMP_KP;
    cfg->pid.ki = APP_OBSERVE_TEMP_KI;
    cfg->pid.kd = APP_OBSERVE_TEMP_KD;
    cfg->pid.max_out = (float32_t)heat_max;
    cfg->pid.max_iout = (float32_t)heat_max;
    cfg->target = target;
    cfg->deadband = APP_OBSERVE_TEMP_DEADBAND;
    cfg->period = APP_OBSERVE_TEMP_PERIOD;
    cfg->compare_min = 0.0F;
    cfg->compare_max = (float32_t)heat_max;
    cfg->compare_slew = APP_OBSERVE_HEAT_SLEW;
}

/**
 * @brief 判断温控目标温度是否可用。
 *
 * @param temp 温度值，单位摄氏度。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t app_observe_temp_valid(float32_t temp)
{
    if ((isfinite(temp) == 0) ||
        (temp < 30.0F) ||
        (temp > APP_OBSERVE_TEMP_SAFE_MAX)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 获取温控目标温度。
 *
 * @param cal_temp IMU 标定温度。
 * @return 温控目标温度，单位摄氏度。
 */
static float32_t app_observe_temp_target(float32_t cal_temp)
{
#if ROBOT_APP_ENABLE_CALIBRATION != 0U
    AppCaliHead head;

    if ((app_calibration_get_head(&head) != 0U) &&
        (app_observe_temp_valid((float32_t)head.imu_target_temp) != 0U)) {
        return (float32_t)head.imu_target_temp;
    }
#endif

    if (app_observe_temp_valid(cal_temp) != 0U) {
        return cal_temp;
    }

    return ROBOT_THERMAL_IMU_TARGET_TEMP;
}

/**
 * @brief 获取本周期 dt。
 *
 * @param stale 调度断更标志输出。
 * @return 有效 dt，单位 s。
 */
static float32_t app_observe_dt(uint8_t *stale)
{
    float32_t dt = bsp_dwt_get_delta_t(&app_observe_dwt_count);

    if (stale != NULL) {
        *stale = 0U;
    }

    if ((dt <= 0.0F) || (isfinite(dt) == 0) || (dt > APP_OBSERVE_STALE_DT)) {
        if (stale != NULL) {
            *stale = 1U;
        }
        return APP_OBSERVE_DEFAULT_DT;
    }

    return dt;
}

/**
 * @brief 复制当前底盘测量缓存。
 *
 * @param out 输出测量缓存。
 */
static void app_observe_get_chassis(AppObserveChassisMeasure *out)
{
    uint32_t primask;

    if (out == NULL) {
        return;
    }

    primask = app_observe_enter();
    *out = app_observe_chassis;
    app_observe_exit(primask);
}

/**
 * @brief 填充 Observer 周期输入。
 *
 * @param sample IMU 采样值。
 * @param imu_status IMU 读取状态。
 * @param dt 本周期 dt，单位 s。
 * @param timestamp_us 周期时间戳低 32 位，单位 us。
 * @param input 输出 ObserverInput。
 */
static void app_observe_fill_input(const imu_sample_t *sample,
                                   imu_status_t imu_status,
                                   float32_t dt,
                                   uint32_t timestamp_us,
                                   ObserverInput *input)
{
    AppObserveChassisMeasure chassis;

    if (input == NULL) {
        return;
    }

    (void)memset(input, 0, sizeof(*input));
    input->imu.dt = dt;
    input->imu.timestamp_us = timestamp_us;
    input->timestamp_us = timestamp_us;

    if ((sample != NULL) && (imu_status == IMU_OK)) {
        input->imu.online = 1U;
        input->imu.accel.x = sample->accel[0];
        input->imu.accel.y = sample->accel[1];
        input->imu.accel.z = sample->accel[2];
        input->imu.gyro.x = sample->gyro[0];
        input->imu.gyro.y = sample->gyro[1];
        input->imu.gyro.z = sample->gyro[2];
        input->imu.temperature = sample->temperature;
    }

    app_observe_get_chassis(&chassis);
    if ((app_observe_state.valid_mask & OBSERVER_VALID_INS) != 0U &&
        (app_observe_state.ins.ready != 0U)) {
        input->chassis_vel = chassis.vel;
        input->chassis_acc = chassis.acc;
        input->vel_valid_mask = (uint8_t)(chassis.vel_mask & APP_OBSERVE_AXIS_MASK);
        input->acc_valid_mask = (uint8_t)(chassis.acc_mask & APP_OBSERVE_AXIS_MASK);
        if (chassis.timestamp_us != 0U) {
            input->timestamp_us = chassis.timestamp_us;
        }
    }
}

/**
 * @brief 判断本周期是否允许加热。
 *
 * @param sample IMU 采样值。
 * @param imu_status IMU 读取状态。
 * @param stale 非 0 表示调度断更。
 * @return 允许加热返回 1，否则返回 0。
 */
static uint8_t app_observe_heat_ok(const imu_sample_t *sample,
                                   imu_status_t imu_status,
                                   uint8_t stale)
{
    if (APP_OBSERVE_HEAT_ENABLE == 0U) {
        return 0U;
    }

    if ((app_observe_heat_ready == 0U) ||
        (sample == NULL) ||
        (imu_status != IMU_OK) ||
        (stale != 0U)) {
        return 0U;
    }

    if ((isfinite(sample->temperature) == 0) ||
        (sample->temperature >= APP_OBSERVE_TEMP_SAFE_MAX)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 更新 IMU 温控输出。
 *
 * @param sample IMU 采样值。
 * @param imu_status IMU 读取状态。
 * @param dt 本周期 dt，单位 s。
 * @param stale 非 0 表示调度断更。
 * @param output 输出温控结果。
 * @return Thermal 执行状态。
 */
static ThermalStatus app_observe_heat_step(const imu_sample_t *sample,
                                           imu_status_t imu_status,
                                           float32_t dt,
                                           uint8_t stale,
                                           ThermalOutput *output)
{
    ThermalInput input;
    ThermalStatus status;
    uint8_t heat_ok;

    if (output == NULL) {
        return ThermalNull;
    }

    (void)memset(output, 0, sizeof(*output));
    heat_ok = app_observe_heat_ok(sample, imu_status, stale);
    if (heat_ok == 0U) {
        (void)thermal_reset(&app_observe_thermal);
        (void)imu_heat_off();
        output->target = app_observe_thermal.cfg.target;
        if (sample != NULL) {
            output->temperature = sample->temperature;
        }
        return ThermalOk;
    }

    input.temperature = sample->temperature;
    input.dt = dt;
    input.online = 1U;
    status = thermal_step(&app_observe_thermal, &input, output);
    if ((status == ThermalOk) && (output->enable != 0U)) {
        if (imu_heat_set(output->compare) != HAL_OK) {
            output->compare = 0U;
            output->compare_f = 0.0F;
            output->enable = 0U;
            status = ThermalArg;
        }
    } else {
        (void)imu_heat_off();
    }

    return status;
}

/**
 * @brief 更新 Observe 调试探针。
 *
 * @param state Observer 状态。
 * @param diag 诊断状态。
 */
static void app_observe_probe(const ObserverState *state, const AppObserveDiag *diag)
{
    uint64_t ts;

    if ((state == NULL) || (diag == NULL)) {
        return;
    }

    ts = (uint64_t)diag->last_time_us;
    DBG_BOOL_TS("app.observe.ready", state->ins.ready != 0U, ts);
    DBG_U32_TS("app.observe.status", (uint32_t)diag->status, ts);
    DBG_F32_TS("app.observe.yaw", state->ins.euler.yaw, ts);
    DBG_F32_TS("app.observe.pitch", state->ins.euler.pitch, ts);
    DBG_F32_TS("app.observe.roll", state->ins.euler.roll, ts);
    DBG_F32_TS("app.observe.temp", diag->temperature, ts);
    DBG_F32_TS("app.observe.heat", diag->heat_compare, ts);
}

/* ==========================================================================
 * 诊断快照
 * --------------------------------------------------------------------------
 * 周期末统一提交快照和诊断，其他模块只能读取稳定副本。
 * ========================================================================== */
/**
 * @brief 提交 Observe 状态和诊断快照。
 *
 * @param input Observer 周期输入。
 * @param state Observer 输出状态。
 * @param output Thermal 输出状态。
 * @param obs_status Observer 状态。
 * @param thermal_status Thermal 状态。
 * @param imu_status IMU 读取状态。
 * @param stale 非 0 表示调度断更。
 */
static void app_observe_commit(const ObserverInput *input,
                               const ObserverState *state,
                               const ThermalOutput *output,
                               ObserverStatus obs_status,
                               ThermalStatus thermal_status,
                               imu_status_t imu_status,
                               uint8_t stale)
{
    uint32_t primask;
    AppObserveDiag diag;

    if ((input == NULL) || (state == NULL) || (output == NULL)) {
        return;
    }

    primask = app_observe_enter();
    app_observe_state = *state;
    app_observe_diag.loop_count++;
    if (stale != 0U) {
        app_observe_diag.stale_count++;
    }
    if (imu_status != IMU_OK) {
        app_observe_diag.imu_lost_count++;
    }
    if (obs_status != ObserverOk) {
        app_observe_diag.observe_fail_count++;
    }
    if (output->enable == 0U) {
        app_observe_diag.heat_off_count++;
    }
    app_observe_diag.last_time_us = input->timestamp_us;
    app_observe_diag.last_dt = input->imu.dt;
    app_observe_diag.temperature = input->imu.temperature;
    app_observe_diag.heat_target = output->target;
    app_observe_diag.heat_compare = output->compare_f;
    app_observe_diag.yaw = state->ins.euler.yaw;
    app_observe_diag.pitch = state->ins.euler.pitch;
    app_observe_diag.roll = state->ins.euler.roll;
    app_observe_diag.yaw_total = state->ins.yaw_total;
    app_observe_diag.status = obs_status;
    app_observe_diag.thermal_status = (uint32_t)thermal_status;
    app_observe_diag.imu_online = input->imu.online;
    app_observe_diag.ins_ready = state->ins.ready;
    app_observe_diag.heat_enable = output->enable;
    app_observe_diag.chassis_valid_mask = state->chassis.valid_mask;
    diag = app_observe_diag;
    app_observe_exit(primask);

    app_observe_probe(state, &diag);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Observe 任务入口。
 *
 * @param argument CMSIS-RTOS2 输入参数。
 */
static void app_observe_entry(void *argument)
{
    ObserverInput input;
    ObserverState state;
    ThermalOutput thermal_output;
    imu_sample_t sample;
    float32_t dt;
    uint32_t timestamp_us;
    imu_status_t imu_status;
    ObserverStatus obs_status;
    ThermalStatus thermal_status;
    uint8_t stale;

    (void)argument;

    while (1) {
        dt = app_observe_dt(&stale);
        timestamp_us = (uint32_t)bsp_timebase_now_us();

        (void)memset(&sample, 0, sizeof(sample));
        imu_status = imu_read();
        if (imu_status == IMU_OK) {
            imu_status = imu_sample(&sample);
        }

        app_observe_fill_input(&sample, imu_status, dt, timestamp_us, &input);
        (void)memset(&state, 0, sizeof(state));
        obs_status = observer_step(&app_observe_ctx, &input, &state);
        if (obs_status != ObserverOk) {
            (void)observer_get(&app_observe_ctx, &state);
        }

        thermal_status = app_observe_heat_step(&sample,
                                               imu_status,
                                               dt,
                                               stale,
                                               &thermal_output);
        app_observe_commit(&input,
                           &state,
                           &thermal_output,
                           obs_status,
                           thermal_status,
                           imu_status,
                           stale);

        osDelay(APP_OBSERVE_MS_TO_TICKS(APP_OBSERVE_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_observe_init(void)
{
    ObserverCfg observer_cfg;
    ThermalCfg thermal_cfg;
    ObserverStatus observer_status;
    ThermalStatus thermal_status;
    uint32_t heat_max;
    float32_t target;

    (void)memset(&app_observe_state, 0, sizeof(app_observe_state));
    (void)memset(&app_observe_diag, 0, sizeof(app_observe_diag));
    (void)memset(&app_observe_chassis, 0, sizeof(app_observe_chassis));

    app_observe_heat_ready = (imu_heat_init() == HAL_OK) ? 1U : 0U;
    heat_max = imu_heat_max();
    if ((heat_max == 0U) || (heat_max > APP_OBSERVE_HEAT_MAX)) {
        heat_max = APP_OBSERVE_HEAT_MAX;
    }

    target = app_observe_temp_target(imu_cal_temp());
    app_observe_make_cfg(&observer_cfg);
    app_observe_make_thermal_cfg(heat_max, target, &thermal_cfg);

    observer_status = observer_init(&app_observe_ctx, &observer_cfg);
    thermal_status = thermal_init(&app_observe_thermal, &thermal_cfg);
    app_observe_diag.status = observer_status;
    app_observe_diag.thermal_status = (uint32_t)thermal_status;
    app_observe_diag.heat_target = target;
    if ((observer_status != ObserverOk) || (thermal_status != ThermalOk)) {
        (void)imu_heat_off();
        return 0U;
    }

    app_observe_thread = osThreadNew(app_observe_entry, NULL, &app_observe_attr);
    return (app_observe_thread != NULL) ? 1U : 0U;
}

uint8_t app_observe_get_state(ObserverState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_observe_enter();
    *out = app_observe_state;
    app_observe_exit(primask);

    return 1U;
}

uint8_t app_observe_get_diag(AppObserveDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_observe_enter();
    *out = app_observe_diag;
    app_observe_exit(primask);

    return 1U;
}

uint8_t app_observe_set_chassis_measure(const Vec3 *vel,
                                        uint8_t vel_mask,
                                        const Vec3 *acc,
                                        uint8_t acc_mask,
                                        uint32_t timestamp_us)
{
    AppObserveChassisMeasure measure;
    uint32_t primask;

    if (((vel_mask != 0U) && (vel == NULL)) ||
        ((acc_mask != 0U) && (acc == NULL))) {
        return 0U;
    }

    (void)memset(&measure, 0, sizeof(measure));
    if (vel != NULL) {
        measure.vel = *vel;
    }
    if (acc != NULL) {
        measure.acc = *acc;
    }
    measure.vel_mask = (uint8_t)(vel_mask & APP_OBSERVE_AXIS_MASK);
    measure.acc_mask = (uint8_t)(acc_mask & APP_OBSERVE_AXIS_MASK);
    measure.timestamp_us = timestamp_us;

    primask = app_observe_enter();
    app_observe_chassis = measure;
    app_observe_exit(primask);

    return 1U;
}
