/**
 ******************************************************************************
 * @file    app_gimbal.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Gimbal 应用层实现。
 *
 * @note    本文件只做 Input/Observe/电机反馈到 Gimbal controller 的数据接线。
 * @note    自瞄参考已经在 Input 链路中转换为 GimbalCmd，本文件不读取通讯组件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_gimbal.h"

#include "app_gimbal_config.h"
#include "app_input.h"
#include "app_observe.h"
#include "bsp_timebase.h"
#include "debug.h"
#include "main.h"
#include "cmsis_os.h"

#if GIMBAL_TYPE == GIMBAL_TYPE_DIRECT_YAW_PITCH
#include "gimbal_direct_yaw_pitch.h"
#elif GIMBAL_TYPE == GIMBAL_TYPE_DOUBLE_YAW_PITCH
#include "gimbal_double_yaw_pitch.h"
#elif GIMBAL_TYPE == GIMBAL_TYPE_LINKAGE_YAW_PITCH
#include "gimbal_linkage_yaw_pitch.h"
#else
#error "Unsupported GIMBAL_TYPE"
#endif

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_GIMBAL_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 云台应用 glue 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Gimbal 任务控制块。 */
static osThreadId_t app_gimbal_thread;

/** @brief Gimbal 任务属性。 */
static const osThreadAttr_t app_gimbal_attr = {
    .name = "app_gimbal",
    .stack_size = APP_GIMBAL_STACK_SIZE,
    .priority = (osPriority_t)APP_GIMBAL_PRIORITY,
};

/**
 * @brief 云台校准采样缓存。
 */
typedef struct
{
    float32_t max_yaw;      /**< yaw 正限位角度，单位 rad。 */
    float32_t min_yaw;      /**< yaw 负限位角度，单位 rad。 */
    float32_t max_pitch;    /**< pitch 正限位角度，单位 rad。 */
    float32_t min_pitch;    /**< pitch 负限位角度，单位 rad。 */
    uint16_t max_yaw_ecd;   /**< yaw 正限位编码器。 */
    uint16_t min_yaw_ecd;   /**< yaw 负限位编码器。 */
    uint16_t max_pitch_ecd; /**< pitch 正限位编码器。 */
    uint16_t min_pitch_ecd; /**< pitch 负限位编码器。 */
} AppGimbalCaliSample;

/** @brief Gimbal 公共上下文。 */
static Gimbal app_gimbal_ctx;

/** @brief Gimbal 公共上下文快照。 */
static Gimbal app_gimbal_snapshot;

/** @brief 最近一次 Gimbal 输入快照。 */
static GimbalInput app_gimbal_input;

/** @brief 最近一次 Gimbal 输出快照。 */
static GimbalOutput app_gimbal_output;

/** @brief Gimbal 应用层诊断快照。 */
static AppGimbalDiag app_gimbal_diag;

/** @brief 当前 Gimbal 应用层阶段。 */
static AppGimbalPhase app_gimbal_phase;

/** @brief 当前阶段开始时间，单位 ms。 */
static uint32_t app_gimbal_phase_start_ms;

/** @brief 初始化阶段稳定计时，单位 ms。 */
static uint32_t app_gimbal_init_stable_ms;

/** @brief 当前校准步骤。 */
static uint8_t app_gimbal_cali_step;

/** @brief 当前校准步骤低速累计时间，单位 ms。 */
static uint32_t app_gimbal_cali_stable_ms;

/** @brief 校准采样缓存。 */
static AppGimbalCaliSample app_gimbal_cali_sample;

/** @brief 最近一次有效云台校准结果。 */
static GimbalCali app_gimbal_last_cali;

/** @brief 非 0 表示 app_gimbal_last_cali 有效。 */
static uint8_t app_gimbal_last_cali_valid;

#if GIMBAL_TYPE == GIMBAL_TYPE_DIRECT_YAW_PITCH
/** @brief yaw/pitch 直驱云台私有上下文。 */
static GimbalDirectCtx app_gimbal_impl;
#elif GIMBAL_TYPE == GIMBAL_TYPE_DOUBLE_YAW_PITCH
/** @brief double-yaw-pitch 云台私有上下文。 */
static GimbalDoubleCtx app_gimbal_impl;
#elif GIMBAL_TYPE == GIMBAL_TYPE_LINKAGE_YAW_PITCH
/** @brief linkage-yaw-pitch 云台私有上下文。 */
static GimbalLinkageCtx app_gimbal_impl;
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
static uint32_t app_gimbal_enter(void)
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
static void app_gimbal_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 设置应用层阶段并重置阶段计时。
 *
 * @param phase 新阶段。
 * @param now_ms 当前时间，单位 ms。
 */
static void app_gimbal_set_phase(AppGimbalPhase phase, uint32_t now_ms)
{
    app_gimbal_phase = phase;
    app_gimbal_phase_start_ms = now_ms;
    app_gimbal_init_stable_ms = 0U;
    if (phase != AppGimbalPhaseCali) {
        app_gimbal_cali_step = 0U;
        app_gimbal_cali_stable_ms = 0U;
    }
}

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Gimbal App 负责 init/cali/ready 阶段和电机发送，自瞄来源已经在 Input 链路收敛。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/**
 * @brief 判断当前云台实现是否处于应用层 ready 阶段。
 *
 * @return ready 返回 1，否则返回 0。
 */
static uint8_t app_gimbal_phase_ready(void)
{
    return (app_gimbal_phase == AppGimbalPhaseReady) ? 1U : 0U;
}

/**
 * @brief 格式化编码器值到 0~ecd_range-1。
 *
 * @param value 原始编码器值。
 * @return 格式化后的编码器值。
 */
static uint16_t app_gimbal_ecd_format(int32_t value)
{
    int32_t range = app_gimbal_ctx.cfg.ecd_range;

    if (range <= 0) {
        return 0U;
    }

    while (value < 0) {
        value += range;
    }
    while (value >= range) {
        value -= range;
    }

    return (uint16_t)value;
}

/**
 * @brief 编码器相对中值换算为角度。
 *
 * @param ecd 当前编码器。
 * @param offset 中值编码器。
 * @return 相对角，单位 rad。
 */
static float32_t app_gimbal_ecd_angle(uint16_t ecd, uint16_t offset)
{
    int32_t delta;

    delta = (int32_t)ecd - (int32_t)offset;
    if (delta > app_gimbal_ctx.cfg.half_ecd_range) {
        delta -= app_gimbal_ctx.cfg.ecd_range;
    } else if (delta < -app_gimbal_ctx.cfg.half_ecd_range) {
        delta += app_gimbal_ctx.cfg.ecd_range;
    }

    return (float32_t)delta * app_gimbal_ctx.cfg.ecd_to_rad;
}

#if GIMBAL_TYPE == GIMBAL_TYPE_DIRECT_YAW_PITCH
/**
 * @brief 读取直驱 yaw 相对角。
 *
 * @return yaw 相对角，单位 rad。
 */
static float32_t app_gimbal_yaw_relative(void)
{
    return app_gimbal_impl.yaw.relative;
}

/**
 * @brief 读取直驱 pitch 绝对角。
 *
 * @return pitch 绝对角，单位 rad。
 */
static float32_t app_gimbal_pitch_absolute(void)
{
    return app_gimbal_impl.pitch.absolute;
}

/**
 * @brief 读取直驱单轴陀螺仪角速度。
 *
 * @param yaw 非 0 表示读取 yaw，否则读取 pitch。
 * @return 轴角速度，单位 rad/s。
 */
static float32_t app_gimbal_axis_gyro(uint8_t yaw)
{
    return (yaw != 0U) ? app_gimbal_impl.yaw.gyro : app_gimbal_impl.pitch.gyro;
}

/**
 * @brief 读取直驱单轴编码器。
 *
 * @param yaw 非 0 表示读取 yaw，否则读取 pitch。
 * @return 最近反馈编码器。
 */
static uint16_t app_gimbal_axis_ecd(uint8_t yaw)
{
    return (yaw != 0U) ? app_gimbal_impl.yaw.fb.ecd : app_gimbal_impl.pitch.fb.ecd;
}
#else
/**
 * @brief 非直驱构型暂不执行 App 级初始化角度判定。
 *
 * @return 默认相对角。
 */
static float32_t app_gimbal_yaw_relative(void)
{
    return 0.0F;
}

/**
 * @brief 非直驱构型暂不执行 App 级初始化角度判定。
 *
 * @return 默认绝对角。
 */
static float32_t app_gimbal_pitch_absolute(void)
{
    return 0.0F;
}

/**
 * @brief 非直驱构型默认认为已停止。
 *
 * @param yaw 非 0 表示 yaw，否则表示 pitch。
 * @return 默认角速度。
 */
static float32_t app_gimbal_axis_gyro(uint8_t yaw)
{
    (void)yaw;
    return 0.0F;
}

/**
 * @brief 非直驱构型默认编码器。
 *
 * @param yaw 非 0 表示 yaw，否则表示 pitch。
 * @return 默认编码器。
 */
static uint16_t app_gimbal_axis_ecd(uint8_t yaw)
{
    (void)yaw;
    return 0U;
}
#endif

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
static void app_gimbal_pid(GimbalPidCfg *cfg,
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
 * @brief 生成 Gimbal controller 配置。
 *
 * @param cfg 输出配置。
 */
static void app_gimbal_make_cfg(GimbalCfg *cfg)
{
    if (cfg == NULL) {
        return;
    }

    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->type = (GimbalType)GIMBAL_TYPE;
    app_gimbal_pid(&cfg->yaw_abs_pid,
                   ROBOT_GIMBAL_YAW_ABS_PID_KP,
                   ROBOT_GIMBAL_YAW_ABS_PID_KI,
                   ROBOT_GIMBAL_YAW_ABS_PID_KD,
                   ROBOT_GIMBAL_YAW_ABS_PID_MAX_OUT,
                   ROBOT_GIMBAL_YAW_ABS_PID_MAX_IOUT);
    app_gimbal_pid(&cfg->yaw_rel_pid,
                   ROBOT_GIMBAL_YAW_REL_PID_KP,
                   ROBOT_GIMBAL_YAW_REL_PID_KI,
                   ROBOT_GIMBAL_YAW_REL_PID_KD,
                   ROBOT_GIMBAL_YAW_REL_PID_MAX_OUT,
                   ROBOT_GIMBAL_YAW_REL_PID_MAX_IOUT);
    app_gimbal_pid(&cfg->yaw_speed_pid,
                   ROBOT_GIMBAL_YAW_SPEED_PID_KP,
                   ROBOT_GIMBAL_YAW_SPEED_PID_KI,
                   ROBOT_GIMBAL_YAW_SPEED_PID_KD,
                   ROBOT_GIMBAL_YAW_SPEED_PID_MAX_OUT,
                   ROBOT_GIMBAL_YAW_SPEED_PID_MAX_IOUT);
    app_gimbal_pid(&cfg->pitch_abs_pid,
                   ROBOT_GIMBAL_PITCH_ABS_PID_KP,
                   ROBOT_GIMBAL_PITCH_ABS_PID_KI,
                   ROBOT_GIMBAL_PITCH_ABS_PID_KD,
                   ROBOT_GIMBAL_PITCH_ABS_PID_MAX_OUT,
                   ROBOT_GIMBAL_PITCH_ABS_PID_MAX_IOUT);
    app_gimbal_pid(&cfg->pitch_rel_pid,
                   ROBOT_GIMBAL_PITCH_REL_PID_KP,
                   ROBOT_GIMBAL_PITCH_REL_PID_KI,
                   ROBOT_GIMBAL_PITCH_REL_PID_KD,
                   ROBOT_GIMBAL_PITCH_REL_PID_MAX_OUT,
                   ROBOT_GIMBAL_PITCH_REL_PID_MAX_IOUT);
    app_gimbal_pid(&cfg->pitch_speed_pid,
                   ROBOT_GIMBAL_PITCH_SPEED_PID_KP,
                   ROBOT_GIMBAL_PITCH_SPEED_PID_KI,
                   ROBOT_GIMBAL_PITCH_SPEED_PID_KD,
                   ROBOT_GIMBAL_PITCH_SPEED_PID_MAX_OUT,
                   ROBOT_GIMBAL_PITCH_SPEED_PID_MAX_IOUT);
    cfg->ecd_to_rad = ROBOT_GIMBAL_MOTOR_ECD_TO_RAD;
    cfg->ecd_range = ROBOT_GIMBAL_ECD_RANGE;
    cfg->half_ecd_range = ROBOT_GIMBAL_ECD_HALF_RANGE;
    cfg->yaw_turn = ROBOT_GIMBAL_YAW_TURN;
    cfg->pitch_turn = ROBOT_GIMBAL_PITCH_TURN;
    cfg->yaw_current_polarity = (int8_t)ROBOT_GIMBAL_YAW_CURRENT_POLARITY;
    cfg->pitch_current_polarity = (int8_t)ROBOT_GIMBAL_PITCH_CURRENT_POLARITY;
    cfg->period_ms = APP_GIMBAL_PERIOD_MS;
}

/**
 * @brief 由采样点计算 GimbalCali。
 *
 * @param sample 校准采样点。
 * @param cali 校准结果输出。
 */
static void app_gimbal_calc_cali(const AppGimbalCaliSample *sample, GimbalCali *cali)
{
    int32_t temp_ecd;
    int32_t temp_max_ecd;
    int32_t temp_min_ecd;

    if ((sample == NULL) || (cali == NULL)) {
        return;
    }

    (void)memset(cali, 0, sizeof(*cali));

    if (app_gimbal_ctx.cfg.yaw_turn != 0U) {
        temp_ecd = (int32_t)sample->min_yaw_ecd - (int32_t)sample->max_yaw_ecd;
        if (temp_ecd < 0) {
            temp_ecd += app_gimbal_ctx.cfg.ecd_range;
        }
        temp_ecd = (int32_t)sample->max_yaw_ecd + (temp_ecd / 2);
        cali->yaw_offset = app_gimbal_ecd_format(temp_ecd);
        cali->yaw_max = -app_gimbal_ecd_angle(sample->max_yaw_ecd, cali->yaw_offset);
        cali->yaw_min = -app_gimbal_ecd_angle(sample->min_yaw_ecd, cali->yaw_offset);
    } else {
        temp_ecd = (int32_t)sample->max_yaw_ecd - (int32_t)sample->min_yaw_ecd;
        if (temp_ecd < 0) {
            temp_ecd += app_gimbal_ctx.cfg.ecd_range;
        }
        temp_ecd = (int32_t)sample->max_yaw_ecd - (temp_ecd / 2);
        cali->yaw_offset = app_gimbal_ecd_format(temp_ecd);
        cali->yaw_max = app_gimbal_ecd_angle(sample->max_yaw_ecd, cali->yaw_offset);
        cali->yaw_min = app_gimbal_ecd_angle(sample->min_yaw_ecd, cali->yaw_offset);
    }

    if (app_gimbal_ctx.cfg.pitch_turn != 0U) {
        temp_ecd = (int32_t)(sample->max_pitch / app_gimbal_ctx.cfg.ecd_to_rad);
        temp_max_ecd = (int32_t)sample->max_pitch_ecd + temp_ecd;
        temp_ecd = (int32_t)(sample->min_pitch / app_gimbal_ctx.cfg.ecd_to_rad);
        temp_min_ecd = (int32_t)sample->min_pitch_ecd + temp_ecd;
    } else {
        temp_ecd = (int32_t)(sample->max_pitch / app_gimbal_ctx.cfg.ecd_to_rad);
        temp_max_ecd = (int32_t)sample->max_pitch_ecd - temp_ecd;
        temp_ecd = (int32_t)(sample->min_pitch / app_gimbal_ctx.cfg.ecd_to_rad);
        temp_min_ecd = (int32_t)sample->min_pitch_ecd - temp_ecd;
    }

    temp_max_ecd = (int32_t)app_gimbal_ecd_format(temp_max_ecd);
    temp_min_ecd = (int32_t)app_gimbal_ecd_format(temp_min_ecd);
    temp_ecd = temp_max_ecd - temp_min_ecd;
    if (temp_ecd > app_gimbal_ctx.cfg.half_ecd_range) {
        temp_ecd -= app_gimbal_ctx.cfg.ecd_range;
    } else if (temp_ecd < -app_gimbal_ctx.cfg.half_ecd_range) {
        temp_ecd += app_gimbal_ctx.cfg.ecd_range;
    }

    if ((app_gimbal_ctx.cfg.pitch_turn != 0U) && (temp_max_ecd > temp_min_ecd)) {
        temp_min_ecd += app_gimbal_ctx.cfg.ecd_range;
    }

    temp_ecd = temp_max_ecd - (temp_ecd / 2);
    cali->pitch_offset = app_gimbal_ecd_format(temp_ecd);
    if (app_gimbal_ctx.cfg.pitch_turn != 0U) {
        cali->pitch_max = -app_gimbal_ecd_angle(sample->max_pitch_ecd, cali->pitch_offset);
        cali->pitch_min = -app_gimbal_ecd_angle(sample->min_pitch_ecd, cali->pitch_offset);
    } else {
        cali->pitch_max = app_gimbal_ecd_angle(sample->max_pitch_ecd, cali->pitch_offset);
        cali->pitch_min = app_gimbal_ecd_angle(sample->min_pitch_ecd, cali->pitch_offset);
    }

    cali->yaw_polarity = app_gimbal_ctx.cfg.yaw_current_polarity;
    cali->pitch_polarity = app_gimbal_ctx.cfg.pitch_current_polarity;
}

/**
 * @brief 判断初始化是否已稳定到中值。
 *
 * @param now_ms 当前时间，单位 ms。
 * @return 完成返回 1，否则返回 0。
 */
static uint8_t app_gimbal_init_done(uint32_t now_ms)
{
    float32_t yaw_err = ROBOT_GIMBAL_INIT_YAW_SET - app_gimbal_yaw_relative();
    float32_t pitch_err = ROBOT_GIMBAL_INIT_PITCH_SET - app_gimbal_pitch_absolute();

    if ((fabsf(yaw_err) < ROBOT_GIMBAL_INIT_ANGLE_ERROR) &&
        (fabsf(pitch_err) < ROBOT_GIMBAL_INIT_ANGLE_ERROR)) {
        app_gimbal_init_stable_ms += APP_GIMBAL_PERIOD_MS;
    } else {
        app_gimbal_init_stable_ms = 0U;
    }

    return ((app_gimbal_init_stable_ms >= ROBOT_GIMBAL_INIT_STOP_TIME) ||
            ((uint32_t)(now_ms - app_gimbal_phase_start_ms) >= ROBOT_GIMBAL_INIT_TIME)) ? 1U : 0U;
}

/**
 * @brief 生成初始化阶段命令。
 *
 * @param input GimbalInput。
 * @param now_ms 当前时间，单位 ms。
 */
static void app_gimbal_apply_init(GimbalInput *input, uint32_t now_ms)
{
    float32_t yaw_err;
    float32_t pitch_err;

    if (input == NULL) {
        return;
    }

    if (app_gimbal_phase_start_ms == 0U) {
        app_gimbal_phase_start_ms = now_ms;
    }

    if (app_gimbal_init_done(now_ms) != 0U) {
        app_gimbal_set_phase(AppGimbalPhaseReady, now_ms);
        app_gimbal_diag.init_done_count++;
        return;
    }

    yaw_err = ROBOT_GIMBAL_INIT_YAW_SET - app_gimbal_yaw_relative();
    pitch_err = ROBOT_GIMBAL_INIT_PITCH_SET - app_gimbal_pitch_absolute();
    input->cmd.online = 1U;
    input->cmd.mode = GimbalModeGyro;
    input->cmd.pitch = pitch_err * ROBOT_GIMBAL_INIT_PITCH_SPEED;
    input->cmd.yaw = (fabsf(pitch_err) < ROBOT_GIMBAL_INIT_ANGLE_ERROR) ?
                     (yaw_err * ROBOT_GIMBAL_INIT_YAW_SPEED) : 0.0F;
}

/**
 * @brief 判断当前校准步骤是否已经低速持续足够时间。
 *
 * @param gyro 当前轴角速度。
 * @return 达成返回 1，否则返回 0。
 */
static uint8_t app_gimbal_cali_low_speed(float32_t gyro)
{
    if (fabsf(gyro) < ROBOT_GIMBAL_CALI_GYRO_LIMIT) {
        app_gimbal_cali_stable_ms += APP_GIMBAL_PERIOD_MS;
    }

    return (app_gimbal_cali_stable_ms >= ROBOT_GIMBAL_CALI_STEP_TIME) ? 1U : 0U;
}

/**
 * @brief 推进校准步骤。
 */
static void app_gimbal_cali_next(void)
{
    app_gimbal_cali_stable_ms = 0U;
    app_gimbal_cali_step++;
}

/**
 * @brief 完成校准并注入结果。
 *
 * @param now_ms 当前时间，单位 ms。
 */
static void app_gimbal_finish_cali(uint32_t now_ms)
{
    GimbalCali cali;
    GimbalStatus status;

    app_gimbal_calc_cali(&app_gimbal_cali_sample, &cali);
    status = gimbal_set_cali(&app_gimbal_ctx, &cali);
    app_gimbal_diag.gimbal_status = status;
    if (status == GimbalOk) {
        app_gimbal_last_cali = cali;
        app_gimbal_last_cali_valid = 1U;
        app_gimbal_diag.cali_done_count++;
        app_gimbal_set_phase(AppGimbalPhaseReady, now_ms);
    } else {
        app_gimbal_diag.cali_fail_count++;
        app_gimbal_set_phase(AppGimbalPhaseError, now_ms);
    }
}

/**
 * @brief 生成校准阶段命令。
 *
 * @param input GimbalInput。
 * @param now_ms 当前时间，单位 ms。
 */
static void app_gimbal_apply_cali(GimbalInput *input, uint32_t now_ms)
{
    if (input == NULL) {
        return;
    }

    input->cmd.online = 1U;
    input->cmd.mode = GimbalModeRaw;
    input->cmd.yaw = 0.0F;
    input->cmd.pitch = 0.0F;

    if (app_gimbal_cali_step == ROBOT_GIMBAL_CALI_PITCH_MAX_STEP) {
        input->cmd.pitch = (float32_t)ROBOT_GIMBAL_CALI_MOTOR_SET;
        if (app_gimbal_cali_low_speed(app_gimbal_axis_gyro(0U)) != 0U) {
            app_gimbal_cali_sample.max_pitch = app_gimbal_pitch_absolute();
            app_gimbal_cali_sample.max_pitch_ecd = app_gimbal_axis_ecd(0U);
            app_gimbal_cali_next();
        }
    } else if (app_gimbal_cali_step == ROBOT_GIMBAL_CALI_PITCH_MIN_STEP) {
        input->cmd.pitch = -(float32_t)ROBOT_GIMBAL_CALI_MOTOR_SET;
        if (app_gimbal_cali_low_speed(app_gimbal_axis_gyro(0U)) != 0U) {
            app_gimbal_cali_sample.min_pitch = app_gimbal_pitch_absolute();
            app_gimbal_cali_sample.min_pitch_ecd = app_gimbal_axis_ecd(0U);
            app_gimbal_cali_next();
        }
    } else if (app_gimbal_cali_step == ROBOT_GIMBAL_CALI_YAW_MAX_STEP) {
        input->cmd.yaw = (float32_t)ROBOT_GIMBAL_CALI_MOTOR_SET;
        if (app_gimbal_cali_low_speed(app_gimbal_axis_gyro(1U)) != 0U) {
            app_gimbal_cali_sample.max_yaw = app_gimbal_yaw_relative();
            app_gimbal_cali_sample.max_yaw_ecd = app_gimbal_axis_ecd(1U);
            app_gimbal_cali_next();
        }
    } else if (app_gimbal_cali_step == ROBOT_GIMBAL_CALI_YAW_MIN_STEP) {
        input->cmd.yaw = -(float32_t)ROBOT_GIMBAL_CALI_MOTOR_SET;
        if (app_gimbal_cali_low_speed(app_gimbal_axis_gyro(1U)) != 0U) {
            app_gimbal_cali_sample.min_yaw = app_gimbal_yaw_relative();
            app_gimbal_cali_sample.min_yaw_ecd = app_gimbal_axis_ecd(1U);
            app_gimbal_cali_next();
        }
    } else {
        app_gimbal_finish_cali(now_ms);
    }
}

/**
 * @brief 根据应用层阶段覆盖输入命令。
 *
 * @param input GimbalInput。
 * @param now_ms 当前时间，单位 ms。
 */
static void app_gimbal_apply_phase(GimbalInput *input, uint32_t now_ms)
{
    if (input == NULL) {
        return;
    }

    if (app_gimbal_phase == AppGimbalPhaseInit) {
        app_gimbal_apply_init(input, now_ms);
    } else if (app_gimbal_phase == AppGimbalPhaseCali) {
        app_gimbal_apply_cali(input, now_ms);
    } else if (app_gimbal_phase == AppGimbalPhaseError) {
        (void)memset(&input->cmd, 0, sizeof(input->cmd));
        input->cmd.mode = GimbalModeZero;
    }
}

/**
 * @brief 判断电机反馈是否在超时时间内更新。
 *
 * @param fb DJI 电机反馈。
 * @param now_ms 当前时间，单位 ms。
 * @return 在线返回 1，否则返回 0。
 */
static uint8_t app_gimbal_motor_recent(const DjiMotorFb *fb, uint32_t now_ms)
{
    if (fb == NULL) {
        return 0U;
    }

#if APP_GIMBAL_MOTOR_LOST_MS == 0U
    (void)now_ms;
    return 1U;
#else
    return (((uint32_t)(now_ms - fb->last_time)) <= APP_GIMBAL_MOTOR_LOST_MS) ? 1U : 0U;
#endif
}

/**
 * @brief 读取 DJI 电机反馈并转换为 GimbalMotorFb。
 *
 * @param bus FDCAN 总线。
 * @param rx_id 电机反馈 ID。
 * @param now_ms 当前时间，单位 ms。
 * @param out 云台电机反馈输出。
 * @return Devices/Motor 读取状态。
 */
static motor_status_t app_gimbal_motor_fb(bsp_fdcan_id_t bus,
                                          uint16_t rx_id,
                                          uint32_t now_ms,
                                          GimbalMotorFb *out)
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
    if (app_gimbal_motor_recent(&fb, now_ms) == 0U) {
        return MotorNoData;
    }

    out->ecd = fb.ecd;
    out->speed_rpm = fb.speed_rpm;
    out->current = fb.given_current;
    out->online = 1U;
    return MotorOk;
}

/**
 * @brief 填充上层输入命令。
 *
 * @param input GimbalInput 输出。
 */
static void app_gimbal_fill_cmd(GimbalInput *input)
{
    if (input == NULL) {
        return;
    }

    if (app_input_get_gimbal_cmd(&input->cmd) == 0U) {
        (void)memset(&input->cmd, 0, sizeof(input->cmd));
    }
}

/**
 * @brief 填充 Observe 姿态和角速度。
 *
 * @param input GimbalInput 输出。
 */
static void app_gimbal_fill_observe(GimbalInput *input)
{
    ObserverState state;

    if (input == NULL) {
        return;
    }

    if ((app_observe_get_state(&state) != 0U) &&
        ((state.valid_mask & OBSERVER_VALID_INS) != 0U) &&
        (state.ins.ready != 0U)) {
        input->ins_yaw = state.ins.yaw_total;
        input->ins_pitch = state.ins.euler.pitch;
        input->gyro_x = state.ins.gyro.x;
        input->gyro_y = state.ins.gyro.y;
        input->gyro_z = state.ins.gyro.z;
        input->ins_ready = 1U;
    }
}

/**
 * @brief 填充完整 GimbalInput。
 *
 * @param now_us 当前时间，单位 us。
 * @param input GimbalInput 输出。
 * @param yaw_status yaw 电机反馈读取状态。
 * @param pitch_status pitch 电机反馈读取状态。
 */
static void app_gimbal_fill_input(uint64_t now_us,
                                  GimbalInput *input,
                                  motor_status_t *yaw_status,
                                  motor_status_t *pitch_status)
{
    uint32_t now_ms = (uint32_t)(now_us / 1000ULL);

    if ((input == NULL) || (yaw_status == NULL) || (pitch_status == NULL)) {
        return;
    }

    (void)memset(input, 0, sizeof(*input));
    input->time_us = (uint32_t)now_us;
    input->tick_ms = now_ms;

    app_gimbal_fill_cmd(input);
    app_gimbal_fill_observe(input);
    *yaw_status = app_gimbal_motor_fb(MOTOR_GIMBAL_BUS,
                                      MOTOR_GIMBAL_YAW_ID,
                                      now_ms,
                                      &input->yaw_fb);
    *pitch_status = app_gimbal_motor_fb(MOTOR_GIMBAL_BUS,
                                        MOTOR_GIMBAL_PITCH_ID,
                                        now_ms,
                                        &input->pitch_fb);
}

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/**
 * @brief 发送云台输出，失败或禁发时发送零电流。
 *
 * @param output Gimbal 输出。
 * @param status Gimbal controller 状态。
 * @param zero_out 非 0 输出是否被清零。
 * @return 电机发送状态。
 */
static motor_status_t app_gimbal_send(const GimbalOutput *output,
                                      GimbalStatus status,
                                      uint8_t *zero_out)
{
    if (zero_out != NULL) {
        *zero_out = 1U;
    }

    if ((output != NULL) &&
        (status == GimbalOk) &&
        (output->send_enable != 0U)) {
        if (zero_out != NULL) {
            *zero_out = 0U;
        }
        return motor_gimbal_yaw_pitch(output->yaw_current,
                                      output->pitch_current,
                                      0);
    }

    return motor_gimbal_yaw_pitch(0, 0, 0);
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
 * @param gimbal_status controller 状态。
 * @param yaw_status yaw 反馈读取状态。
 * @param pitch_status pitch 反馈读取状态。
 * @param send_status 电机发送状态。
 * @param zero_out 非 0 表示本周期发送零电流。
 */
static void app_gimbal_commit(const GimbalInput *input,
                              const GimbalOutput *output,
                              GimbalStatus gimbal_status,
                              motor_status_t yaw_status,
                              motor_status_t pitch_status,
                              motor_status_t send_status,
                              uint8_t zero_out)
{
    uint32_t primask;
    AppGimbalDiag diag;

    if ((input == NULL) || (output == NULL)) {
        return;
    }

    primask = app_gimbal_enter();
    app_gimbal_input = *input;
    app_gimbal_output = *output;
    app_gimbal_snapshot = app_gimbal_ctx;
    app_gimbal_diag.loop_count++;
    if (input->cmd.online == 0U) {
        app_gimbal_diag.input_lost_count++;
    }
    if (input->ins_ready == 0U) {
        app_gimbal_diag.observe_lost_count++;
    }
    if (input->yaw_fb.online == 0U) {
        app_gimbal_diag.yaw_fb_lost_count++;
    }
    if (input->pitch_fb.online == 0U) {
        app_gimbal_diag.pitch_fb_lost_count++;
    }
    if (gimbal_status != GimbalOk) {
        app_gimbal_diag.gimbal_fail_count++;
    }
    if (send_status == MotorOk) {
        app_gimbal_diag.send_count++;
    } else {
        app_gimbal_diag.send_fail_count++;
    }
    if (zero_out != 0U) {
        app_gimbal_diag.zero_count++;
    }
    app_gimbal_diag.last_tick_ms = input->tick_ms;
    app_gimbal_diag.last_time_us = input->time_us;
    app_gimbal_diag.gimbal_status = gimbal_status;
    app_gimbal_diag.yaw_status = yaw_status;
    app_gimbal_diag.pitch_status = pitch_status;
    app_gimbal_diag.send_status = send_status;
    app_gimbal_diag.mode = output->mode;
    app_gimbal_diag.phase = app_gimbal_phase;
    app_gimbal_diag.yaw_current = output->yaw_current;
    app_gimbal_diag.pitch_current = output->pitch_current;
    app_gimbal_diag.aux_current = output->aux_current;
    app_gimbal_diag.cali_step = app_gimbal_cali_step;
    app_gimbal_diag.input_online = input->cmd.online;
    app_gimbal_diag.observe_ready = input->ins_ready;
    app_gimbal_diag.yaw_online = input->yaw_fb.online;
    app_gimbal_diag.pitch_online = input->pitch_fb.online;
    app_gimbal_diag.send_enable = output->send_enable;
    diag = app_gimbal_diag;
    app_gimbal_exit(primask);

    DBG_BOOL("app.gimbal.in", diag.input_online != 0U);
    DBG_BOOL("app.gimbal.obs", diag.observe_ready != 0U);
    DBG_U32("app.gimbal.status", (uint32_t)diag.gimbal_status);
    DBG_F32("app.gimbal.yaw", (float32_t)diag.yaw_current);
    DBG_F32("app.gimbal.pitch", (float32_t)diag.pitch_current);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Gimbal 任务入口。
 *
 * @param argument CMSIS-RTOS2 输入参数。
 */
static void app_gimbal_entry(void *argument)
{
    GimbalInput input;
    GimbalOutput output;
    GimbalStatus gimbal_status;
    motor_status_t yaw_status;
    motor_status_t pitch_status;
    motor_status_t send_status;
    uint64_t now_us;
    uint8_t zero_out;

    (void)argument;

    while (1) {
        now_us = bsp_timebase_now_us();
        app_gimbal_fill_input(now_us, &input, &yaw_status, &pitch_status);
        app_gimbal_apply_phase(&input, (uint32_t)(now_us / 1000ULL));

        (void)memset(&output, 0, sizeof(output));
        gimbal_status = gimbal_step(&app_gimbal_ctx, &input, &output);
        if ((gimbal_status != GimbalOk) || (output.send_enable == 0U)) {
            (void)memset(&output, 0, sizeof(output));
        }

        send_status = app_gimbal_send(&output, gimbal_status, &zero_out);
        app_gimbal_commit(&input,
                          &output,
                          gimbal_status,
                          yaw_status,
                          pitch_status,
                          send_status,
                          zero_out);

        osDelay(APP_GIMBAL_MS_TO_TICKS(APP_GIMBAL_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_gimbal_init(void)
{
    GimbalCfg cfg;
    GimbalStatus status;

    (void)memset(&app_gimbal_ctx, 0, sizeof(app_gimbal_ctx));
    (void)memset(&app_gimbal_snapshot, 0, sizeof(app_gimbal_snapshot));
    (void)memset(&app_gimbal_input, 0, sizeof(app_gimbal_input));
    (void)memset(&app_gimbal_output, 0, sizeof(app_gimbal_output));
    (void)memset(&app_gimbal_diag, 0, sizeof(app_gimbal_diag));
    (void)memset(&app_gimbal_impl, 0, sizeof(app_gimbal_impl));
    (void)memset(&app_gimbal_cali_sample, 0, sizeof(app_gimbal_cali_sample));
    (void)memset(&app_gimbal_last_cali, 0, sizeof(app_gimbal_last_cali));
    app_gimbal_last_cali_valid = 0U;
    app_gimbal_phase = AppGimbalPhaseInit;
    app_gimbal_phase_start_ms = 0U;
    app_gimbal_init_stable_ms = 0U;
    app_gimbal_cali_step = 0U;
    app_gimbal_cali_stable_ms = 0U;

    app_gimbal_make_cfg(&cfg);
    status = gimbal_init(&app_gimbal_ctx, &cfg, &app_gimbal_impl);
    app_gimbal_diag.gimbal_status = status;
    if (status != GimbalOk) {
        return 0U;
    }

    app_gimbal_snapshot = app_gimbal_ctx;
    app_gimbal_thread = osThreadNew(app_gimbal_entry, NULL, &app_gimbal_attr);
    return (app_gimbal_thread != NULL) ? 1U : 0U;
}

uint8_t app_gimbal_get_state(Gimbal *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_gimbal_enter();
    *out = app_gimbal_snapshot;
    app_gimbal_exit(primask);
    return 1U;
}

uint8_t app_gimbal_get_input(GimbalInput *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_gimbal_enter();
    *out = app_gimbal_input;
    app_gimbal_exit(primask);
    return 1U;
}

uint8_t app_gimbal_get_output(GimbalOutput *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_gimbal_enter();
    *out = app_gimbal_output;
    app_gimbal_exit(primask);
    return 1U;
}

uint8_t app_gimbal_get_diag(AppGimbalDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_gimbal_enter();
    *out = app_gimbal_diag;
    app_gimbal_exit(primask);
    return 1U;
}

uint8_t app_gimbal_ready(void)
{
    uint8_t ready;
    uint32_t primask;

    primask = app_gimbal_enter();
    ready = app_gimbal_phase_ready();
    app_gimbal_exit(primask);
    return ready;
}

uint8_t app_gimbal_get_phase(AppGimbalPhase *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_gimbal_enter();
    *out = app_gimbal_phase;
    app_gimbal_exit(primask);
    return 1U;
}

uint8_t app_gimbal_start_cali(void)
{
    uint32_t primask;
    uint32_t now_ms;

    now_ms = (uint32_t)(bsp_timebase_now_us() / 1000ULL);
    primask = app_gimbal_enter();
    if (app_gimbal_phase == AppGimbalPhaseCali) {
        app_gimbal_exit(primask);
        return 0U;
    }
    (void)memset(&app_gimbal_cali_sample, 0, sizeof(app_gimbal_cali_sample));
    app_gimbal_cali_step = (uint8_t)ROBOT_GIMBAL_CALI_START_STEP;
    app_gimbal_cali_stable_ms = 0U;
    app_gimbal_set_phase(AppGimbalPhaseCali, now_ms);
    app_gimbal_cali_step = (uint8_t)ROBOT_GIMBAL_CALI_START_STEP;
    app_gimbal_exit(primask);
    return 1U;
}

uint8_t app_gimbal_set_cali(const GimbalCali *cali)
{
    uint32_t primask;
    GimbalStatus status;

    if (cali == NULL) {
        return 0U;
    }

    primask = app_gimbal_enter();
    status = gimbal_set_cali(&app_gimbal_ctx, cali);
    app_gimbal_snapshot = app_gimbal_ctx;
    app_gimbal_diag.gimbal_status = status;
    if (status == GimbalOk) {
        app_gimbal_last_cali = *cali;
        app_gimbal_last_cali_valid = 1U;
        app_gimbal_set_phase(AppGimbalPhaseReady, (uint32_t)(bsp_timebase_now_us() / 1000ULL));
    }
    app_gimbal_exit(primask);

    return (status == GimbalOk) ? 1U : 0U;
}

uint8_t app_gimbal_get_cali(GimbalCali *out)
{
    uint32_t primask;
    uint8_t valid;

    if (out == NULL) {
        return 0U;
    }

    primask = app_gimbal_enter();
    valid = app_gimbal_last_cali_valid;
    if (valid != 0U) {
        *out = app_gimbal_last_cali;
    }
    app_gimbal_exit(primask);
    return valid;
}
