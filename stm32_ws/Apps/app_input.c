/**
 ******************************************************************************
 * @file    app_input.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Input 应用层实现。
 *
 * @note    本文件只做 RC 帧同步、视觉输入读取、Observe yaw 快照注入和
 *          Input 输出缓存，不实现机构控制闭环。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_input.h"

#include "app_communication.h"
#include "app_input_config.h"
#include "app_observe.h"
#include "aim.h"
#include "bsp_timebase.h"
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
#define APP_INPUT_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 输入聚合 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Input 任务控制块。 */
static osThreadId_t app_input_thread;

/** @brief Input 任务属性。 */
static const osThreadAttr_t app_input_attr = {
    .name = "app_input",
    .stack_size = APP_INPUT_STACK_SIZE,
    .priority = (osPriority_t)APP_INPUT_PRIORITY,
};

/** @brief Input 组件上下文。 */
static Input app_input_ctx;

/** @brief 自瞄误差整形上下文。 */
static Aim app_input_aim;

/** @brief 最近一次 RC DBUS 状态缓存。 */
static RcDbusState app_input_rc;

/** @brief 最近一次 Input 输出快照。 */
static InputOutput app_input_output;

/** @brief Input 诊断快照。 */
static AppInputDiag app_input_diag;

/** @brief 最近一次处理的 RC 原始帧序号。 */
static uint32_t app_input_last_seq;

/** @brief 最近一次 Aim 执行时间戳，单位 us。 */
static uint64_t app_input_last_aim_us;

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
static uint32_t app_input_enter(void)
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
static void app_input_exit(uint32_t primask)
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
 * @brief 生成 Input 默认配置。
 *
 * @param cfg 输出配置。
 */
static void app_input_make_cfg(InputCfg *cfg)
{
    if (cfg == NULL) {
        return;
    }

    (void)memset(cfg, 0, sizeof(*cfg));

    cfg->gimbal_rc.yaw_ch = (int8_t)ROBOT_INPUT_GIMBAL_YAW_CHANNEL;
    cfg->gimbal_rc.pitch_ch = (int8_t)ROBOT_INPUT_GIMBAL_PITCH_CHANNEL;
    cfg->gimbal_rc.mode_sw = (int8_t)ROBOT_INPUT_GIMBAL_MODE_CHANNEL;
    cfg->gimbal_rc.override_sw = (int8_t)ROBOT_INPUT_GIMBAL_MODE_CHANNEL;
    cfg->gimbal_rc.deadband = (int16_t)ROBOT_INPUT_GIMBAL_RC_DEADBAND;
    cfg->gimbal_rc.yaw_scale = ROBOT_INPUT_GIMBAL_YAW_RC_SEN;
    cfg->gimbal_rc.pitch_scale = ROBOT_INPUT_GIMBAL_PITCH_RC_SEN;
    cfg->gimbal_rc.mouse_yaw_scale = -ROBOT_INPUT_GIMBAL_YAW_MOUSE_SEN;
    cfg->gimbal_rc.mouse_pitch_scale = ROBOT_INPUT_GIMBAL_PITCH_MOUSE_SEN;
    cfg->gimbal_rc.sw_up_mode = GimbalModeGyro;
    cfg->gimbal_rc.sw_mid_mode = GimbalModeZero;
    cfg->gimbal_rc.sw_down_mode = GimbalModeGyro;
    cfg->gimbal_rc.zero_key = ROBOT_INPUT_GIMBAL_ZERO_KEY;
    cfg->gimbal_rc.gyro_key = ROBOT_INPUT_GIMBAL_SPIN_KEY;
    cfg->gimbal_rc.encode_key = ROBOT_INPUT_GIMBAL_RELATIVE_KEY;
    cfg->gimbal_rc.raw_key = 0U;
    cfg->gimbal_rc.turn_key = ROBOT_INPUT_GIMBAL_TURN_KEY;
    cfg->gimbal_rc.turn_speed = ROBOT_GIMBAL_TURN_SPEED;
    cfg->gimbal_rc.turn_done_err = APP_INPUT_TURN_DONE_ERR;
    cfg->gimbal_rc.priority = APP_INPUT_RC_PRIORITY;

    cfg->chassis_rc.vx_ch = (int8_t)ROBOT_INPUT_CHASSIS_X_CHANNEL;
    cfg->chassis_rc.vy_ch = (int8_t)ROBOT_INPUT_CHASSIS_Y_CHANNEL;
    cfg->chassis_rc.wz_ch = (int8_t)ROBOT_INPUT_CHASSIS_WZ_CHANNEL;
    cfg->chassis_rc.mode_sw = (int8_t)ROBOT_INPUT_CHASSIS_MODE_CHANNEL;
    cfg->chassis_rc.follow_sw = (int8_t)ROBOT_INPUT_CHASSIS_FOLLOW_CHANNEL;
    cfg->chassis_rc.override_sw = (int8_t)ROBOT_INPUT_CHASSIS_MODE_CHANNEL;
    cfg->chassis_rc.deadband = (int16_t)ROBOT_INPUT_CHASSIS_RC_DEADBAND;
    cfg->chassis_rc.vx_scale = ROBOT_INPUT_CHASSIS_VX_RC_SEN;
    cfg->chassis_rc.vy_scale = -ROBOT_INPUT_CHASSIS_VY_RC_SEN;
    cfg->chassis_rc.wz_scale = -ROBOT_INPUT_CHASSIS_WZ_RC_SEN;
    cfg->chassis_rc.key_vx_max = ROBOT_INPUT_CHASSIS_KEY_VX_MAX;
    cfg->chassis_rc.key_vx_min = ROBOT_INPUT_CHASSIS_KEY_VX_MIN;
    cfg->chassis_rc.key_vy_max = ROBOT_INPUT_CHASSIS_KEY_VY_MAX;
    cfg->chassis_rc.key_vy_min = ROBOT_INPUT_CHASSIS_KEY_VY_MIN;
    cfg->chassis_rc.key_wz = 0.0F;
    cfg->chassis_rc.vx_lpf_num = ROBOT_CHASSIS_ACCEL_X_NUM;
    cfg->chassis_rc.vy_lpf_num = ROBOT_CHASSIS_ACCEL_Y_NUM;
    cfg->chassis_rc.period_s = APP_INPUT_PERIOD_S;
    cfg->chassis_rc.spin_wz = ROBOT_SPIN_SPEED * ROBOT_SPIN_FACTOR;
    cfg->chassis_rc.swing_move_angle = ROBOT_CHASSIS_SWING_MOVE_ANGLE;
    cfg->chassis_rc.swing_stop_angle = ROBOT_CHASSIS_SWING_NO_MOVE_ANGLE;
    cfg->chassis_rc.swing_step = ROBOT_INPUT_CHASSIS_SWING_STEP;
    cfg->chassis_rc.front_key = ROBOT_INPUT_CHASSIS_FRONT_KEY;
    cfg->chassis_rc.back_key = ROBOT_INPUT_CHASSIS_BACK_KEY;
    cfg->chassis_rc.left_key = ROBOT_INPUT_CHASSIS_LEFT_KEY;
    cfg->chassis_rc.right_key = ROBOT_INPUT_CHASSIS_RIGHT_KEY;
    cfg->chassis_rc.turn_l_key = 0U;
    cfg->chassis_rc.turn_r_key = 0U;
    cfg->chassis_rc.spin_key = ROBOT_INPUT_CHASSIS_SPIN_KEY;
    cfg->chassis_rc.stop_key = ROBOT_INPUT_CHASSIS_STOP_KEY;
    cfg->chassis_rc.follow_key = ROBOT_INPUT_CHASSIS_FOLLOW_KEY;
    cfg->chassis_rc.swing_key = ROBOT_INPUT_CHASSIS_SWING_KEY;
    cfg->chassis_rc.jump_key = 0U;
    cfg->chassis_rc.sw_up_mode = ChassisModeSpin;
    cfg->chassis_rc.sw_mid_mode = ChassisModeStop;
    cfg->chassis_rc.sw_down_mode = ChassisModeNoFollow;
    cfg->chassis_rc.follow_down_mode = ChassisModeFollowGimbalYaw;
    cfg->chassis_rc.follow_else_mode = ChassisModeNoFollow;
    cfg->chassis_rc.priority = APP_INPUT_RC_PRIORITY;

    cfg->shoot_rc.mode_sw = (int8_t)ROBOT_INPUT_SHOOT_MODE_CHANNEL;
    cfg->shoot_rc.toggle_sw = RC_SW_UP;
    cfg->shoot_rc.fire_sw = RC_SW_DOWN;
    cfg->shoot_rc.fric_on_key = ROBOT_INPUT_SHOOT_ON_KEY;
    cfg->shoot_rc.fric_off_key = ROBOT_INPUT_SHOOT_OFF_KEY;
    cfg->shoot_rc.fire_once_key = 0U;
    cfg->shoot_rc.fire_hold_key = 0U;
    cfg->shoot_rc.mag_open_key = ROBOT_INPUT_MAGAZINE_OPEN_KEY;
    cfg->shoot_rc.mag_close_key = ROBOT_INPUT_MAGAZINE_CLOSE_KEY;
    cfg->shoot_rc.press_long_time = ROBOT_INPUT_SHOOT_PRESS_LONG_TIME;
    cfg->shoot_rc.switch_long_time = ROBOT_INPUT_SHOOT_SWITCH_LONG_TIME;
    cfg->shoot_rc.mag_mode_sw = (int8_t)ROBOT_INPUT_MAGAZINE_MODE_CHANNEL;
    cfg->shoot_rc.mag_guard_sw = (int8_t)ROBOT_INPUT_MAGAZINE_GUARD_CHANNEL;
    cfg->shoot_rc.mag_guard_value = ROBOT_INPUT_MAGAZINE_GUARD_VALUE;
    cfg->shoot_rc.mag_open_sw = ROBOT_INPUT_MAGAZINE_OPEN_SW;
    cfg->shoot_rc.mag_close_sw = ROBOT_INPUT_MAGAZINE_CLOSE_SW;
    cfg->shoot_rc.once_mouse_l = 1U;
    cfg->shoot_rc.hold_mouse_l = 1U;
    cfg->shoot_rc.hold_mouse_r = 0U;
    cfg->shoot_rc.priority = APP_INPUT_RC_PRIORITY;

    cfg->cali_rc.begin_count = ROBOT_INPUT_CALI_BEGIN_COUNT;
    cfg->cali_rc.action_count = ROBOT_INPUT_CALI_ACTION_COUNT;
    cfg->cali_rc.gimbal_key = ROBOT_INPUT_CALI_GIMBAL_KEY;
    cfg->cali_rc.imu_key = ROBOT_INPUT_CALI_IMU_KEY;
    cfg->cali_rc.enable_old_gesture = ROBOT_INPUT_CALI_ENABLE_OLD_GESTURE;
    cfg->cali_rc.priority = APP_INPUT_RC_PRIORITY;
}

/**
 * @brief 生成 Aim 默认配置。
 *
 * @param cfg 输出配置。
 */
static void app_input_make_aim_cfg(AimCfg *cfg)
{
    if (cfg == NULL) {
        return;
    }

    (void)aim_default_cfg(cfg);
    cfg->yaw.kv = ROBOT_AA_KV_YAW;
    cfg->yaw.kd = ROBOT_AA_KD_YAW;
    cfg->yaw.omega_max = ROBOT_AA_OMEGA_MAX_YAW;
    cfg->yaw.accel_max = ROBOT_AA_ACCEL_MAX_YAW;
    cfg->pitch.kv = ROBOT_AA_KV_PITCH;
    cfg->pitch.kd = ROBOT_AA_KD_PITCH;
    cfg->pitch.omega_max = ROBOT_AA_OMEGA_MAX_PITCH;
    cfg->pitch.accel_max = ROBOT_AA_ACCEL_MAX_PITCH;
    cfg->err_soft_zone = ROBOT_AA_ERR_SOFT_ZONE_RAD;
    cfg->d_lpf_alpha = ROBOT_AA_D_LPF_ALPHA;
    cfg->eps_angle = ROBOT_AA_EPS_TH_RAD;
    cfg->eps_omega = ROBOT_AA_EPS_OMEGA_RAD;
    cfg->dt_default = APP_INPUT_PERIOD_S;
    cfg->mode = GimbalModeGyro;
    cfg->median_enable = ROBOT_AA_MEDIAN_ENABLE;
    cfg->zero_hold_count = ROBOT_AA_ZERO_HOLD_COUNT;
}

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Input App 把 RC、视觉和 Aim 整形结果送入 Component/Input，输出 Controller 命令快照。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/**
 * @brief 同步 RC 原始帧到 DBUS 状态缓存。
 *
 * @param now_ms 当前时间，单位 ms。
 * @return 最近一次 DBUS 解析状态。
 */
static RcStatus app_input_sync_rc(uint32_t now_ms)
{
    RcFrame frame;
    RcStatus status = RcOk;
    uint8_t stale;

    if (rc_frame(&frame) != 0U) {
        if (frame.seq != app_input_last_seq) {
            status = rc_dbus_parse(frame.data, frame.size, frame.tick_ms, &app_input_rc);
            app_input_last_seq = frame.seq;
            app_input_diag.last_frame_seq = frame.seq;
            if (status == RcOk) {
                app_input_diag.rc_sync_count++;
            } else {
                rc_dbus_state_clear(&app_input_rc);
                app_input_diag.rc_parse_fail++;
            }
        }
    } else {
        status = RcFrameErr;
    }

    stale = (uint8_t)((app_input_rc.online == 0U) ||
                      ((now_ms - app_input_rc.last_update_tick) > APP_INPUT_RC_LOST_MS));
    if (stale != 0U) {
        if (app_input_rc.online != 0U) {
            app_input_diag.rc_lost_count++;
        }
        app_input_rc.online = 0U;
    }

    app_input_diag.rc_status = status;
    app_input_diag.rc_online = app_input_rc.online;
    return status;
}

/**
 * @brief 读取 host 自瞄误差并整形为视觉输入。
 *
 * @param now_us 当前时间戳，单位 us。
 * @param vision 视觉输入输出。
 * @return 非 0 表示本周期有视觉指针参与输入。
 */
static uint8_t app_input_fill_vision(uint64_t now_us, InputVisionState *vision)
{
#if APP_INPUT_USE_COMM_AIM
    CommunicationStatus status;
    CommunicationAimState raw;
    AimInput aim_in;
    AimOutput aim_out;
    AimStatus aim_status;
    float32_t dt;

    if (vision == NULL) {
        return 0U;
    }

    (void)memset(vision, 0, sizeof(*vision));
    (void)memset(&raw, 0, sizeof(raw));
    status = app_communication_get_aim(&raw);
    app_input_diag.vision_status = status;
    if (status != CommunicationOk) {
        app_input_diag.vision_online = 0U;
        return 0U;
    }

    if (app_input_last_aim_us == 0ULL) {
        dt = APP_INPUT_PERIOD_S;
    } else {
        dt = (float32_t)(now_us - app_input_last_aim_us) / 1000000.0F;
    }
    app_input_last_aim_us = now_us;

    (void)memset(&aim_in, 0, sizeof(aim_in));
    aim_in.yaw_err = raw.yaw_err_rad;
    aim_in.pitch_err = raw.pitch_err_rad;
    aim_in.dt = dt;
    aim_in.timestamp_us = raw.host_time_us;
    aim_in.enable = raw.enable;
    if ((raw.valid_mask & COMM_HOST_AIM_YAW_VALID) != 0U) {
        aim_in.valid_mask |= AIM_YAW_VALID;
    }
    if ((raw.valid_mask & COMM_HOST_AIM_PITCH_VALID) != 0U) {
        aim_in.valid_mask |= AIM_PITCH_VALID;
    }

    (void)memset(&aim_out, 0, sizeof(aim_out));
    aim_status = aim_step(&app_input_aim, &aim_in, &aim_out);
    if (aim_status != AimOk) {
        app_input_diag.vision_online = 0U;
        return 0U;
    }

    vision->yaw = aim_out.yaw;
    vision->pitch = aim_out.pitch;
    vision->mode = aim_out.mode;
    vision->priority = raw.priority;
    vision->tick_ms = raw.tick_ms;
    vision->online = aim_out.online;
    if (aim_out.online != 0U) {
        vision->mode_valid = 1U;
        vision->valid_mask |= INPUT_GIMBAL_MODE_VALID;
    }
    if ((aim_out.valid_mask & AIM_YAW_VALID) != 0U) {
        vision->valid_mask |= INPUT_GIMBAL_YAW_VALID;
    }
    if ((aim_out.valid_mask & AIM_PITCH_VALID) != 0U) {
        vision->valid_mask |= INPUT_GIMBAL_PITCH_VALID;
    }

    app_input_diag.vision_online = ((vision->online != 0U) && (vision->valid_mask != 0U)) ? 1U : 0U;
    if (app_input_diag.vision_online != 0U) {
        app_input_diag.vision_count++;
    }
    return 1U;
#else
    (void)now_us;
    (void)vision;
    app_input_diag.vision_status = CommunicationState;
    app_input_diag.vision_online = 0U;
    return 0U;
#endif
}

/**
 * @brief 填充当前云台 yaw 快照。
 *
 * @param frame Input 周期帧。
 */
static void app_input_fill_yaw(InputFrame *frame)
{
    ObserverState state;

    if (frame == NULL) {
        return;
    }

    if ((app_observe_get_state(&state) != 0U) &&
        ((state.valid_mask & OBSERVER_VALID_INS) != 0U) &&
        (state.ins.ready != 0U)) {
        frame->gimbal_yaw = state.ins.yaw_total;
        frame->gimbal_yaw_valid = 1U;
    }
}

/* ==========================================================================
 * 诊断快照
 * --------------------------------------------------------------------------
 * 周期末统一提交快照和诊断，其他模块只能读取稳定副本。
 * ========================================================================== */
/**
 * @brief 提交 Input 输出和诊断快照。
 *
 * @param output Input 输出。
 * @param status Input 状态。
 * @param now_ms 当前时间，单位 ms。
 */
static void app_input_commit(const InputOutput *output, InputStatus status, uint32_t now_ms)
{
    uint32_t primask;

    if (output == NULL) {
        return;
    }

    primask = app_input_enter();
    app_input_output = *output;
    app_input_diag.loop_count++;
    if (status != InputOk) {
        app_input_diag.input_fail_count++;
    }
    app_input_diag.input_status = status;
    app_input_diag.last_tick_ms = now_ms;
    app_input_diag.gimbal_online = output->gimbal.online;
    app_input_diag.chassis_online = output->chassis.online;
    app_input_diag.shoot_online = output->shoot.online;
    app_input_diag.mag_online = output->mag.online;
    app_input_diag.mag_valid = output->mag.valid;
    app_input_diag.mag_action = output->mag.action;
    app_input_diag.cali_online = output->cali.online;
    app_input_diag.cali_valid = output->cali.valid;
    app_input_diag.cali_action = output->cali.action;
    app_input_exit(primask);

    DBG_BOOL("app.input.rc", app_input_diag.rc_online != 0U);
    DBG_BOOL("app.input.vision", app_input_diag.vision_online != 0U);
    DBG_U32("app.input.status", (uint32_t)status);
    DBG_F32("app.input.yaw", output->gimbal.yaw);
    DBG_F32("app.input.vx", output->chassis.vx);
    DBG_F32("app.input.vy", output->chassis.vy);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Input 任务入口。
 *
 * @param argument CMSIS-RTOS2 输入参数。
 */
static void app_input_entry(void *argument)
{
    InputFrame frame;
    InputOutput output;
    InputVisionState vision;
    InputStatus status;
    uint64_t now_us;
    uint32_t now_ms;

    (void)argument;

    while (1) {
        now_us = bsp_timebase_now_us();
        now_ms = (uint32_t)(now_us / 1000ULL);
        (void)app_input_sync_rc(now_ms);

        (void)memset(&frame, 0, sizeof(frame));
        frame.rc = &app_input_rc;
        if (app_input_fill_vision(now_us, &vision) != 0U) {
            frame.vision = &vision;
        }
        frame.tick_ms = now_ms;
        app_input_fill_yaw(&frame);

        (void)memset(&output, 0, sizeof(output));
        status = input_step(&app_input_ctx, &frame, &output);
        if (status != InputOk) {
            (void)input_get(&app_input_ctx, &output);
        }
        app_input_commit(&output, status, now_ms);

        osDelay(APP_INPUT_MS_TO_TICKS(APP_INPUT_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_input_init(void)
{
    InputCfg cfg;
    AimCfg aim_cfg;

    (void)memset(&app_input_ctx, 0, sizeof(app_input_ctx));
    (void)memset(&app_input_aim, 0, sizeof(app_input_aim));
    (void)memset(&app_input_rc, 0, sizeof(app_input_rc));
    (void)memset(&app_input_output, 0, sizeof(app_input_output));
    (void)memset(&app_input_diag, 0, sizeof(app_input_diag));
    app_input_last_seq = 0U;
    app_input_last_aim_us = 0ULL;

    app_input_make_cfg(&cfg);
    if (input_init(&app_input_ctx, &cfg) != InputOk) {
        return 0U;
    }

    app_input_make_aim_cfg(&aim_cfg);
    if (aim_init(&app_input_aim, &aim_cfg) != AimOk) {
        return 0U;
    }

    app_input_thread = osThreadNew(app_input_entry, NULL, &app_input_attr);
    return (app_input_thread != NULL) ? 1U : 0U;
}

uint8_t app_input_get_output(InputOutput *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_output;
    app_input_exit(primask);
    return 1U;
}

uint8_t app_input_get_ref(InputRef *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_output.ref;
    app_input_exit(primask);
    return 1U;
}

uint8_t app_input_get_gimbal_cmd(GimbalCmd *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_output.gimbal;
    app_input_exit(primask);
    return 1U;
}

uint8_t app_input_get_chassis_cmd(ChassisCmd *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_output.chassis;
    app_input_exit(primask);
    return 1U;
}

uint8_t app_input_get_shoot_cmd(ShootCmd *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_output.shoot;
    app_input_exit(primask);
    return 1U;
}

uint8_t app_input_get_magazine_cmd(InputMagCmd *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_output.mag;
    app_input_exit(primask);
    return 1U;
}

uint8_t app_input_get_cali_cmd(InputCaliCmd *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_output.cali;
    app_input_exit(primask);
    return 1U;
}

uint8_t app_input_get_diag(AppInputDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_input_enter();
    *out = app_input_diag;
    app_input_exit(primask);
    return 1U;
}
