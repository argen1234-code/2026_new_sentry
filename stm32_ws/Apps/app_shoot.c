/**
 ******************************************************************************
 * @file    app_shoot.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Shoot 应用层实现。
 *
 * @note    本文件只做 Input/热量/电机反馈到 Shoot controller 的数据接线。
 * @note    本文件不读取遥控器、裁判系统、云台任务或旧发射任务全局状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_shoot.h"

#include "app_gimbal.h"
#include "app_input.h"
#include "app_shoot_config.h"
#include "bsp_timebase.h"
#include "debug.h"
#include "main.h"
#include "cmsis_os.h"

#if SHOOT_TYPE == SHOOT_TYPE_NONE
#include "shoot_none.h"
#elif SHOOT_TYPE == SHOOT_TYPE_MONO
#include "shoot_mono.h"
#elif SHOOT_TYPE == SHOOT_TYPE_DOUBLE
#include "shoot_double.h"
#else
#error "Unsupported SHOOT_TYPE"
#endif

#include <math.h>
#include <string.h>

#if SHOOT_TYPE == SHOOT_TYPE_NONE
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 当前构型需要的摩擦轮数量。 */
#define APP_SHOOT_FRIC_NEED 0U
#elif SHOOT_TYPE == SHOOT_TYPE_MONO
/** @brief 当前构型需要的摩擦轮数量。 */
#define APP_SHOOT_FRIC_NEED 2U
#elif SHOOT_TYPE == SHOOT_TYPE_DOUBLE
/** @brief 当前构型需要的摩擦轮数量。 */
#define APP_SHOOT_FRIC_NEED 4U
#endif

/** @brief 当前构型所需摩擦轮全部在线的 bit mask。 */
#define APP_SHOOT_FRIC_ALL_MASK ((uint8_t)((1U << APP_SHOOT_FRIC_NEED) - 1U))

/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_SHOOT_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/**
 * @brief 发射热量测量缓存。
 */
typedef struct
{
    float32_t heat;   /**< 当前热量。 */
    float32_t limit;  /**< 当前热量上限。 */
    uint8_t valid;    /**< 非 0 表示缓存有效。 */
} AppShootHeat;

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 发射应用 glue 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Shoot 任务控制块。 */
static osThreadId_t app_shoot_thread;

/** @brief Shoot 任务属性。 */
static const osThreadAttr_t app_shoot_attr = {
    .name = "app_shoot",
    .stack_size = APP_SHOOT_STACK_SIZE,
    .priority = (osPriority_t)APP_SHOOT_PRIORITY,
};

/** @brief Shoot 公共上下文。 */
static Shoot app_shoot_ctx;

/** @brief Shoot 公共上下文快照。 */
static Shoot app_shoot_snapshot;

/** @brief 最近一次 Shoot 输入快照。 */
static ShootInput app_shoot_input;

/** @brief 最近一次 Shoot 输出快照。 */
static ShootOutput app_shoot_output;

/** @brief Shoot 应用层诊断快照。 */
static AppShootDiag app_shoot_diag;

/** @brief 最近一次热量注入缓存。 */
static AppShootHeat app_shoot_heat;

#if SHOOT_TYPE == SHOOT_TYPE_NONE
/** @brief 无发射机构私有上下文。 */
static ShootNoneCtx app_shoot_impl;
#elif SHOOT_TYPE == SHOOT_TYPE_MONO
/** @brief 单组摩擦轮发射私有上下文。 */
static ShootMonoCtx app_shoot_impl;
#elif SHOOT_TYPE == SHOOT_TYPE_DOUBLE
/** @brief 双组摩擦轮发射私有上下文。 */
static ShootDoubleCtx app_shoot_impl;
#endif

/** @brief 摩擦轮电机反馈 ID 表。 */
static const uint16_t app_shoot_fric_id[SHOOT_FRIC_MAX] = {
    MOTOR_FRIC_0_ID,
    MOTOR_FRIC_1_ID,
    MOTOR_FRIC_2_ID,
    MOTOR_FRIC_3_ID,
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
static uint32_t app_shoot_enter(void)
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
static void app_shoot_exit(uint32_t primask)
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
static void app_shoot_pid(ShootPidCfg *cfg,
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
 * @brief 生成 Shoot controller 配置。
 *
 * @param cfg 输出配置。
 */
static void app_shoot_make_cfg(ShootCfg *cfg)
{
    if (cfg == NULL) {
        return;
    }

    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->type = (ShootType)SHOOT_TYPE;
    app_shoot_pid(&cfg->trigger_pid,
                  ROBOT_SHOOT_TRIGGER_PID_KP,
                  ROBOT_SHOOT_TRIGGER_PID_KI,
                  ROBOT_SHOOT_TRIGGER_PID_KD,
                  ROBOT_SHOOT_TRIGGER_BULLET_MAX_OUT,
                  ROBOT_SHOOT_TRIGGER_BULLET_MAX_IOUT);
    app_shoot_pid(&cfg->fric_pid,
                  ROBOT_SHOOT_FRIC_PID_KP,
                  ROBOT_SHOOT_FRIC_PID_KI,
                  ROBOT_SHOOT_FRIC_PID_KD,
                  ROBOT_SHOOT_FRIC_MAX_OUT,
                  ROBOT_SHOOT_FRIC_MAX_IOUT);
    cfg->trigger_ecd_to_rad = ROBOT_SHOOT_ECD_TO_ANGLE;
    cfg->trigger_ecd_range = ROBOT_SHOOT_ECD_RANGE;
    cfg->trigger_half_ecd_range = ROBOT_SHOOT_ECD_HALF_RANGE;
    cfg->trigger_rpm_to_rad = ROBOT_SHOOT_RPM_TO_SPEED;
    cfg->fric_rpm_to_speed = ROBOT_SHOOT_FRIC_RPM_TO_SPEED;
    cfg->trigger_step = ROBOT_SHOOT_STEP_ANGLE;
    cfg->trigger_done_err = ROBOT_SHOOT_TRIGGER_DONE_ERR;
    cfg->trigger_speed = ROBOT_SHOOT_TRIGGER_SPEED;
    cfg->block_speed = ROBOT_SHOOT_BLOCK_TRIGGER_SPEED;
    cfg->block_time = ROBOT_SHOOT_BLOCK_TIME_MS;
    cfg->reverse_time = ROBOT_SHOOT_REVERSE_TIME_MS;
    cfg->heat_per_bullet = (float32_t)ROBOT_SHOOT_HEAT_PER_BULLET;
    cfg->heat_cool = (float32_t)ROBOT_SHOOT_COOL;
    cfg->fric_ready_ratio = ROBOT_SHOOT_FRIC_READY_RATIO;
    cfg->fric_speed[0] = ROBOT_SHOOT_FRIC_UP_SPEED;
    cfg->fric_speed[1] = -ROBOT_SHOOT_FRIC_UP_SPEED;
    cfg->fric_speed[2] = ROBOT_SHOOT_FRIC_UP_SPEED;
    cfg->fric_speed[3] = -ROBOT_SHOOT_FRIC_UP_SPEED;
    cfg->fric_polarity[0] = (int8_t)ROBOT_SHOOT_FRIC_POLARITY_0;
    cfg->fric_polarity[1] = (int8_t)ROBOT_SHOOT_FRIC_POLARITY_1;
    cfg->fric_polarity[2] = (int8_t)ROBOT_SHOOT_FRIC_POLARITY_2;
    cfg->fric_polarity[3] = (int8_t)ROBOT_SHOOT_FRIC_POLARITY_3;
    cfg->trigger_polarity = (int8_t)ROBOT_SHOOT_TRIGGER_POLARITY;
}

/**
 * @brief 判断电机反馈是否在超时时间内更新。
 *
 * @param fb DJI 电机反馈。
 * @param now_ms 当前时间，单位 ms。
 * @return 在线返回 1，否则返回 0。
 */
static uint8_t app_shoot_motor_recent(const DjiMotorFb *fb, uint32_t now_ms)
{
    if (fb == NULL) {
        return 0U;
    }

#if APP_SHOOT_MOTOR_LOST_MS == 0U
    (void)now_ms;
    return 1U;
#else
    return (((uint32_t)(now_ms - fb->last_time)) <= APP_SHOOT_MOTOR_LOST_MS) ? 1U : 0U;
#endif
}

/**
 * @brief 读取 DJI 电机反馈并转换为 ShootMotorFb。
 *
 * @param bus FDCAN 总线。
 * @param rx_id 电机反馈 ID。
 * @param now_ms 当前时间，单位 ms。
 * @param out 发射电机反馈输出。
 * @return Devices/Motor 读取状态。
 */
static motor_status_t app_shoot_motor_fb(bsp_fdcan_id_t bus,
                                         uint16_t rx_id,
                                         uint32_t now_ms,
                                         ShootMotorFb *out)
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
    if (app_shoot_motor_recent(&fb, now_ms) == 0U) {
        return MotorNoData;
    }

    out->ecd = fb.ecd;
    out->speed_rpm = fb.speed_rpm;
    out->current = fb.given_current;
    out->temperature = fb.temperature;
    out->online = 1U;
    return MotorOk;
}

/* ==========================================================================
 * 输入采集与状态更新
 * --------------------------------------------------------------------------
 * Shoot App 只采集输入、热量和电机反馈，拨弹/摩擦轮控制留在 Shoot Controller。 这里把外部快照转换为本 App 周期内使用的输入结构。
 * ========================================================================== */
/**
 * @brief 填充上层输入命令。
 *
 * @param input ShootInput 输出。
 */
static void app_shoot_fill_cmd(ShootInput *input)
{
    if (input == NULL) {
        return;
    }

    if (app_input_get_shoot_cmd(&input->cmd) == 0U) {
        (void)memset(&input->cmd, 0, sizeof(input->cmd));
    }
    if (app_gimbal_ready() == 0U) {
        input->cmd.force_stop = 1U;
    }
}

/**
 * @brief 复制当前热量缓存。
 *
 * @param heat 输出热量缓存。
 */
static void app_shoot_get_heat(AppShootHeat *heat)
{
    uint32_t primask;

    if (heat == NULL) {
        return;
    }

    primask = app_shoot_enter();
    *heat = app_shoot_heat;
    app_shoot_exit(primask);
}

/**
 * @brief 填充热量预算输入。
 *
 * @param input ShootInput 输出。
 */
static void app_shoot_fill_heat(ShootInput *input)
{
    AppShootHeat heat;

    if (input == NULL) {
        return;
    }

    (void)memset(&heat, 0, sizeof(heat));
    app_shoot_get_heat(&heat);
    if (heat.valid != 0U) {
        input->heat = heat.heat;
        input->heat_limit = (heat.limit > 0.0F) ? heat.limit : (float32_t)ROBOT_SHOOT_LIMIT;
    } else {
        input->heat = 0.0F;
        input->heat_limit = (float32_t)ROBOT_SHOOT_LIMIT;
    }
}

/**
 * @brief 填充完整 ShootInput。
 *
 * @param now_us 当前时间，单位 us。
 * @param input ShootInput 输出。
 * @param trigger_status trigger 反馈读取状态输出。
 * @param fric_status 摩擦轮反馈读取状态输出。
 */
static void app_shoot_fill_input(uint64_t now_us,
                                 ShootInput *input,
                                 motor_status_t *trigger_status,
                                 motor_status_t fric_status[SHOOT_FRIC_MAX])
{
    uint32_t i;
    uint32_t now_ms = (uint32_t)(now_us / 1000ULL);

    if ((input == NULL) || (trigger_status == NULL) || (fric_status == NULL)) {
        return;
    }

    (void)memset(input, 0, sizeof(*input));
    input->tick_ms = now_ms;

    app_shoot_fill_cmd(input);
    app_shoot_fill_heat(input);

    *trigger_status = app_shoot_motor_fb(MOTOR_GIMBAL_BUS,
                                         MOTOR_TRIGGER_ID,
                                         now_ms,
                                         &input->trigger_fb);
    for (i = 0U; i < SHOOT_FRIC_MAX; i++) {
        fric_status[i] = app_shoot_motor_fb(MOTOR_FRIC_BUS,
                                            app_shoot_fric_id[i],
                                            now_ms,
                                            &input->fric_fb[i]);
    }
}

/**
 * @brief 生成摩擦轮反馈在线 bit mask。
 *
 * @param input 周期输入。
 * @return 在线 bit mask。
 */
static uint8_t app_shoot_fric_mask(const ShootInput *input)
{
    uint8_t mask = 0U;
    uint32_t i;

    if (input == NULL) {
        return 0U;
    }

    for (i = 0U; i < APP_SHOOT_FRIC_NEED; i++) {
        if (input->fric_fb[i].online != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

/**
 * @brief 判断当前构型所需反馈是否完整。
 *
 * @param input 周期输入。
 * @return 反馈完整返回 1，否则返回 0。
 */
static uint8_t app_shoot_feedback_ready(const ShootInput *input)
{
    if (input == NULL) {
        return 0U;
    }

#if SHOOT_TYPE == SHOOT_TYPE_NONE
    return 1U;
#else
    if (input->trigger_fb.online == 0U) {
        return 0U;
    }

    return (app_shoot_fric_mask(input) == APP_SHOOT_FRIC_ALL_MASK) ? 1U : 0U;
#endif
}

/**
 * @brief 判断摩擦轮温度是否处于安全范围。
 *
 * @param input 周期输入。
 * @return 温度安全返回 1，超过安全上限返回 0。
 */
static uint8_t app_shoot_temp_safe(const ShootInput *input)
{
    uint32_t i;

    if (input == NULL) {
        return 0U;
    }

#if SHOOT_TYPE == SHOOT_TYPE_NONE
    (void)i;
    return 1U;
#else
    for (i = 0U; i < APP_SHOOT_FRIC_NEED; i++) {
        if ((input->fric_fb[i].online != 0U) &&
            (input->fric_fb[i].temperature >= (uint8_t)APP_SHOOT_FRIC_TEMP_LIMIT)) {
            return 0U;
        }
    }

    return 1U;
#endif
}

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/**
 * @brief 发送发射输出，失败或禁止发送时发送零电流。
 *
 * @param output Shoot 输出。
 * @param status Shoot controller 状态。
 * @param fric_send 摩擦轮发送状态输出。
 * @param trigger_send trigger 发送状态输出。
 * @param zero_out 非 0 表示本周期发送零电流。
 */
static void app_shoot_send(const ShootOutput *output,
                           ShootStatus status,
                           motor_status_t *fric_send,
                           motor_status_t *trigger_send,
                           uint8_t *zero_out)
{
    int16_t trigger = 0;
    int16_t fric[SHOOT_FRIC_MAX] = {0, 0, 0, 0};

    if (zero_out != NULL) {
        *zero_out = 1U;
    }

    if ((output != NULL) &&
        (status == ShootOk) &&
        (output->send_enable != 0U)) {
        uint32_t i;
        trigger = output->trigger_current;
        for (i = 0U; i < SHOOT_FRIC_MAX; i++) {
            fric[i] = output->fric_current[i];
        }
        if (zero_out != NULL) {
            *zero_out = 0U;
        }
    }

    if (fric_send != NULL) {
        *fric_send = motor_fric(fric[0], fric[1], fric[2], fric[3]);
    } else {
        (void)motor_fric(fric[0], fric[1], fric[2], fric[3]);
    }

    if (trigger_send != NULL) {
        *trigger_send = motor_gimbal_trigger(trigger);
    } else {
        (void)motor_gimbal_trigger(trigger);
    }
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
 * @param shoot_status controller 状态。
 * @param trigger_status trigger 反馈读取状态。
 * @param fric_status 摩擦轮反馈读取状态。
 * @param fric_send 摩擦轮发送状态。
 * @param trigger_send trigger 发送状态。
 * @param zero_out 非 0 表示本周期发送零电流。
 * @param temp_safe 非 0 表示摩擦轮温度未触发安全钳位。
 * @param now_us 当前时间戳，单位 us。
 */
static void app_shoot_commit(const ShootInput *input,
                             const ShootOutput *output,
                             ShootStatus shoot_status,
                             motor_status_t trigger_status,
                             const motor_status_t fric_status[SHOOT_FRIC_MAX],
                             motor_status_t fric_send,
                             motor_status_t trigger_send,
                             uint8_t zero_out,
                             uint8_t temp_safe,
                             uint64_t now_us)
{
    uint32_t primask;
    uint32_t i;
    uint8_t fric_mask;
    AppShootDiag diag;

    if ((input == NULL) || (output == NULL) || (fric_status == NULL)) {
        return;
    }

    fric_mask = app_shoot_fric_mask(input);

    primask = app_shoot_enter();
    app_shoot_input = *input;
    app_shoot_output = *output;
    app_shoot_snapshot = app_shoot_ctx;
    app_shoot_diag.loop_count++;
    if (input->cmd.online == 0U) {
        app_shoot_diag.input_lost_count++;
    }
#if SHOOT_TYPE != SHOOT_TYPE_NONE
    if (input->trigger_fb.online == 0U) {
        app_shoot_diag.trigger_fb_lost_count++;
    }
    if (fric_mask != APP_SHOOT_FRIC_ALL_MASK) {
        app_shoot_diag.fric_fb_lost_count++;
    }
#endif
    if (shoot_status != ShootOk) {
        app_shoot_diag.shoot_fail_count++;
    }
    if (temp_safe == 0U) {
        app_shoot_diag.fric_temp_over_count++;
    }
    if ((fric_send == MotorOk) && (trigger_send == MotorOk)) {
        app_shoot_diag.send_count++;
    } else {
        app_shoot_diag.send_fail_count++;
    }
    if (zero_out != 0U) {
        app_shoot_diag.zero_count++;
    }
    app_shoot_diag.last_tick_ms = input->tick_ms;
    app_shoot_diag.last_time_us = (uint32_t)now_us;
    app_shoot_diag.shoot_status = shoot_status;
    app_shoot_diag.trigger_status = trigger_status;
    for (i = 0U; i < SHOOT_FRIC_MAX; i++) {
        app_shoot_diag.fric_status[i] = fric_status[i];
        app_shoot_diag.fric_current[i] = output->fric_current[i];
        app_shoot_diag.fric_temperature[i] = input->fric_fb[i].temperature;
    }
    app_shoot_diag.trigger_send_status = trigger_send;
    app_shoot_diag.fric_send_status = fric_send;
    app_shoot_diag.mode = output->mode;
    app_shoot_diag.trigger_current = output->trigger_current;
    app_shoot_diag.heat = input->heat;
    app_shoot_diag.heat_limit = input->heat_limit;
    app_shoot_diag.input_online = input->cmd.online;
    app_shoot_diag.trigger_online = input->trigger_fb.online;
    app_shoot_diag.fric_online_mask = fric_mask;
    app_shoot_diag.fric_temp_safe = temp_safe;
    app_shoot_diag.send_enable = output->send_enable;
    diag = app_shoot_diag;
    app_shoot_exit(primask);

    DBG_BOOL("app.shoot.in", diag.input_online != 0U);
    DBG_U32("app.shoot.mode", (uint32_t)diag.mode);
    DBG_I32("app.shoot.trig", (int32_t)diag.trigger_current);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Shoot 周期任务入口。
 *
 * @param argument CMSIS-RTOS2 用户参数，未使用。
 */
static void app_shoot_entry(void *argument)
{
    (void)argument;

    for (;;) {
        uint64_t now_us = bsp_timebase_now_us();
        ShootInput input;
        ShootOutput output;
        ShootStatus shoot_status;
        motor_status_t trigger_status;
        motor_status_t fric_status[SHOOT_FRIC_MAX];
        motor_status_t fric_send;
        motor_status_t trigger_send;
        uint8_t zero_out;
        uint8_t temp_safe;

        (void)memset(&output, 0, sizeof(output));
        app_shoot_fill_input(now_us, &input, &trigger_status, fric_status);
        temp_safe = app_shoot_temp_safe(&input);
        if ((app_shoot_feedback_ready(&input) != 0U) && (temp_safe != 0U)) {
            shoot_status = shoot_step(&app_shoot_ctx, &input, &output);
        } else {
            shoot_status = ShootState;
            (void)shoot_sync(&app_shoot_ctx);
        }

        app_shoot_send(&output, shoot_status, &fric_send, &trigger_send, &zero_out);
        app_shoot_commit(&input,
                         &output,
                         shoot_status,
                         trigger_status,
                         fric_status,
                         fric_send,
                         trigger_send,
                         zero_out,
                         temp_safe,
                         now_us);

        osDelay(APP_SHOOT_MS_TO_TICKS(APP_SHOOT_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_shoot_init(void)
{
    ShootCfg cfg;
    ShootStatus status;

    (void)memset(&app_shoot_ctx, 0, sizeof(app_shoot_ctx));
    (void)memset(&app_shoot_snapshot, 0, sizeof(app_shoot_snapshot));
    (void)memset(&app_shoot_input, 0, sizeof(app_shoot_input));
    (void)memset(&app_shoot_output, 0, sizeof(app_shoot_output));
    (void)memset(&app_shoot_diag, 0, sizeof(app_shoot_diag));
    (void)memset(&app_shoot_heat, 0, sizeof(app_shoot_heat));
    (void)memset(&app_shoot_impl, 0, sizeof(app_shoot_impl));

    app_shoot_make_cfg(&cfg);
    status = shoot_init(&app_shoot_ctx, &cfg, &app_shoot_impl);
    app_shoot_diag.shoot_status = status;
    if (status != ShootOk) {
        return 0U;
    }

    app_shoot_snapshot = app_shoot_ctx;
    app_shoot_thread = osThreadNew(app_shoot_entry, NULL, &app_shoot_attr);
    return (app_shoot_thread != NULL) ? 1U : 0U;
}

uint8_t app_shoot_get_state(Shoot *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_shoot_enter();
    *out = app_shoot_snapshot;
    app_shoot_exit(primask);
    return 1U;
}

uint8_t app_shoot_get_input(ShootInput *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_shoot_enter();
    *out = app_shoot_input;
    app_shoot_exit(primask);
    return 1U;
}

uint8_t app_shoot_get_output(ShootOutput *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_shoot_enter();
    *out = app_shoot_output;
    app_shoot_exit(primask);
    return 1U;
}

uint8_t app_shoot_get_diag(AppShootDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_shoot_enter();
    *out = app_shoot_diag;
    app_shoot_exit(primask);
    return 1U;
}

uint8_t app_shoot_set_heat(float32_t heat, float32_t limit)
{
    uint32_t primask;

    if ((isfinite(heat) == 0) || (isfinite(limit) == 0) || (heat < 0.0F) || (limit < 0.0F)) {
        return 0U;
    }

    primask = app_shoot_enter();
    app_shoot_heat.heat = heat;
    app_shoot_heat.limit = limit;
    app_shoot_heat.valid = 1U;
    app_shoot_exit(primask);

    return 1U;
}
