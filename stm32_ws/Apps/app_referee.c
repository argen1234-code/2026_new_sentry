/**
 ******************************************************************************
 * @file    app_referee.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Referee 应用层实现。
 *
 * @note    本文件只做裁判系统状态维护、热量/功率注入和客户端 UI 发送调度。
 * @note    裁判系统离线或关键有效位不足时，不向下游控制 app 注入过期数据。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_referee.h"

#include "app_referee_config.h"
#include "bsp_timebase.h"
#include "main.h"
#include "cmsis_os.h"

#if ROBOT_APP_ENABLE_CHASSIS != 0U
#include "app_chassis.h"
#endif
#if ROBOT_APP_ENABLE_SHOOT != 0U
#include "app_shoot.h"
#endif

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_REFEREE_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 裁判状态维护 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Referee 任务控制块。 */
static osThreadId_t app_referee_thread;

/** @brief Referee 任务属性。 */
static const osThreadAttr_t app_referee_attr = {
    .name = "app_referee",
    .stack_size = APP_REFEREE_STACK_SIZE,
    .priority = (osPriority_t)APP_REFEREE_PRIORITY,
};

/** @brief Referee 应用层状态缓存。 */
static AppRefereeState app_referee_state;

/** @brief Referee 应用层诊断缓存。 */
static AppRefereeDiag app_referee_diag;

/** @brief 最近一次裁判系统原始状态缓存。 */
static RefereeState app_referee_raw;

/** @brief 最近一次裁判系统原始状态是否有效。 */
static uint8_t app_referee_raw_valid;

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/** @brief UI 组件实例。 */
static Ui app_referee_ui;

/** @brief UI 当前绑定的机器人 ID。 */
static uint8_t app_referee_ui_robot_id;

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
static uint32_t app_referee_enter(void)
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
static void app_referee_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 判断周期事件是否到期。
 *
 * @param now_ms 当前时间戳，单位 ms。
 * @param last_ms 上次触发时间戳。
 * @param period_ms 周期，单位 ms。
 * @return 到期返回 1，否则返回 0。
 */
static uint8_t app_referee_due(uint32_t now_ms, uint32_t *last_ms, uint32_t period_ms)
{
    if (last_ms == NULL) {
        return 0U;
    }

    if ((*last_ms == 0U) || (period_ms == 0U) || ((uint32_t)(now_ms - *last_ms) >= period_ms)) {
        *last_ms = now_ms;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 根据机器人 ID 计算客户端 ID。
 *
 * @param robot_id 裁判系统机器人 ID。
 * @return 对应客户端 ID。
 */
static uint16_t app_referee_client_id(uint8_t robot_id)
{
    return (uint16_t)((uint16_t)robot_id + (uint16_t)UI_CLIENT_ID_OFFSET);
}

/**
 * @brief 从原始状态中读取指定发射机构热量。
 *
 * @param raw 原始裁判状态。
 * @param heat 当前热量输出。
 * @param limit 热量上限输出。
 * @return 成功返回 1，输入无效返回 0。
 */
static uint8_t app_referee_heat_from_raw(const RefereeState *raw, float *heat, float *limit)
{
    if ((raw == NULL) || (heat == NULL) || (limit == NULL)) {
        return 0U;
    }

    if ((raw->valid_mask & (REFEREE_VALID_ROBOT_STATUS | REFEREE_VALID_POWER_HEAT)) !=
        (REFEREE_VALID_ROBOT_STATUS | REFEREE_VALID_POWER_HEAT)) {
        return 0U;
    }

    *limit = (float)raw->robot_status.shooter_barrel_heat_limit;
    if (APP_REFEREE_SHOOTER_ID == 1U) {
        *heat = (float)raw->power_heat.shooter_17mm_1_barrel_heat;
    } else if (APP_REFEREE_SHOOTER_ID == 2U) {
        *heat = (float)raw->power_heat.shooter_17mm_2_barrel_heat;
    } else {
        return 0U;
    }

    return 1U;
}

/* ==========================================================================
 * 配置生成与参数展开
 * --------------------------------------------------------------------------
 * 本区把 Robot target 宏展开成 Component/Device 的 Cfg，运行期控制逻辑不写在配置头里。
 * ========================================================================== */
/**
 * @brief 从裁判原始状态生成应用层快照。
 *
 * @param raw 裁判原始状态。
 * @param online 非 0 表示裁判系统在线。
 * @param now_ms 当前时间，单位 ms。
 * @param out 应用层状态输出。
 */
static void app_referee_make_state(const RefereeState *raw,
                                   uint8_t online,
                                   uint32_t now_ms,
                                   AppRefereeState *out)
{
    float heat;
    float limit;

    if ((raw == NULL) || (out == NULL)) {
        return;
    }

    (void)memset(out, 0, sizeof(*out));
    out->online = online;
    out->valid_mask = raw->valid_mask;
    out->last_rx_tick = raw->last_rx_tick;
    out->last_update_ms = now_ms;
    out->stale = ((online == 0U) && (raw->last_rx_tick != 0U)) ? 1U : 0U;

    if ((raw->valid_mask & REFEREE_VALID_ROBOT_STATUS) != 0U) {
        out->robot_id = raw->robot_status.robot_id;
        out->client_id = app_referee_client_id(raw->robot_status.robot_id);
    }
    if ((raw->valid_mask & REFEREE_VALID_WARNING) != 0U) {
        out->warning = raw->warning;
    }
    if ((raw->valid_mask & REFEREE_VALID_HURT) != 0U) {
        out->hurt = raw->hurt;
    }
    if ((raw->valid_mask & REFEREE_VALID_SHOOT_DATA) != 0U) {
        out->shoot_data = raw->shoot_data;
    }
    if (app_referee_heat_from_raw(raw, &heat, &limit) != 0U) {
        out->heat_valid = online;
        out->shoot_heat = heat;
        out->shoot_heat_limit = limit;
        out->power_valid = online;
        out->chassis_power = 0.0F;
        out->chassis_power_limit = (float)raw->robot_status.chassis_power_limit;
        out->chassis_buffer = (float)raw->power_heat.buffer_energy;
    }
}

/* ==========================================================================
 * 诊断快照
 * --------------------------------------------------------------------------
 * 周期末统一提交快照和诊断，其他模块只能读取稳定副本。
 * ========================================================================== */
/**
 * @brief 提交原始裁判状态和应用层状态。
 *
 * @param raw 裁判原始状态。
 * @param state 应用层状态。
 */
static void app_referee_commit_state(const RefereeState *raw, const AppRefereeState *state)
{
    uint32_t primask;

    if ((raw == NULL) || (state == NULL)) {
        return;
    }

    primask = app_referee_enter();
    app_referee_raw = *raw;
    app_referee_raw_valid = 1U;
    app_referee_state = *state;
    app_referee_exit(primask);
}

/**
 * @brief 记录一次状态周期诊断。
 *
 * @param state_status 裁判状态读取状态。
 * @param diag_status 裁判诊断读取状态。
 * @param state 应用层状态。
 */
static void app_referee_record_state(RefereeStatus state_status,
                                     RefereeStatus diag_status,
                                     const AppRefereeState *state)
{
    uint32_t primask;

    primask = app_referee_enter();
    app_referee_diag.state_status = state_status;
    app_referee_diag.diag_status = diag_status;
    if ((state == NULL) || (state->online == 0U)) {
        app_referee_diag.offline_count++;
    }
    if ((state != NULL) && (state->stale != 0U)) {
        app_referee_diag.stale_count++;
    }
    if ((state != NULL) && (state->online != 0U) &&
        ((state->heat_valid == 0U) || (state->robot_id == 0U))) {
        app_referee_diag.valid_fail_count++;
    }
    app_referee_exit(primask);
}

/**
 * @brief 向 Shoot app 注入当前热量。
 *
 * @param online 裁判系统是否在线。
 */
static void app_referee_inject_shoot(uint8_t online)
{
#if ROBOT_APP_ENABLE_SHOOT != 0U
    RefereeStatus status;
    uint16_t heat = 0U;
    uint16_t limit = 0U;
    uint8_t ok = 0U;
    uint32_t primask;

    if (online == 0U) {
        return;
    }

    status = referee_shoot_heat(APP_REFEREE_SHOOTER_ID, &limit, &heat);
    if (status == RefereeOk) {
        ok = app_shoot_set_heat((float32_t)heat, (float32_t)limit);
    }

    primask = app_referee_enter();
    app_referee_diag.heat_status = status;
    if ((status == RefereeOk) && (ok != 0U)) {
        app_referee_diag.heat_inject_count++;
    } else {
        app_referee_diag.heat_fail_count++;
    }
    app_referee_exit(primask);
#else
    (void)online;
#endif
}

/**
 * @brief 向 Chassis app 注入当前功率预算。
 *
 * @param online 裁判系统是否在线。
 */
static void app_referee_inject_chassis(uint8_t online)
{
#if ROBOT_APP_ENABLE_CHASSIS != 0U
    RefereeStatus status;
    float limit = 0.0F;
    float buffer = 0.0F;
    uint8_t ok = 0U;
    uint32_t primask;

    if (online == 0U) {
        return;
    }

    status = referee_chassis_budget(&limit, &buffer);
    if (status == RefereeOk) {
        ok = app_chassis_set_power(0.0F, (float32_t)limit, (float32_t)buffer);
    }

    primask = app_referee_enter();
    app_referee_diag.power_status = status;
    if ((status == RefereeOk) && (ok != 0U)) {
        app_referee_diag.power_inject_count++;
    } else {
        app_referee_diag.power_fail_count++;
    }
    app_referee_exit(primask);
#else
    (void)online;
#endif
}

/**
 * @brief 按当前机器人 ID 初始化或重置 UI。
 *
 * @param robot_id 当前机器人 ID。
 * @return 成功返回 1，失败返回 0。
 */
static uint8_t app_referee_ensure_ui(uint8_t robot_id)
{
    UiCfg cfg;
    UiStatus status;
    uint32_t primask;

    if (robot_id == 0U) {
        return 0U;
    }

    if ((app_referee_diag.ui_ready != 0U) && (app_referee_ui_robot_id == robot_id)) {
        return 1U;
    }

    cfg.sender_id = (uint16_t)robot_id;
    cfg.receiver_id = app_referee_client_id(robot_id);
    cfg.send_per_step = (uint8_t)APP_REFEREE_UI_SEND_PER_STEP;

    status = ui_init(&app_referee_ui, &cfg);

    primask = app_referee_enter();
    app_referee_diag.ui_init_status = status;
    if (status == UiOk) {
        app_referee_diag.ui_ready = 1U;
        app_referee_diag.ui_init_count++;
        app_referee_ui_robot_id = robot_id;
    } else {
        app_referee_diag.ui_ready = 0U;
        app_referee_diag.ui_init_fail_count++;
        app_referee_ui_robot_id = 0U;
    }
    app_referee_exit(primask);

    return (status == UiOk) ? 1U : 0U;
}

/**
 * @brief 按节流周期刷新 UI。
 *
 * @param state 当前应用层状态。
 * @param now_ms 当前时间，单位 ms。
 * @param last_ui_ms 上次 UI step 时间。
 */
static void app_referee_step_ui(const AppRefereeState *state, uint32_t now_ms, uint32_t *last_ui_ms)
{
    UiStatus status;
    uint32_t primask;

    if ((state == NULL) || (state->online == 0U) || (state->robot_id == 0U)) {
        return;
    }
    if (app_referee_ensure_ui(state->robot_id) == 0U) {
        return;
    }
    if (app_referee_due(now_ms, last_ui_ms, APP_REFEREE_UI_PERIOD_MS) == 0U) {
        return;
    }

    status = ui_step(&app_referee_ui);

    primask = app_referee_enter();
    app_referee_diag.ui_status = status;
    if (status == UiOk) {
        app_referee_diag.ui_send_count++;
    } else {
        app_referee_diag.ui_fail_count++;
    }
    app_referee_exit(primask);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Referee 任务入口。
 *
 * @param argument CMSIS-RTOS2 输入参数。
 */
static void app_referee_entry(void *argument)
{
    RefereeState raw;
    RefereeDiag raw_diag;
    AppRefereeState state;
    RefereeStatus state_status;
    RefereeStatus diag_status;
    uint32_t now_us;
    uint32_t now_ms;
    uint32_t last_ui_ms = 0U;
    uint32_t primask;
    uint8_t online;

    (void)argument;

    while (1) {
        now_us = (uint32_t)bsp_timebase_now_us();
        now_ms = now_us / 1000U;
        (void)memset(&raw, 0, sizeof(raw));
        (void)memset(&raw_diag, 0, sizeof(raw_diag));

        state_status = referee_get(&raw);
        diag_status = referee_diag(&raw_diag);
        online = ((state_status == RefereeOk) &&
                  (referee_online(APP_REFEREE_TIMEOUT_MS, now_ms) != 0U)) ? 1U : 0U;

        app_referee_make_state(&raw, online, now_ms, &state);
        if (state_status == RefereeOk) {
            app_referee_commit_state(&raw, &state);
        }
        app_referee_record_state(state_status, diag_status, &state);

        if (online != 0U) {
            app_referee_inject_shoot(online);
            app_referee_inject_chassis(online);
            app_referee_step_ui(&state, now_ms, &last_ui_ms);
        }

        primask = app_referee_enter();
        app_referee_diag.loop_count++;
        app_referee_diag.last_tick_ms = now_ms;
        app_referee_diag.last_time_us = now_us;
        app_referee_exit(primask);

        osDelay(APP_REFEREE_MS_TO_TICKS(APP_REFEREE_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_referee_init(void)
{
    RefereeStatus status;
    uint32_t primask;

    (void)memset(&app_referee_state, 0, sizeof(app_referee_state));
    (void)memset(&app_referee_diag, 0, sizeof(app_referee_diag));
    (void)memset(&app_referee_raw, 0, sizeof(app_referee_raw));
    (void)memset(&app_referee_ui, 0, sizeof(app_referee_ui));
    app_referee_raw_valid = 0U;
    app_referee_ui_robot_id = 0U;

    status = referee_init();

    primask = app_referee_enter();
    app_referee_diag.init_status = status;
    app_referee_diag.state_status = status;
    app_referee_diag.heat_status = RefereeBadState;
    app_referee_diag.power_status = RefereeBadState;
    app_referee_diag.ui_init_status = UiBadState;
    app_referee_diag.ui_status = UiBadState;
    app_referee_diag.ready = (status == RefereeOk) ? 1U : 0U;
    app_referee_exit(primask);

    if (status != RefereeOk) {
        return 0U;
    }

    app_referee_thread = osThreadNew(app_referee_entry, NULL, &app_referee_attr);
    return (app_referee_thread != NULL) ? 1U : 0U;
}

uint8_t app_referee_get_state(AppRefereeState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_referee_enter();
    *out = app_referee_state;
    app_referee_exit(primask);
    return 1U;
}

uint8_t app_referee_get_diag(AppRefereeDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_referee_enter();
    *out = app_referee_diag;
    app_referee_exit(primask);
    return 1U;
}

uint8_t app_referee_get_raw(RefereeState *out)
{
    uint32_t primask;
    uint8_t valid;

    if (out == NULL) {
        return 0U;
    }

    primask = app_referee_enter();
    valid = app_referee_raw_valid;
    if (valid != 0U) {
        *out = app_referee_raw;
    }
    app_referee_exit(primask);
    return valid;
}

uint8_t app_referee_get_ui_diag(UiDiag *out)
{
    if ((out == NULL) || (app_referee_diag.ui_ready == 0U)) {
        return 0U;
    }

    return (ui_diag(&app_referee_ui, out) == UiOk) ? 1U : 0U;
}
