/**
 ******************************************************************************
 * @file    app_alarm.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   告警应用层实现。
 *
 * @note    本文件从 Daemon 读取最高优先级错误，并用蜂鸣器发出低优先级提示。
 * @note    本文件不修改 Daemon 状态，也不参与任何控制输出安全钳位。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_alarm.h"

#include "app_alarm_config.h"
#include "bsp_timebase.h"
#include "buzzer.h"
#include "main.h"
#include "cmsis_os.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_ALARM_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/**
 * @brief Alarm 内部响声序列状态。
 */
typedef struct
{
    AppDaemonItem item;      /**< 当前告警项。 */
    uint32_t phase_ms;       /**< 当前单响阶段已持续时间。 */
    uint32_t gap_ms;         /**< 一组响声结束后的静音倒计时。 */
    uint8_t beep_left;       /**< 当前组剩余响声次数。 */
    uint8_t beep_total;      /**< 当前组总响声次数。 */
    uint8_t active;          /**< 非 0 表示正在告警。 */
    uint8_t sounding;        /**< 非 0 表示当前蜂鸣器已打开。 */
} AppAlarmTone;

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 蜂鸣器告警 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief Alarm 任务控制块。 */
static osThreadId_t app_alarm_thread;

/** @brief Alarm 任务属性。 */
static const osThreadAttr_t app_alarm_attr = {
    .name = "app_alarm",
    .stack_size = APP_ALARM_STACK_SIZE,
    .priority = (osPriority_t)APP_ALARM_PRIORITY,
};

/** @brief Alarm 诊断缓存。 */
static AppAlarmDiag app_alarm_diag;

/** @brief Alarm 响声状态。 */
static AppAlarmTone app_alarm_tone;

/* ==========================================================================
 * 临界区与时间工具
 * --------------------------------------------------------------------------
 * 短临界区只保护快照复制和轻量状态切换，避免任务读写同一缓存时撕裂。
 * ========================================================================== */
/**
 * @brief 进入短临界区。
 *
 * @return 进入前 PRIMASK。
 */
static uint32_t app_alarm_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出短临界区。
 *
 * @param primask 进入前 PRIMASK。
 */
static void app_alarm_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 获取当前毫秒时间戳。
 *
 * @return 当前时间戳低 32 位，单位 ms。
 */
static uint32_t app_alarm_now_ms(void)
{
    return (uint32_t)(bsp_timebase_now_us() / 1000ULL);
}

/* ==========================================================================
 * 输出执行
 * --------------------------------------------------------------------------
 * 本区只调用设备或下层组件执行输出，失败时记录诊断并保持非阻塞语义。
 * ========================================================================== */
/**
 * @brief 关闭蜂鸣器并记录底层错误。
 */
static void app_alarm_silent(void)
{
    if (buzzer_off() != HAL_OK) {
        app_alarm_diag.buzzer_error++;
    }
    app_alarm_tone.sounding = 0U;
}

/**
 * @brief 打开蜂鸣器并记录底层错误。
 */
static void app_alarm_sound(void)
{
    if (buzzer_start_duty(APP_ALARM_FREQ_HZ, APP_ALARM_DUTY) != HAL_OK) {
        app_alarm_diag.buzzer_error++;
    }
    app_alarm_tone.sounding = 1U;
}

/**
 * @brief 根据健康项计算响声次数。
 *
 * @param item Daemon 健康项。
 * @return 本组告警响声次数。
 */
static uint8_t app_alarm_beep_count(AppDaemonItem item)
{
    uint32_t count = ((uint32_t)item + 1U);

    if (count == 0U) {
        count = 1U;
    }
    if (count > (uint32_t)APP_ALARM_MAX_BEEP) {
        count = (uint32_t)APP_ALARM_MAX_BEEP;
    }
    return (uint8_t)count;
}

/**
 * @brief 开始一组新的告警响声。
 *
 * @param item Daemon 健康项。
 */
static void app_alarm_start(AppDaemonItem item)
{
    uint8_t count = app_alarm_beep_count(item);

    (void)memset(&app_alarm_tone, 0, sizeof(app_alarm_tone));
    app_alarm_tone.item = item;
    app_alarm_tone.beep_left = count;
    app_alarm_tone.beep_total = count;
    app_alarm_tone.active = 1U;

    app_alarm_diag.warn_count++;
    app_alarm_diag.last_item = item;
    app_alarm_diag.last_beep_count = count;
    app_alarm_diag.active = 1U;
}

/**
 * @brief 停止当前告警。
 */
static void app_alarm_stop(void)
{
    if (app_alarm_tone.active != 0U) {
        app_alarm_diag.silence_count++;
    }
    (void)memset(&app_alarm_tone, 0, sizeof(app_alarm_tone));
    app_alarm_diag.active = 0U;
    app_alarm_silent();
}

/**
 * @brief 推进告警响声状态机。
 *
 * @param item 当前最高优先级错误项。
 */
static void app_alarm_tick(AppDaemonItem item)
{
    if ((app_alarm_tone.active == 0U) || (app_alarm_tone.item != item)) {
        app_alarm_start(item);
    }

    if (app_alarm_tone.gap_ms > 0U) {
        if (app_alarm_tone.gap_ms > APP_ALARM_PERIOD_MS) {
            app_alarm_tone.gap_ms -= APP_ALARM_PERIOD_MS;
        } else {
            app_alarm_tone.gap_ms = 0U;
            app_alarm_tone.beep_left = app_alarm_tone.beep_total;
            app_alarm_tone.phase_ms = 0U;
        }
        app_alarm_silent();
        return;
    }

    if (app_alarm_tone.beep_left == 0U) {
        app_alarm_tone.gap_ms = APP_ALARM_REPEAT_GAP_MS;
        app_alarm_silent();
        return;
    }

    app_alarm_tone.phase_ms += APP_ALARM_PERIOD_MS;
    if (app_alarm_tone.phase_ms < APP_ALARM_BEEP_OFF_MS) {
        app_alarm_silent();
        return;
    }

    if (app_alarm_tone.phase_ms < (APP_ALARM_BEEP_OFF_MS + APP_ALARM_BEEP_ON_MS)) {
        app_alarm_sound();
        return;
    }

    app_alarm_tone.phase_ms = 0U;
    app_alarm_tone.beep_left--;
    app_alarm_silent();
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief Alarm 任务入口。
 *
 * @param argument CMSIS-RTOS2 参数，未使用。
 */
static void app_alarm_entry(void *argument)
{
    AppDaemonState state;
    uint32_t primask;

    (void)argument;

    while (1) {
        if ((app_daemon_get_state(&state) == AppDaemonOk) &&
            (state.any_error != 0U) &&
            (state.highest_error < AppDaemonItemCount)) {
            app_alarm_tick(state.highest_error);
        } else {
            app_alarm_stop();
        }

        primask = app_alarm_enter();
        app_alarm_diag.loop_count++;
        app_alarm_diag.last_tick_ms = app_alarm_now_ms();
        app_alarm_exit(primask);

        osDelay(APP_ALARM_MS_TO_TICKS(APP_ALARM_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_alarm_init(void)
{
    (void)memset(&app_alarm_diag, 0, sizeof(app_alarm_diag));
    (void)memset(&app_alarm_tone, 0, sizeof(app_alarm_tone));
    app_alarm_diag.last_item = AppDaemonItemCount;
    app_alarm_silent();

    app_alarm_thread = osThreadNew(app_alarm_entry, NULL, &app_alarm_attr);
    app_alarm_diag.ready = (app_alarm_thread != NULL) ? 1U : 0U;
    return (app_alarm_thread != NULL) ? 1U : 0U;
}

uint8_t app_alarm_get_diag(AppAlarmDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_alarm_enter();
    *out = app_alarm_diag;
    app_alarm_exit(primask);
    return 1U;
}
