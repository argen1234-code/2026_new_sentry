/**
 ******************************************************************************
 * @file    app_magazine.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   弹舱应用层实现。
 *
 * @note    本文件只做 Input 弹舱命令到 Devices/Magazine 的应用层接线。
 * @note    本文件不读取遥控器、不参与发射闭环，也不恢复旧 shoot 全局逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "app_magazine.h"

#include "app_input.h"
#include "app_magazine_config.h"
#include "bsp_timebase.h"
#include "debug.h"
#include "magazine.h"
#include "main.h"
#include "cmsis_os.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置本 App 的私有宏、任务属性和内部状态类型，便于先建立文件边界。
 * ========================================================================== */
/** @brief 将毫秒转换为 CMSIS-RTOS2 tick，向上取整避免短周期被截断。 */
#define APP_MAGAZINE_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/* ==========================================================================
 * 任务对象与状态缓存
 * --------------------------------------------------------------------------
 * 弹舱附件 的全局状态只在本文件内保存，对外通过快照 API 暴露。
 * ========================================================================== */
/** @brief 弹舱任务控制块。 */
static osThreadId_t app_magazine_thread;

/** @brief 弹舱任务属性。 */
static const osThreadAttr_t app_magazine_attr = {
    .name = "app_magazine",
    .stack_size = APP_MAGAZINE_STACK_SIZE,
    .priority = (osPriority_t)APP_MAGAZINE_PRIORITY,
};

/** @brief 弹舱当前状态。 */
static AppMagazineState app_magazine_state;

/** @brief 弹舱诊断快照。 */
static AppMagazineDiag app_magazine_diag;

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
static uint32_t app_magazine_enter(void)
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
static void app_magazine_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 判断命令动作是否已经等于当前状态。
 *
 * @param action 命令动作。
 * @return 已处于目标状态返回 1，否则返回 0。
 */
static uint8_t app_magazine_same(InputMagAction action)
{
    if ((action == InputMagOpen) && (app_magazine_state == AppMagazineOpen)) {
        return 1U;
    }
    if ((action == InputMagClose) && (app_magazine_state == AppMagazineClosed)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 执行弹舱设备层动作。
 *
 * @param action 命令动作。
 * @return HAL_OK 表示成功，否则表示设备层失败。
 */
static HAL_StatusTypeDef app_magazine_do(InputMagAction action)
{
    if (action == InputMagOpen) {
        return magazine_open();
    }
    if (action == InputMagClose) {
        return magazine_close();
    }

    return HAL_OK;
}

/**
 * @brief 记录无需动作的保持周期。
 *
 * @param cmd 当前命令，可为空。
 */
static void app_magazine_hold(const InputMagCmd *cmd)
{
    uint32_t primask;

    primask = app_magazine_enter();
    app_magazine_diag.hold_count++;
    app_magazine_diag.input_online = (cmd != NULL) ? cmd->online : 0U;
    app_magazine_diag.cmd_valid = (cmd != NULL) ? cmd->valid : 0U;
    app_magazine_diag.last_action = (cmd != NULL) ? cmd->action : InputMagHold;
    app_magazine_diag.io_ok = 1U;
    app_magazine_diag.state = app_magazine_state;
    app_magazine_exit(primask);
}

/* ==========================================================================
 * 任务入口
 * --------------------------------------------------------------------------
 * 任务入口只按固定周期串起采集、step、输出和诊断，不在这里新增分支协议。
 * ========================================================================== */
/**
 * @brief 弹舱周期任务入口。
 *
 * @param argument CMSIS-RTOS2 用户参数，未使用。
 */
static void app_magazine_entry(void *argument)
{
    (void)argument;

    for (;;) {
        InputMagCmd cmd;
        uint32_t primask;

        (void)memset(&cmd, 0, sizeof(cmd));
        (void)app_input_get_magazine_cmd(&cmd);
        (void)app_magazine_set_cmd(&cmd);

        primask = app_magazine_enter();
        app_magazine_diag.loop_count++;
        app_magazine_diag.last_tick_ms = (uint32_t)(bsp_timebase_now_us() / 1000ULL);
        app_magazine_exit(primask);

        DBG_U32("app.mag.state", (uint32_t)app_magazine_state);
        osDelay(APP_MAGAZINE_MS_TO_TICKS(APP_MAGAZINE_PERIOD_MS));
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 提供初始化、快照读取和显式注入，保持 App 之间的依赖可追踪。
 * ========================================================================== */
uint8_t app_magazine_init(void)
{
    (void)memset(&app_magazine_diag, 0, sizeof(app_magazine_diag));
    app_magazine_state = AppMagazineUnknown;

    if (magazine_init() != HAL_OK) {
        app_magazine_diag.io_fail_count++;
        app_magazine_diag.io_ok = 0U;
        app_magazine_diag.state = AppMagazineUnknown;
        return 0U;
    }

    app_magazine_state = AppMagazineClosed;
    app_magazine_diag.io_ok = 1U;
    app_magazine_diag.state = app_magazine_state;

    app_magazine_thread = osThreadNew(app_magazine_entry, NULL, &app_magazine_attr);
    return (app_magazine_thread != NULL) ? 1U : 0U;
}

uint8_t app_magazine_set_cmd(const InputMagCmd *cmd)
{
    HAL_StatusTypeDef io_status;
    uint32_t primask;

    if ((cmd == NULL) || (cmd->online == 0U) || (cmd->valid == 0U) || (cmd->action == InputMagHold)) {
        app_magazine_hold(cmd);
        return 1U;
    }

    primask = app_magazine_enter();
    app_magazine_diag.cmd_count++;
    app_magazine_diag.input_online = cmd->online;
    app_magazine_diag.cmd_valid = cmd->valid;
    app_magazine_diag.last_action = cmd->action;
    app_magazine_exit(primask);

    if (app_magazine_same(cmd->action) != 0U) {
        app_magazine_hold(cmd);
        return 1U;
    }

    io_status = app_magazine_do(cmd->action);

    primask = app_magazine_enter();
    if (io_status == HAL_OK) {
        if (cmd->action == InputMagOpen) {
            app_magazine_state = AppMagazineOpen;
            app_magazine_diag.open_count++;
        } else if (cmd->action == InputMagClose) {
            app_magazine_state = AppMagazineClosed;
            app_magazine_diag.close_count++;
        }
        app_magazine_diag.io_ok = 1U;
    } else {
        app_magazine_state = AppMagazineUnknown;
        app_magazine_diag.io_fail_count++;
        app_magazine_diag.io_ok = 0U;
    }
    app_magazine_diag.state = app_magazine_state;
    app_magazine_exit(primask);

    return (io_status == HAL_OK) ? 1U : 0U;
}

uint8_t app_magazine_get_state(AppMagazineState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_magazine_enter();
    *out = app_magazine_state;
    app_magazine_exit(primask);
    return 1U;
}

uint8_t app_magazine_get_diag(AppMagazineDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = app_magazine_enter();
    *out = app_magazine_diag;
    app_magazine_exit(primask);
    return 1U;
}
