/**
 ******************************************************************************
 * @file    template.c
 * @author  <你的姓名>
 * @version V1.0.0
 * @date    <年>-<月>-<日>
 * @brief   应用层模块模板。
 *
 * @note
 *          本文件是 APP 级常驻任务的参考模板。
 *          请用具体模块实现替换占位上下文、请求、待应用配置和 step 逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/**
 ******************************************************************************
 * 引入配套头文件。<请删除本段>
 ******************************************************************************
 */
#include "template.h"

/**
 ******************************************************************************
 * 引入本地和第三方依赖头文件。<请删除本段>
 ******************************************************************************
 */
#include "cmsis_os.h"
#include "main.h"

/**
 ******************************************************************************
 * 引入标准头文件。<请删除本段>
 ******************************************************************************
 */
#include <stdint.h>
#include <string.h>

/**
 ******************************************************************************
 * 私有参数宏。<请删除本段>
 ******************************************************************************
 */
#ifndef TEMPLATE_INIT_DELAY_MS
#define TEMPLATE_INIT_DELAY_MS (0U)
#endif

#ifndef TEMPLATE_PERIOD_MS
#define TEMPLATE_PERIOD_MS (1U)
#endif

/** @brief 模板任务启动延时，单位 ms。 */
#define APP_TEMPLATE_INIT_DELAY_MS ((uint32_t)TEMPLATE_INIT_DELAY_MS)

/** @brief 模板任务栈大小，单位 byte。 */
#define APP_TEMPLATE_STACK_SIZE ((uint32_t)1024U)

/** @brief 模板任务优先级。 */
#define APP_TEMPLATE_PRIORITY osPriorityAboveNormal

/**
 ******************************************************************************
 * 私有宏函数。<请删除本段>
 ******************************************************************************
 */
#define APP_TEMPLATE_MS_TO_TICKS(ms) \
    ((uint32_t)((((uint32_t)(ms) * osKernelGetTickFreq()) + 999U) / 1000U))

/**
 ******************************************************************************
 * 私有类型、结构体和枚举。<请删除本段>
 ******************************************************************************
 */
typedef struct
{
    uint8_t initialized;       /**< APP 上下文初始化后为非零。 */
    AppTemplateMode mode;      /**< 当前运行模式。 */
    uint32_t period_ms;        /**< 任务周期，单位 ms。 */
    uint32_t loop_count;       /**< 已执行的任务迭代次数。 */
    uint32_t last_tick;        /**< 最近一次更新 tick。 */
    AppTemplateEventCb event;  /**< 事件回调。 */
    void *user;                /**< 事件回调用户上下文。 */
} AppTemplateCtx;

typedef struct
{
    int16_t command;           /**< 示例外部命令或覆盖请求。 */
    uint32_t last_req_tick;    /**< 最近一次请求更新的 tick。 */
    uint8_t active;            /**< 请求激活时为非零。 */
} AppTemplateReqCtrl;

typedef struct
{
    uint8_t valid;             /**< 存在待应用配置时为非零。 */
    uint32_t param0;           /**< 占位配置参数 0。 */
    uint32_t param1;           /**< 占位配置参数 1。 */
} AppTemplatePendingConfig;

/**
 ******************************************************************************
 * 私有全局变量。<请删除本段>
 ******************************************************************************
 */
/** @brief 模板应用上下文。 */
static AppTemplateCtx app_template_ctx;

/** @brief 模板应用诊断快照。 */
static AppTemplateDiag app_template_diag;

/** @brief 模板外部请求控制缓存。 */
static AppTemplateReqCtrl app_template_req_ctrl;

/** @brief APP 初始化前收到的模板配置。 */
static AppTemplatePendingConfig app_template_pending_config;

/** @brief 模板任务控制块。 */
static osThreadId_t app_template_thread;

/** @brief 模板任务属性。 */
static const osThreadAttr_t app_template_attr = {
    .name = "app_template",
    .stack_size = APP_TEMPLATE_STACK_SIZE,
    .priority = (osPriority_t)APP_TEMPLATE_PRIORITY,
};

/**
 ******************************************************************************
 * 私有静态函数声明。<请删除本段>
 ******************************************************************************
 */
static uint32_t app_template_enter(void);
static void app_template_leave(uint32_t primask);
static void app_template_reset(const AppTemplateConfig *config);
static void app_template_notify(AppTemplateEvent event);
static void app_template_apply_config(void);
static void app_template_update_diag(uint32_t tick);
static void app_template_step(void);
static void app_template_entry(void *argument);

/**
 ******************************************************************************
 * 私有静态函数实现。<请删除本段>
 ******************************************************************************
 */
static uint32_t app_template_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void app_template_leave(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void app_template_reset(const AppTemplateConfig *config)
{
    memset(&app_template_ctx, 0, sizeof(app_template_ctx));
    memset(&app_template_diag, 0, sizeof(app_template_diag));
    memset(&app_template_req_ctrl, 0, sizeof(app_template_req_ctrl));

    app_template_ctx.mode = AppTemplateModeRun;
    app_template_ctx.period_ms = app_template_period(TEMPLATE_PERIOD_MS);
    if (config != NULL) {
        app_template_ctx.mode = config->mode;
        app_template_ctx.period_ms = app_template_period(config->period_ms);
        app_template_ctx.event = config->event;
        app_template_ctx.user = config->user;
    }

    app_template_ctx.initialized = 1U;
}

static void app_template_notify(AppTemplateEvent event)
{
    if (app_template_ctx.event != NULL) {
        app_template_ctx.event(event, app_template_ctx.user);
    }
}

static void app_template_apply_config(void)
{
    if (app_template_pending_config.valid == 0U) {
        return;
    }

    /* 待办：将 app_template_pending_config.param0 / param1 应用到真实模块。 */

    app_template_pending_config.valid = 0U;
}

static void app_template_update_diag(uint32_t tick)
{
    app_template_diag.initialized = app_template_ctx.initialized;
    app_template_diag.mode = app_template_ctx.mode;
    app_template_diag.request_active = app_template_req_ctrl.active;
    app_template_diag.command = app_template_req_ctrl.command;
    app_template_diag.loop_count = app_template_ctx.loop_count;
    app_template_diag.last_tick = tick;
}

static void app_template_step(void)
{
    uint32_t tick = osKernelGetTickCount();
    uint32_t primask;

    /* 待办：读取输入，执行观测器、控制或服务逻辑，然后写入输出。 */

    primask = app_template_enter();
    app_template_ctx.loop_count++;
    app_template_ctx.last_tick = tick;
    app_template_update_diag(tick);
    app_template_leave(primask);
}

static void app_template_entry(void *argument)
{
    uint32_t period_ticks;
    uint32_t wake_tick;

    (void)argument;

    if (APP_TEMPLATE_INIT_DELAY_MS != 0U) {
        osDelay(APP_TEMPLATE_MS_TO_TICKS(APP_TEMPLATE_INIT_DELAY_MS));
    }

    period_ticks = APP_TEMPLATE_MS_TO_TICKS(app_template_ctx.period_ms);
    if (period_ticks == 0U) {
        period_ticks = 1U;
    }

    app_template_notify(AppTemplateEventStarted);

    wake_tick = osKernelGetTickCount();
    for (;;) {
        app_template_step();
        wake_tick += period_ticks;
        osDelayUntil(wake_tick);
    }
}

/**
 ******************************************************************************
 * 公共函数实现。<请删除本段>
 ******************************************************************************
 */
AppTemplateStatus app_template_init(const AppTemplateConfig *config)
{
    uint32_t primask;

    if (app_template_thread != NULL) {
        return AppTemplateOk;
    }
    if ((config != NULL) && (app_template_mode_valid(config->mode) == 0U)) {
        return AppTemplateParamError;
    }

    primask = app_template_enter();
    app_template_reset(config);
    app_template_apply_config();
    app_template_leave(primask);

    app_template_thread = osThreadNew(app_template_entry, NULL, &app_template_attr);
    return (app_template_thread != NULL) ? AppTemplateOk : AppTemplateError;
}

void app_template_config_set(uint32_t param0, uint32_t param1)
{
    uint32_t primask = app_template_enter();

    app_template_pending_config.param0 = param0;
    app_template_pending_config.param1 = param1;
    app_template_pending_config.valid = 1U;

    if (app_template_ctx.initialized != 0U) {
        app_template_apply_config();
    }

    app_template_leave(primask);
}

void app_template_request_set(int16_t command, uint8_t active)
{
    uint32_t primask = app_template_enter();

    app_template_req_ctrl.command = command;
    app_template_req_ctrl.last_req_tick = osKernelGetTickCount();
    app_template_req_ctrl.active = (active != 0U) ? 1U : 0U;

    app_template_leave(primask);
    app_template_notify(AppTemplateEventRequest);
}

void app_template_diag_get(AppTemplateDiag *diag)
{
    uint32_t primask;

    if (diag == NULL) {
        return;
    }

    primask = app_template_enter();
    *diag = app_template_diag;
    app_template_leave(primask);
}
