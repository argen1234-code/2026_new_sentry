/**
 ******************************************************************************
 * @file    template_doxygen.h
 * @author  <你的姓名>
 * @version V1.0.0
 * @date    <年>-<月>-<日>
 * @brief   应用层模块 Doxygen 注释参考头文件。
 *
 * @details
 *          本文件用于展示嵌入式 C 项目中头文件常见对象的 Doxygen 注释写法。
 *          模板覆盖公共宏、枚举、结构体、回调、内联函数和公共 API。
 *
 * @note    头文件只描述接口契约，不暴露模块内部状态和私有实现细节。
 * @note    私有静态函数、私有宏和私有全局变量的注释应写在对应 .c 文件中。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_TEMPLATE_DOXYGEN_H
#define PRINTK_TEMPLATE_DOXYGEN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * 引入标准头文件。
 ******************************************************************************
 */
#include <stdint.h>

/**
 * @defgroup AppTemplate 应用层模板模块
 * @brief 应用层常驻任务模块接口示例。
 *
 * @details
 *          本组展示一个 APP 层模块的典型公共接口组织方式。实际模块应按业务裁剪，
 *          删除无用配置、事件和 API。
 * @{
 */

/**
 ******************************************************************************
 * 公共宏定义。
 ******************************************************************************
 */
/** @brief 模板模块主版本号。 */
#define APP_TEMPLATE_VER_MAJOR 1U

/** @brief 模板模块次版本号。 */
#define APP_TEMPLATE_VER_MINOR 0U

/** @brief 模板模块修订号。 */
#define APP_TEMPLATE_VER_PATCH 0U

/** @brief 模板任务默认周期，单位 ms。 */
#define APP_TEMPLATE_PERIOD_DEFAULT 1U

/**
 ******************************************************************************
 * 公共类型、结构体、枚举和回调。
 ******************************************************************************
 */
/**
 * @brief 模板模块状态码。
 */
typedef enum
{
    AppTemplateOk = 0,       /**< 操作成功。 */
    AppTemplateError,        /**< 通用错误。 */
    AppTemplateParamError,   /**< 参数无效。 */
    AppTemplateBusy          /**< 模块忙。 */
} AppTemplateStatus;

/**
 * @brief 模板模块运行模式。
 */
typedef enum
{
    AppTemplateModeIdle = 0, /**< 空闲模式。 */
    AppTemplateModeRun       /**< 运行模式。 */
} AppTemplateMode;

/**
 * @brief 模板模块事件。
 */
typedef enum
{
    AppTemplateEventStarted = 0, /**< 任务已启动。 */
    AppTemplateEventRequest      /**< 收到外部请求。 */
} AppTemplateEvent;

/**
 * @brief 模板模块事件回调类型。
 *
 * @param event 事件类型。
 * @param user 用户上下文。
 *
 * @note 回调中不应执行阻塞等待或长时间计算。
 */
typedef void (*AppTemplateEventCb)(AppTemplateEvent event, void *user);

/**
 * @brief 模板模块初始化配置。
 */
typedef struct
{
    AppTemplateMode mode;      /**< 初始运行模式。 */
    uint32_t period_ms;        /**< 任务周期，单位 ms；为 0 时使用默认周期。 */
    AppTemplateEventCb event;  /**< 事件回调；为 NULL 时不通知事件。 */
    void *user;                /**< 事件回调用户上下文。 */
} AppTemplateConfig;

/**
 * @brief 模板模块诊断快照。
 *
 * @note 诊断结构体用于暴露只读运行状态，适合被调试接口或上位机周期性读取。
 */
typedef struct
{
    uint8_t initialized;       /**< APP 上下文初始化后为非零。 */
    AppTemplateMode mode;      /**< 当前运行模式。 */
    uint8_t request_active;    /**< 外部请求状态。 */
    int16_t command;           /**< 最近一次命令快照。 */
    uint32_t loop_count;       /**< 已执行的任务迭代次数。 */
    uint32_t last_tick;        /**< 最近一次更新 tick。 */
} AppTemplateDiag;

/**
 ******************************************************************************
 * 公共内联函数。
 ******************************************************************************
 */
/**
 * @brief 判断状态码是否表示成功。
 *
 * @param status 模板模块状态码。
 * @return 成功返回 1，否则返回 0。
 */
static inline uint8_t app_template_is_ok(AppTemplateStatus status)
{
    return (status == AppTemplateOk) ? 1U : 0U;
}

/**
 * @brief 判断运行模式是否有效。
 *
 * @param mode 运行模式。
 * @return 有效返回 1，否则返回 0。
 */
static inline uint8_t app_template_mode_valid(AppTemplateMode mode)
{
    return ((mode == AppTemplateModeIdle) || (mode == AppTemplateModeRun)) ? 1U : 0U;
}

/**
 * @brief 计算实际任务周期。
 *
 * @param period_ms 输入周期，单位 ms。
 * @return 输入非零时返回输入值，否则返回默认周期。
 */
static inline uint32_t app_template_period(uint32_t period_ms)
{
    return (period_ms == 0U) ? APP_TEMPLATE_PERIOD_DEFAULT : period_ms;
}

/**
 ******************************************************************************
 * 公共函数声明。
 ******************************************************************************
 */
/**
 * @brief 初始化模板模块并创建常驻任务。
 *
 * @param config 初始化配置；为 NULL 时使用默认配置。
 * @return 返回模块状态码。
 * @retval AppTemplateOk 初始化成功，或模块已经初始化。
 * @retval AppTemplateParamError 配置参数无效。
 * @retval AppTemplateError 任务创建或底层初始化失败。
 *
 * @pre 调用前应完成模块依赖的 BSP、驱动和算法对象初始化。
 * @post 返回 AppTemplateOk 后，模块进入可运行状态。
 *
 * @note 重复调用时应直接返回 AppTemplateOk，不重复创建任务。
 */
AppTemplateStatus app_template_init(const AppTemplateConfig *config);

/**
 * @brief 设置待应用配置参数。
 *
 * @param param0 配置参数 0。
 * @param param1 配置参数 1。
 *
 * @details
 *          该接口用于展示初始化前配置缓存的写法。真实模块应把 param0/param1
 *          替换为明确的业务配置字段。
 */
void app_template_config_set(uint32_t param0, uint32_t param1);

/**
 * @brief 设置外部请求状态。
 *
 * @param command 外部命令。
 * @param active 非零表示请求激活。
 *
 * @details
 *          周期任务可在下一次执行时读取该请求快照，并根据 active 判断是否使用。
 *
 * @par 线程安全
 *      实现应使用临界区保护请求缓存，避免任务和中断同时访问造成快照撕裂。
 */
void app_template_request_set(int16_t command, uint8_t active);

/**
 * @brief 获取模板模块诊断快照。
 *
 * @param diag 诊断快照输出指针。
 *
 * @warning diag 为 NULL 时函数必须直接返回，不允许解引用。
 *
 * @par 线程安全
 *      实现应复制完整快照，不向外暴露内部诊断对象指针。
 */
void app_template_diag_get(AppTemplateDiag *diag);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // PRINTK_TEMPLATE_DOXYGEN_H
