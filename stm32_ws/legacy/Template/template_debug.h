/**
 ******************************************************************************
 * @file    template_debug.h
 * @author  GPT
 * @version V2.0.0
 * @date    2026-05-20
 * @brief   应用层 Debug 探针模板。
 *
 * @note    新版 Debug 不再要求每个 App 准备独立封装文件；业务代码可直接
 *          include "debug.h" 并调用 DBG_F32、DBG_U32、DBG_I32、DBG_BOOL。
 * @note    本模板只保留命名建议，真实模块通常不需要再创建本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_TEMPLATE_DEBUG_H
#define PRINTK_TEMPLATE_DEBUG_H

#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 模板模块初始化状态探针名称。 */
#define APP_TEMPLATE_NAME_INIT "app.template.init"

/** @brief 模板模块运行模式探针名称。 */
#define APP_TEMPLATE_NAME_MODE "app.template.mode"

/** @brief 模板模块循环计数探针名称。 */
#define APP_TEMPLATE_NAME_LOOP "app.template.loop"

/**
 * @brief 模板模块 Debug 采样示例。
 *
 * @param init_ok 初始化状态，非 0 表示成功。
 * @param mode 当前模式。
 * @param loop_count 循环计数。
 * @param timestamp_us 采样时间戳，单位 us。
 */
static inline void app_template_debug_sample(uint8_t init_ok,
                                             uint32_t mode,
                                             uint32_t loop_count,
                                             uint64_t timestamp_us)
{
    DBG_BOOL_TS(APP_TEMPLATE_NAME_INIT, init_ok != 0U, timestamp_us);
    DBG_U32_TS(APP_TEMPLATE_NAME_MODE, mode, timestamp_us);
    DBG_U32_TS(APP_TEMPLATE_NAME_LOOP, loop_count, timestamp_us);
}

#ifdef __cplusplus
}
#endif

#endif // PRINTK_TEMPLATE_DEBUG_H
