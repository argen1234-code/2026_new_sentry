/**
 ******************************************************************************
 * @file    bsp_tim.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   TIM BSP 通用接口。
 *
 * @note    本文件只封装 Timer 的通用硬件能力：基本计数、更新中断、
 *          PWM 启停、比较值读写和周期回调分发。
 * @note    PWM 是 Timer 输出比较能力的一种使用方式；具体设备语义
 *          应放在设备驱动或应用层。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_TIM_H
#define PRINTK_BSP_TIM_H

#include "tim.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */

/**
 * @brief 板上可由 BSP 管理的 TIM 标识。
 *
 * 新增或删除 TIM 实例时，需要同步维护本枚举和 bsp_tim.c 中的 bsp_tim_slots[]。
 */
typedef enum
{
    BSP_TIM_ID_TIM1 = 0,
    BSP_TIM_ID_TIM2,
    BSP_TIM_ID_TIM3,
    BSP_TIM_ID_TIM12,
    BSP_TIM_ID_TIM24,
    BSP_TIM_ID_COUNT
} bsp_tim_id_t;

/**
 * @brief TIM 启停模式。
 */
typedef enum
{
    BSP_TIM_MODE_BASE = 0,    /**< 基本计数模式。 */
    BSP_TIM_MODE_BASE_IT      /**< 基本计数 + update 中断模式。 */
} bsp_tim_mode_t;

/**
 * @brief TIM 事件类型。
 */
typedef enum
{
    BSP_TIM_EVENT_PERIOD_ELAPSED = 0,
    BSP_TIM_EVENT_COUNT
} bsp_tim_event_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief TIM 事件回调函数类型。
 *
 * @param htim 触发事件的 TIM 句柄。
 * @param user 注册回调时传入的用户上下文。
 */
typedef void (*bsp_tim_event_callback_t)(TIM_HandleTypeDef *htim, void *user);

/**
 * @brief 初始化 TIM BSP 回调表。
 */
void bsp_tim_init(void);

/**
 * @brief 根据 BSP TIM 标识获取 TIM 句柄。
 *
 * @param id BSP TIM 标识。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
TIM_HandleTypeDef *bsp_tim_handle(bsp_tim_id_t id);

/**
 * @brief 根据外设实例地址获取 TIM 句柄。
 *
 * @param instance TIM 外设实例，例如 `TIM1`、`TIM24`。
 * @return 匹配成功返回对应句柄，否则返回 `NULL`。
 */
TIM_HandleTypeDef *bsp_tim_from_instance(TIM_TypeDef *instance);

/**
 * @brief 判断 TIM 句柄是否由本 BSP 管理。
 *
 * @param htim 待检查的 TIM 句柄。
 * @return 受管理返回 1，否则返回 0。
 */
uint8_t bsp_tim_is_supported(TIM_HandleTypeDef *htim);

/**
 * @brief 启动 TIM。
 *
 * @param htim 目标 TIM 句柄。
 * @param mode TIM 启动模式。
 * @return 返回 HAL TIM 状态码。
 */
HAL_StatusTypeDef bsp_tim_start(TIM_HandleTypeDef *htim, bsp_tim_mode_t mode);

/**
 * @brief 停止 TIM。
 *
 * @param htim 目标 TIM 句柄。
 * @param mode TIM 停止模式。
 * @return 返回 HAL TIM 状态码。
 */
HAL_StatusTypeDef bsp_tim_stop(TIM_HandleTypeDef *htim, bsp_tim_mode_t mode);

/**
 * @brief 获取 TIM 当前计数值。
 *
 * @param htim 目标 TIM 句柄。
 * @return 当前计数值；入参非法时返回 0。
 */
uint32_t bsp_tim_get_counter(TIM_HandleTypeDef *htim);

/**
 * @brief 设置 TIM 当前计数值。
 *
 * @param htim 目标 TIM 句柄。
 * @param value 需要写入的计数值。
 */
void bsp_tim_set_counter(TIM_HandleTypeDef *htim, uint32_t value);

/**
 * @brief 获取 TIM 自动重装载值 ARR。
 *
 * @param htim 目标 TIM 句柄。
 * @return 当前 ARR 值；入参非法时返回 0。
 */
uint32_t bsp_tim_get_autoreload(TIM_HandleTypeDef *htim);

/**
 * @brief 设置 TIM 自动重装载值 ARR。
 *
 * @param htim 目标 TIM 句柄。
 * @param value 需要写入的 ARR 值。
 */
void bsp_tim_set_autoreload(TIM_HandleTypeDef *htim, uint32_t value);

/**
 * @brief 获取指定 TIM 通道比较值。
 *
 * @param htim 目标 TIM 句柄。
 * @param channel HAL 定义的 TIM 通道，例如 `TIM_CHANNEL_1`。
 * @return 当前比较值；入参非法时返回 0。
 */
uint32_t bsp_tim_get_compare(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 设置指定 TIM 通道比较值。
 *
 * 当 value 超过当前 ARR 时，该接口会裁剪到 ARR。
 *
 * @param htim 目标 TIM 句柄。
 * @param channel HAL 定义的 TIM 通道，例如 `TIM_CHANNEL_1`。
 * @param value 需要写入的比较值。
 */
void bsp_tim_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t value);

/**
 * @brief 启动指定 TIM 通道 PWM 输出。
 *
 * @param htim 目标 TIM 句柄。
 * @param channel HAL 定义的 TIM 通道，例如 `TIM_CHANNEL_1`。
 * @return 返回 HAL TIM 状态码。
 */
HAL_StatusTypeDef bsp_tim_start_pwm(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 停止指定 TIM 通道 PWM 输出。
 *
 * @param htim 目标 TIM 句柄。
 * @param channel HAL 定义的 TIM 通道，例如 `TIM_CHANNEL_1`。
 * @return 返回 HAL TIM 状态码。
 */
HAL_StatusTypeDef bsp_tim_stop_pwm(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 绑定 TIM 事件回调。
 *
 * @param htim 目标 TIM 句柄。
 * @param event TIM 事件类型。
 * @param callback 回调函数，传入 `NULL` 表示取消绑定。
 * @param user 用户上下文。
 * @return 返回 HAL TIM 状态码。
 */
HAL_StatusTypeDef bsp_tim_on(TIM_HandleTypeDef *htim,
                             bsp_tim_event_t event,
                             bsp_tim_event_callback_t callback,
                             void *user);

/**
 * @brief 分发 HAL TIM 周期更新回调。
 *
 * 应在工程中的 HAL_TIM_PeriodElapsedCallback() 内调用。
 *
 * @param htim 触发周期更新事件的 TIM 句柄。
 */
void bsp_tim_dispatch_period_elapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_TIM_H
