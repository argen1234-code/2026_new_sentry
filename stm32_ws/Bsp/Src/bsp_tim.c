/**
 ******************************************************************************
 * @file    bsp_tim.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   TIM BSP 通用实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_tim.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief TIM 事件绑定槽。
 */
typedef struct
{
    bsp_tim_event_callback_t callback;    /**< 事件回调，可为 NULL。 */
    void *user;                           /**< 用户上下文。 */
} bsp_tim_callback_slot_t;

/**
 * @brief TIM 分发槽位。
 *
 * 每个槽位对应一个板级 TIM 句柄及其事件回调。
 */
typedef struct
{
    bsp_tim_id_t id;                                        /**< BSP 层暴露给上层使用的 TIM 标识。 */
    TIM_HandleTypeDef *htim;                                /**< CubeMX/HAL 生成的 TIM 句柄。 */
    TIM_TypeDef *instance;                                  /**< TIM 外设实例地址，例如 TIM1、TIM24。 */
    bsp_tim_callback_slot_t events[BSP_TIM_EVENT_COUNT];    /**< TIM 事件绑定表。 */
} bsp_tim_slot_t;

/**
 * @brief 板级 TIM 回调注册表。
 *
 * 新增或删除 TIM 时，本表必须与 bsp_tim_id_t 和 CubeMX 生成的
 * htimx/TIMx 实例保持一致。
 */
static bsp_tim_slot_t bsp_tim_slots[] = {
    {.id = BSP_TIM_ID_TIM1, .htim = &htim1, .instance = TIM1},
    {.id = BSP_TIM_ID_TIM2, .htim = &htim2, .instance = TIM2},
    {.id = BSP_TIM_ID_TIM3, .htim = &htim3, .instance = TIM3},
    {.id = BSP_TIM_ID_TIM12, .htim = &htim12, .instance = TIM12},
    {.id = BSP_TIM_ID_TIM24, .htim = &htim24, .instance = TIM24},
};

#define BSP_TIM_SLOT_COUNT ((uint32_t)(sizeof(bsp_tim_slots) / sizeof(bsp_tim_slots[0])))    /**< bsp_tim_slots[] 中受 BSP 管理的 TIM 数量。 */

/**
 * @brief 根据 TIM 句柄查找回调槽位。
 *
 * @param htim 目标 TIM 句柄。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_tim_slot_t *slot_by_handle(TIM_HandleTypeDef *htim)
{
    uint32_t i;

    if (htim == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_TIM_SLOT_COUNT; i++) {
        if (bsp_tim_slots[i].htim == htim) {
            return &bsp_tim_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP TIM 标识查找回调槽位。
 *
 * @param id BSP TIM 标识。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_tim_slot_t *slot_by_id(bsp_tim_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_TIM_SLOT_COUNT; i++) {
        if (bsp_tim_slots[i].id == id) {
            return &bsp_tim_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据外设实例查找回调槽位。
 *
 * @param instance TIM 外设实例。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_tim_slot_t *slot_by_instance(TIM_TypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_TIM_SLOT_COUNT; i++) {
        if (bsp_tim_slots[i].instance == instance) {
            return &bsp_tim_slots[i];
        }
    }

    return NULL;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_tim_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_TIM_SLOT_COUNT; i++) {
        bsp_tim_slots[i].events[BSP_TIM_EVENT_PERIOD_ELAPSED].callback = NULL;
        bsp_tim_slots[i].events[BSP_TIM_EVENT_PERIOD_ELAPSED].user = NULL;
    }
}

TIM_HandleTypeDef *bsp_tim_handle(bsp_tim_id_t id)
{
    bsp_tim_slot_t *slot = slot_by_id(id);

    if (slot == NULL) {
        return NULL;
    }

    return slot->htim;
}

TIM_HandleTypeDef *bsp_tim_from_instance(TIM_TypeDef *instance)
{
    bsp_tim_slot_t *slot = slot_by_instance(instance);

    if (slot == NULL) {
        return NULL;
    }

    return slot->htim;
}

uint8_t bsp_tim_is_supported(TIM_HandleTypeDef *htim)
{
    return (slot_by_handle(htim) != NULL) ? 1U : 0U;
}

HAL_StatusTypeDef bsp_tim_start(TIM_HandleTypeDef *htim, bsp_tim_mode_t mode)
{
    if (slot_by_handle(htim) == NULL) {
        return HAL_ERROR;
    }

    switch (mode) {
    case BSP_TIM_MODE_BASE:
        return HAL_TIM_Base_Start(htim);

    case BSP_TIM_MODE_BASE_IT:
        return HAL_TIM_Base_Start_IT(htim);

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef bsp_tim_stop(TIM_HandleTypeDef *htim, bsp_tim_mode_t mode)
{
    if (slot_by_handle(htim) == NULL) {
        return HAL_ERROR;
    }

    switch (mode) {
    case BSP_TIM_MODE_BASE:
        return HAL_TIM_Base_Stop(htim);

    case BSP_TIM_MODE_BASE_IT:
        return HAL_TIM_Base_Stop_IT(htim);

    default:
        return HAL_ERROR;
    }
}

uint32_t bsp_tim_get_counter(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return 0U;
    }

    return __HAL_TIM_GET_COUNTER(htim);
}

void bsp_tim_set_counter(TIM_HandleTypeDef *htim, uint32_t value)
{
    if (htim == NULL) {
        return;
    }

    __HAL_TIM_SET_COUNTER(htim, value);
}

uint32_t bsp_tim_get_autoreload(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return 0U;
    }

    return __HAL_TIM_GET_AUTORELOAD(htim);
}

void bsp_tim_set_autoreload(TIM_HandleTypeDef *htim, uint32_t value)
{
    if (htim == NULL) {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(htim, value);
}

uint32_t bsp_tim_get_compare(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == NULL) {
        return 0U;
    }

    return __HAL_TIM_GET_COMPARE(htim, channel);
}

void bsp_tim_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t value)
{
    uint32_t autoreload;

    if (htim == NULL) {
        return;
    }

    autoreload = __HAL_TIM_GET_AUTORELOAD(htim);
    if (value > autoreload) {
        /* @diff: 旧版 bsp_PWM.c 手写 CCR1~CCR4；新版统一收敛到 TIM compare setter 并继续保留 ARR 裁剪。 */
        value = autoreload;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, value);
}

HAL_StatusTypeDef bsp_tim_start_pwm(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == NULL) {
        return HAL_ERROR;
    }

    return HAL_TIM_PWM_Start(htim, channel);
}

HAL_StatusTypeDef bsp_tim_stop_pwm(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == NULL) {
        return HAL_ERROR;
    }

    return HAL_TIM_PWM_Stop(htim, channel);
}

HAL_StatusTypeDef bsp_tim_on(TIM_HandleTypeDef *htim,
                             bsp_tim_event_t event,
                             bsp_tim_event_callback_t callback,
                             void *user)
{
    bsp_tim_slot_t *slot = slot_by_handle(htim);

    if ((slot == NULL) || (event >= BSP_TIM_EVENT_COUNT)) {
        return HAL_ERROR;
    }

    slot->events[event].callback = callback;
    slot->events[event].user = user;
    return HAL_OK;
}

void bsp_tim_dispatch_period_elapsed(TIM_HandleTypeDef *htim)
{
    bsp_tim_slot_t *slot = slot_by_handle(htim);

    if ((slot == NULL) || (slot->events[BSP_TIM_EVENT_PERIOD_ELAPSED].callback == NULL)) {
        return;
    }

    slot->events[BSP_TIM_EVENT_PERIOD_ELAPSED].callback(
        htim,
        slot->events[BSP_TIM_EVENT_PERIOD_ELAPSED].user);
}
