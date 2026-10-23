/**
 ******************************************************************************
 * @file    bsp_dts.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DTS 控制器 BSP 抽象实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_dts.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief DTS 单个事件的回调绑定槽。
 */
typedef struct
{
    bsp_dts_event_callback_t callback;    /**< 事件回调，可为 NULL。 */
    void *user;                           /**< 事件回调用户上下文。 */
} bsp_dts_callback_slot_t;

/**
 * @brief DTS 控制器分发槽位。
 */
typedef struct
{
    bsp_dts_id_t id;                                        /**< BSP 层暴露给上层使用的 DTS 标识。 */
    DTS_HandleTypeDef *hdts;                                /**< CubeMX/HAL 生成的 DTS 句柄。 */
    DTS_TypeDef *instance;                                  /**< DTS 外设实例地址。 */
    bsp_dts_callback_slot_t events[BSP_DTS_EVENT_COUNT];    /**< DTS 事件回调表。 */
} bsp_dts_slot_t;

/**
 * @brief 板级 DTS 槽位表。
 *
 * 新增或删除 DTS 实例时，需要同步维护 bsp_dts_id_t 和该表。
 */
static bsp_dts_slot_t bsp_dts_slots[] = {
    {.id = BSP_DTS_ID_DTS, .hdts = &hdts, .instance = DTS},
};

#define BSP_DTS_SLOT_COUNT ((uint32_t)(sizeof(bsp_dts_slots) / sizeof(bsp_dts_slots[0])))    /**< bsp_dts_slots[] 中受 BSP 管理的 DTS 数量。 */

/**
 * @brief 根据 HAL DTS 句柄查找槽位。
 * @param hdts HAL DTS 句柄。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_dts_slot_t *slot_by_handle(DTS_HandleTypeDef *hdts)
{
    uint32_t i;

    if (hdts == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_DTS_SLOT_COUNT; i++) {
        if (bsp_dts_slots[i].hdts == hdts) {
            return &bsp_dts_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP DTS 标识查找槽位。
 * @param id BSP DTS 标识。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_dts_slot_t *slot_by_id(bsp_dts_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_DTS_SLOT_COUNT; i++) {
        if (bsp_dts_slots[i].id == id) {
            return &bsp_dts_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 DTS 外设实例查找槽位。
 * @param instance DTS 外设实例地址。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_dts_slot_t *slot_by_instance(DTS_TypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_DTS_SLOT_COUNT; i++) {
        if (bsp_dts_slots[i].instance == instance) {
            return &bsp_dts_slots[i];
        }
    }

    return NULL;
}

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */

/**
 * @brief 分发 DTS HAL 回调事件。
 * @param hdts 触发事件的 HAL DTS 句柄。
 * @param event BSP DTS 事件类型。
 */
static void dispatch_event(DTS_HandleTypeDef *hdts, bsp_dts_event_t event)
{
    bsp_dts_slot_t *slot = slot_by_handle(hdts);

    if ((slot == NULL) || (event >= BSP_DTS_EVENT_COUNT) || (slot->events[event].callback == NULL)) {
        return;
    }

    slot->events[event].callback(hdts, slot->events[event].user);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_dts_init(void)
{
    uint32_t i;
    uint32_t event;

    for (i = 0U; i < BSP_DTS_SLOT_COUNT; i++) {
        /* @diff: 新版 DTS BSP 只清空事件回调，不启动测量，也不维护温度业务状态。 */
        for (event = 0U; event < BSP_DTS_EVENT_COUNT; event++) {
            bsp_dts_slots[i].events[event].callback = NULL;
            bsp_dts_slots[i].events[event].user = NULL;
        }
    }
}

DTS_HandleTypeDef *bsp_dts_handle(bsp_dts_id_t id)
{
    bsp_dts_slot_t *slot = slot_by_id(id);

    return (slot == NULL) ? NULL : slot->hdts;
}

DTS_HandleTypeDef *bsp_dts_from_instance(DTS_TypeDef *instance)
{
    bsp_dts_slot_t *slot = slot_by_instance(instance);

    return (slot == NULL) ? NULL : slot->hdts;
}

uint8_t bsp_dts_is_supported(DTS_HandleTypeDef *hdts)
{
    return (slot_by_handle(hdts) != NULL) ? 1U : 0U;
}

HAL_StatusTypeDef bsp_dts_start(DTS_HandleTypeDef *hdts)
{
    if (slot_by_handle(hdts) == NULL) {
        return HAL_ERROR;
    }

    return HAL_DTS_Start(hdts);
}

HAL_StatusTypeDef bsp_dts_stop(DTS_HandleTypeDef *hdts)
{
    if (slot_by_handle(hdts) == NULL) {
        return HAL_ERROR;
    }

    return HAL_DTS_Stop(hdts);
}

HAL_StatusTypeDef bsp_dts_get_temperature(DTS_HandleTypeDef *hdts, int32_t *temperature)
{
    if ((slot_by_handle(hdts) == NULL) || (temperature == NULL)) {
        return HAL_ERROR;
    }

    return HAL_DTS_GetTemperature(hdts, temperature);
}

HAL_StatusTypeDef bsp_dts_start_it(DTS_HandleTypeDef *hdts)
{
    if (slot_by_handle(hdts) == NULL) {
        return HAL_ERROR;
    }

    return HAL_DTS_Start_IT(hdts);
}

HAL_StatusTypeDef bsp_dts_stop_it(DTS_HandleTypeDef *hdts)
{
    if (slot_by_handle(hdts) == NULL) {
        return HAL_ERROR;
    }

    return HAL_DTS_Stop_IT(hdts);
}

void bsp_dts_irq_handler(DTS_HandleTypeDef *hdts)
{
    if (slot_by_handle(hdts) == NULL) {
        return;
    }

    HAL_DTS_IRQHandler(hdts);
}

HAL_DTS_StateTypeDef bsp_dts_get_state(DTS_HandleTypeDef *hdts)
{
    if (slot_by_handle(hdts) == NULL) {
        return HAL_DTS_STATE_RESET;
    }

    return HAL_DTS_GetState(hdts);
}

HAL_StatusTypeDef bsp_dts_on(DTS_HandleTypeDef *hdts,
                             bsp_dts_event_t event,
                             bsp_dts_event_callback_t callback,
                             void *user)
{
    bsp_dts_slot_t *slot = slot_by_handle(hdts);

    if ((slot == NULL) || (event >= BSP_DTS_EVENT_COUNT)) {
        return HAL_ERROR;
    }

    slot->events[event].callback = callback;
    slot->events[event].user = user;
    return HAL_OK;
}

void HAL_DTS_EndCallback(DTS_HandleTypeDef *hdts)
{
    dispatch_event(hdts, BSP_DTS_EVENT_END);
}

void HAL_DTS_LowCallback(DTS_HandleTypeDef *hdts)
{
    dispatch_event(hdts, BSP_DTS_EVENT_LOW);
}

void HAL_DTS_HighCallback(DTS_HandleTypeDef *hdts)
{
    dispatch_event(hdts, BSP_DTS_EVENT_HIGH);
}

void HAL_DTS_AsyncEndCallback(DTS_HandleTypeDef *hdts)
{
    dispatch_event(hdts, BSP_DTS_EVENT_ASYNC_END);
}

void HAL_DTS_AsyncLowCallback(DTS_HandleTypeDef *hdts)
{
    dispatch_event(hdts, BSP_DTS_EVENT_ASYNC_LOW);
}

void HAL_DTS_AsyncHighCallback(DTS_HandleTypeDef *hdts)
{
    dispatch_event(hdts, BSP_DTS_EVENT_ASYNC_HIGH);
}
