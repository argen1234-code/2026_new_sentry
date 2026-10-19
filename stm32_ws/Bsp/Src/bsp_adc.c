/**
 ******************************************************************************
 * @file    bsp_adc.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   ADC 控制器 BSP 抽象实现。
 *
 * @note    本文件只维护 ADC 句柄表、HAL ADC 操作包装和 HAL ADC
 *          回调分发。
 * @note    与旧版业务混合写法相比，新版不在 BSP 中解释采样值含义，
 *          也不维护业务状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_adc.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置 ADC BSP 的私有槽位表和回调绑定，不解释采样值的业务含义。
 * ========================================================================== */

/**
 * @brief ADC 事件绑定槽。
 */
typedef struct
{
    bsp_adc_event_callback_t callback;    /**< 事件回调，可为 NULL。 */
    void *user;                           /**< 事件回调用户上下文。 */
} bsp_adc_callback_slot_t;

/**
 * @brief ADC 分发槽位。
 *
 * 每个槽位对应一个板级 ADC 句柄及其事件回调注册信息。
 */
typedef struct
{
    bsp_adc_id_t id;                                        /**< BSP 层暴露给上层使用的 ADC 标识。 */
    ADC_HandleTypeDef *hadc;                                /**< CubeMX/HAL 生成的 ADC 句柄。 */
    ADC_TypeDef *instance;                                  /**< ADC 外设实例地址，例如 ADC1。 */
    bsp_adc_callback_slot_t events[BSP_ADC_EVENT_COUNT];    /**< ADC 事件绑定表。 */
} bsp_adc_slot_t;

/**
 * @brief 板级 ADC 回调注册表。
 *
 * 新增或删除 ADC 时，本表必须与 bsp_adc_id_t 和 CubeMX 生成的 hadcx/ADCx 实例保持一致。
 */
static bsp_adc_slot_t bsp_adc_slots[] = {
    {.id = BSP_ADC_ID_ADC1, .hadc = &hadc1, .instance = ADC1},
    {.id = BSP_ADC_ID_ADC3, .hadc = &hadc3, .instance = ADC3},
};

#define BSP_ADC_SLOT_COUNT ((uint32_t)(sizeof(bsp_adc_slots) / sizeof(bsp_adc_slots[0])))    /**< bsp_adc_slots[] 中受 BSP 管理的 ADC 数量。 */

/**
 * @brief 根据 ADC 句柄查找槽位。
 *
 * @param hadc 目标 ADC 句柄。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_adc_slot_t *slot_by_handle(ADC_HandleTypeDef *hadc)
{
    uint32_t i;

    if (hadc == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_ADC_SLOT_COUNT; i++) {
        if (bsp_adc_slots[i].hadc == hadc) {
            return &bsp_adc_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP ADC 标识查找槽位。
 *
 * @param id BSP ADC 标识。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_adc_slot_t *slot_by_id(bsp_adc_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_ADC_SLOT_COUNT; i++) {
        if (bsp_adc_slots[i].id == id) {
            return &bsp_adc_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 ADC 外设实例查找槽位。
 *
 * @param instance ADC 外设实例。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_adc_slot_t *slot_by_instance(ADC_TypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_ADC_SLOT_COUNT; i++) {
        if (bsp_adc_slots[i].instance == instance) {
            return &bsp_adc_slots[i];
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
 * @brief 清空单个 ADC 槽位的事件绑定。
 *
 * @param slot ADC 分发槽位。
 */
static void clear_callbacks(bsp_adc_slot_t *slot)
{
    uint32_t i;

    if (slot == NULL) {
        return;
    }

    for (i = 0U; i < BSP_ADC_EVENT_COUNT; i++) {
        slot->events[i].callback = NULL;
        slot->events[i].user = NULL;
    }
}

/**
 * @brief 分发 ADC 普通事件。
 *
 * @param hadc 触发事件的 ADC 句柄。
 * @param event ADC 事件类型。
 */
static void dispatch_event(ADC_HandleTypeDef *hadc, bsp_adc_event_t event)
{
    bsp_adc_slot_t *slot = slot_by_handle(hadc);

    if ((slot == NULL) || (event >= BSP_ADC_EVENT_COUNT) || (slot->events[event].callback == NULL)) {
        return;
    }

    slot->events[event].callback(hadc, slot->events[event].user);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_adc_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_ADC_SLOT_COUNT; i++) {
        /* @diff: 新版 ADC BSP 只清空事件回调，不启动转换，也不保存采样业务状态。 */
        clear_callbacks(&bsp_adc_slots[i]);
    }
}

ADC_HandleTypeDef *bsp_adc_handle(bsp_adc_id_t id)
{
    bsp_adc_slot_t *slot = slot_by_id(id);

    if (slot == NULL) {
        return NULL;
    }

    return slot->hadc;
}

ADC_HandleTypeDef *bsp_adc_from_instance(ADC_TypeDef *instance)
{
    bsp_adc_slot_t *slot = slot_by_instance(instance);

    if (slot == NULL) {
        return NULL;
    }

    return slot->hadc;
}

uint8_t bsp_adc_is_supported(ADC_HandleTypeDef *hadc)
{
    return (slot_by_handle(hadc) != NULL) ? 1U : 0U;
}

HAL_StatusTypeDef bsp_adc_calibration_start(ADC_HandleTypeDef *hadc,
                                            uint32_t calibration_mode,
                                            uint32_t single_diff)
{
    if (slot_by_handle(hadc) == NULL) {
        return HAL_ERROR;
    }

    return HAL_ADCEx_Calibration_Start(hadc, calibration_mode, single_diff);
}

HAL_StatusTypeDef bsp_adc_start(ADC_HandleTypeDef *hadc, const bsp_adc_start_t *start)
{
    if ((slot_by_handle(hadc) == NULL) || (start == NULL)) {
        return HAL_ERROR;
    }

    switch (start->mode) {
    case BSP_ADC_MODE_POLL:
        return HAL_ADC_Start(hadc);

    case BSP_ADC_MODE_IT:
        return HAL_ADC_Start_IT(hadc);

    case BSP_ADC_MODE_DMA:
        if ((start->data == NULL) || (start->length == 0U)) {
            return HAL_ERROR;
        }
        return HAL_ADC_Start_DMA(hadc, start->data, start->length);

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef bsp_adc_stop(ADC_HandleTypeDef *hadc, bsp_adc_mode_t mode)
{
    if (slot_by_handle(hadc) == NULL) {
        return HAL_ERROR;
    }

    switch (mode) {
    case BSP_ADC_MODE_POLL:
        return HAL_ADC_Stop(hadc);

    case BSP_ADC_MODE_IT:
        return HAL_ADC_Stop_IT(hadc);

    case BSP_ADC_MODE_DMA:
        return HAL_ADC_Stop_DMA(hadc);

    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef bsp_adc_poll_for_conversion(ADC_HandleTypeDef *hadc, uint32_t timeout)
{
    if (slot_by_handle(hadc) == NULL) {
        return HAL_ERROR;
    }

    return HAL_ADC_PollForConversion(hadc, timeout);
}

uint32_t bsp_adc_get_value(ADC_HandleTypeDef *hadc)
{
    if (slot_by_handle(hadc) == NULL) {
        return 0U;
    }

    return HAL_ADC_GetValue(hadc);
}

HAL_StatusTypeDef bsp_adc_config_channel(ADC_HandleTypeDef *hadc,
                                         const ADC_ChannelConfTypeDef *config)
{
    if ((slot_by_handle(hadc) == NULL) || (config == NULL)) {
        return HAL_ERROR;
    }

    return HAL_ADC_ConfigChannel(hadc, (ADC_ChannelConfTypeDef *)config);
}

HAL_StatusTypeDef bsp_adc_on(ADC_HandleTypeDef *hadc,
                             bsp_adc_event_t event,
                             bsp_adc_event_callback_t callback,
                             void *user)
{
    bsp_adc_slot_t *slot = slot_by_handle(hadc);

    if ((slot == NULL) || (event >= BSP_ADC_EVENT_COUNT)) {
        return HAL_ERROR;
    }

    slot->events[event].callback = callback;
    slot->events[event].user = user;
    return HAL_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    dispatch_event(hadc, BSP_ADC_EVENT_CONV_CPLT);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    dispatch_event(hadc, BSP_ADC_EVENT_CONV_HALF);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    dispatch_event(hadc, BSP_ADC_EVENT_ERROR);
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    dispatch_event(hadc, BSP_ADC_EVENT_LEVEL_OUT_OF_WINDOW);
}

void HAL_ADCEx_EndOfSamplingCallback(ADC_HandleTypeDef *hadc)
{
    dispatch_event(hadc, BSP_ADC_EVENT_END_OF_SAMPLING);
}
