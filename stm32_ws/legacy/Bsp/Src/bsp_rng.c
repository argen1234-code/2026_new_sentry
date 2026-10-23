/**
 ******************************************************************************
 * @file    bsp_rng.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   RNG 控制器 BSP 抽象实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_rng.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief RNG 分发槽位。
 */
typedef struct
{
    bsp_rng_id_t id;                            /**< BSP 层暴露给上层使用的 RNG 标识。 */
    RNG_HandleTypeDef *hrng;                    /**< CubeMX/HAL 生成的 RNG 句柄。 */
    RNG_TypeDef *instance;                      /**< RNG 外设实例地址。 */
    bsp_rng_ready_callback_t ready_callback;    /**< 随机数就绪回调，可为 NULL。 */
    void *ready_user;                           /**< 随机数就绪用户上下文。 */
    bsp_rng_event_callback_t error_callback;    /**< 错误回调，可为 NULL。 */
    void *error_user;                           /**< 错误用户上下文。 */
} bsp_rng_slot_t;

/**
 * @brief 板级 RNG 回调注册表。
 */
static bsp_rng_slot_t bsp_rng_slots[] = {
    {.id = BSP_RNG_ID_RNG, .hrng = &hrng, .instance = RNG},
};

#define BSP_RNG_SLOT_COUNT ((uint32_t)(sizeof(bsp_rng_slots) / sizeof(bsp_rng_slots[0])))    /**< bsp_rng_slots[] 中受 BSP 管理的 RNG 数量。 */

/**
 * @brief 根据 HAL RNG 句柄查找槽位。
 * @param hrng HAL RNG 句柄。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_rng_slot_t *slot_by_handle(RNG_HandleTypeDef *hrng)
{
    uint32_t i;

    if (hrng == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_RNG_SLOT_COUNT; i++) {
        if (bsp_rng_slots[i].hrng == hrng) {
            return &bsp_rng_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP RNG 标识查找槽位。
 * @param id BSP RNG 标识。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_rng_slot_t *slot_by_id(bsp_rng_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_RNG_SLOT_COUNT; i++) {
        if (bsp_rng_slots[i].id == id) {
            return &bsp_rng_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 RNG 外设实例查找槽位。
 * @param instance RNG 外设实例地址。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_rng_slot_t *slot_by_instance(RNG_TypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_RNG_SLOT_COUNT; i++) {
        if (bsp_rng_slots[i].instance == instance) {
            return &bsp_rng_slots[i];
        }
    }

    return NULL;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_rng_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_RNG_SLOT_COUNT; i++) {
        /* @diff: CubeMX 仍负责 RNG 硬件初始化；新版 RNG BSP 只清空事件回调。 */
        bsp_rng_slots[i].ready_callback = NULL;
        bsp_rng_slots[i].ready_user = NULL;
        bsp_rng_slots[i].error_callback = NULL;
        bsp_rng_slots[i].error_user = NULL;
    }
}

RNG_HandleTypeDef *bsp_rng_handle(bsp_rng_id_t id)
{
    bsp_rng_slot_t *slot = slot_by_id(id);

    return (slot == NULL) ? NULL : slot->hrng;
}

RNG_HandleTypeDef *bsp_rng_from_instance(RNG_TypeDef *instance)
{
    bsp_rng_slot_t *slot = slot_by_instance(instance);

    return (slot == NULL) ? NULL : slot->hrng;
}

uint8_t bsp_rng_is_supported(RNG_HandleTypeDef *hrng)
{
    return (slot_by_handle(hrng) != NULL) ? 1U : 0U;
}

HAL_StatusTypeDef bsp_rng_generate(RNG_HandleTypeDef *hrng, uint32_t *random32bit)
{
    if ((slot_by_handle(hrng) == NULL) || (random32bit == NULL)) {
        return HAL_ERROR;
    }

    return HAL_RNG_GenerateRandomNumber(hrng, random32bit);
}

HAL_StatusTypeDef bsp_rng_generate_it(RNG_HandleTypeDef *hrng)
{
    if (slot_by_handle(hrng) == NULL) {
        return HAL_ERROR;
    }

    return HAL_RNG_GenerateRandomNumber_IT(hrng);
}

uint32_t bsp_rng_read_last(RNG_HandleTypeDef *hrng)
{
    if (slot_by_handle(hrng) == NULL) {
        return 0U;
    }

    return HAL_RNG_ReadLastRandomNumber(hrng);
}

void bsp_rng_irq_handler(RNG_HandleTypeDef *hrng)
{
    if (slot_by_handle(hrng) == NULL) {
        return;
    }

    HAL_RNG_IRQHandler(hrng);
}

HAL_RNG_StateTypeDef bsp_rng_get_state(RNG_HandleTypeDef *hrng)
{
    if (slot_by_handle(hrng) == NULL) {
        return HAL_RNG_STATE_RESET;
    }

    return HAL_RNG_GetState(hrng);
}

uint32_t bsp_rng_get_error(RNG_HandleTypeDef *hrng)
{
    if (slot_by_handle(hrng) == NULL) {
        return HAL_RNG_ERROR_NONE;
    }

    return HAL_RNG_GetError(hrng);
}

HAL_StatusTypeDef bsp_rng_recover_seed_error(RNG_HandleTypeDef *hrng)
{
    if (slot_by_handle(hrng) == NULL) {
        return HAL_ERROR;
    }

    return HAL_RNGEx_RecoverSeedError(hrng);
}

HAL_StatusTypeDef bsp_rng_set_config(RNG_HandleTypeDef *hrng, const RNG_ConfigTypeDef *config)
{
    if ((slot_by_handle(hrng) == NULL) || (config == NULL)) {
        return HAL_ERROR;
    }

    return HAL_RNGEx_SetConfig(hrng, config);
}

HAL_StatusTypeDef bsp_rng_get_config(RNG_HandleTypeDef *hrng, RNG_ConfigTypeDef *config)
{
    if ((slot_by_handle(hrng) == NULL) || (config == NULL)) {
        return HAL_ERROR;
    }

    return HAL_RNGEx_GetConfig(hrng, config);
}

HAL_StatusTypeDef bsp_rng_lock_config(RNG_HandleTypeDef *hrng)
{
    if (slot_by_handle(hrng) == NULL) {
        return HAL_ERROR;
    }

    return HAL_RNGEx_LockConfig(hrng);
}

HAL_StatusTypeDef bsp_rng_on(RNG_HandleTypeDef *hrng, bsp_rng_event_t event, void *callback, void *user)
{
    bsp_rng_slot_t *slot = slot_by_handle(hrng);

    if ((slot == NULL) || (event >= BSP_RNG_EVENT_COUNT)) {
        return HAL_ERROR;
    }

    if (event == BSP_RNG_EVENT_READY) {
        slot->ready_callback = (bsp_rng_ready_callback_t)callback;
        slot->ready_user = user;
    } else {
        slot->error_callback = (bsp_rng_event_callback_t)callback;
        slot->error_user = user;
    }

    return HAL_OK;
}

void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng)
{
    bsp_rng_slot_t *slot = slot_by_handle(hrng);

    if ((slot == NULL) || (slot->error_callback == NULL)) {
        return;
    }

    slot->error_callback(hrng, slot->error_user);
}

void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit)
{
    bsp_rng_slot_t *slot = slot_by_handle(hrng);

    if ((slot == NULL) || (slot->ready_callback == NULL)) {
        return;
    }

    slot->ready_callback(hrng, random32bit, slot->ready_user);
}
