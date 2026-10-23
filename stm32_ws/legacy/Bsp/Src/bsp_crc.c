/**
 ******************************************************************************
 * @file    bsp_crc.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   CRC 控制器 BSP 抽象实现。
 *
 * @note    本文件只维护 CRC 句柄表、HAL CRC 操作包装和状态读取。
 * @note    与业务混合写法相比，新版不在 BSP 中规定某个协议的多项式、帧格式、
 *          字节序兼容或校验失败处理策略。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_crc.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief CRC 控制器槽位。
 *
 * 每个槽位对应一个板级 CRC 句柄。
 */
typedef struct
{
    bsp_crc_id_t id;            /**< BSP 层暴露给上层使用的 CRC 标识。 */
    CRC_HandleTypeDef *hcrc;    /**< CubeMX/HAL 生成的 CRC 句柄。 */
    CRC_TypeDef *instance;      /**< CRC 外设实例地址，例如 CRC。 */
} bsp_crc_slot_t;

/**
 * @brief 板级 CRC 句柄注册表。
 *
 * 新增或删除 CRC 时，本表必须与 bsp_crc_id_t 和 CubeMX 生成的 hcrcx/CRCx 实例保持一致。
 */
static bsp_crc_slot_t bsp_crc_slots[] = {
    {.id = BSP_CRC_ID_CRC, .hcrc = &hcrc, .instance = CRC},
};

#define BSP_CRC_SLOT_COUNT ((uint32_t)(sizeof(bsp_crc_slots) / sizeof(bsp_crc_slots[0])))    /**< bsp_crc_slots[] 中受 BSP 管理的 CRC 数量。 */

/**
 * @brief 根据 CRC 句柄查找槽位。
 *
 * @param hcrc 目标 CRC 句柄。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_crc_slot_t *slot_by_handle(CRC_HandleTypeDef *hcrc)
{
    uint32_t i;

    if (hcrc == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_CRC_SLOT_COUNT; i++) {
        if (bsp_crc_slots[i].hcrc == hcrc) {
            return &bsp_crc_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP CRC 标识查找槽位。
 *
 * @param id BSP CRC 标识。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_crc_slot_t *slot_by_id(bsp_crc_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_CRC_SLOT_COUNT; i++) {
        if (bsp_crc_slots[i].id == id) {
            return &bsp_crc_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据外设实例查找槽位。
 *
 * @param instance CRC 外设实例。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_crc_slot_t *slot_by_instance(CRC_TypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_CRC_SLOT_COUNT; i++) {
        if (bsp_crc_slots[i].instance == instance) {
            return &bsp_crc_slots[i];
        }
    }

    return NULL;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_crc_init(void)
{
    /* @diff: CubeMX 仍负责 CRC 硬件初始化；新版 CRC BSP 没有内部业务状态需要清空。 */
}

CRC_HandleTypeDef *bsp_crc_handle(bsp_crc_id_t id)
{
    bsp_crc_slot_t *slot = slot_by_id(id);

    if (slot == NULL) {
        return NULL;
    }

    return slot->hcrc;
}

CRC_HandleTypeDef *bsp_crc_from_instance(CRC_TypeDef *instance)
{
    bsp_crc_slot_t *slot = slot_by_instance(instance);

    if (slot == NULL) {
        return NULL;
    }

    return slot->hcrc;
}

uint8_t bsp_crc_is_supported(CRC_HandleTypeDef *hcrc)
{
    return (slot_by_handle(hcrc) != NULL) ? 1U : 0U;
}

uint32_t bsp_crc_calculate(CRC_HandleTypeDef *hcrc, const uint32_t *data, uint32_t length)
{
    if ((slot_by_handle(hcrc) == NULL) || (data == NULL) || (length == 0U)) {
        return 0U;
    }

    return HAL_CRC_Calculate(hcrc, (uint32_t *)data, length);
}

uint32_t bsp_crc_accumulate(CRC_HandleTypeDef *hcrc, const uint32_t *data, uint32_t length)
{
    if ((slot_by_handle(hcrc) == NULL) || (data == NULL) || (length == 0U)) {
        return 0U;
    }

    return HAL_CRC_Accumulate(hcrc, (uint32_t *)data, length);
}

HAL_StatusTypeDef bsp_crc_reset(CRC_HandleTypeDef *hcrc)
{
    if (slot_by_handle(hcrc) == NULL) {
        return HAL_ERROR;
    }

    __HAL_CRC_DR_RESET(hcrc);
    return HAL_OK;
}

HAL_StatusTypeDef bsp_crc_set_initial_value(CRC_HandleTypeDef *hcrc, uint32_t init_value)
{
    if (slot_by_handle(hcrc) == NULL) {
        return HAL_ERROR;
    }

    __HAL_CRC_INITIALCRCVALUE_CONFIG(hcrc, init_value);
    return HAL_OK;
}

HAL_StatusTypeDef bsp_crc_set_independent_data(CRC_HandleTypeDef *hcrc, uint8_t value)
{
    if (slot_by_handle(hcrc) == NULL) {
        return HAL_ERROR;
    }

    __HAL_CRC_SET_IDR(hcrc, value);
    return HAL_OK;
}

uint8_t bsp_crc_get_independent_data(CRC_HandleTypeDef *hcrc)
{
    if (slot_by_handle(hcrc) == NULL) {
        return 0U;
    }

    return (uint8_t)__HAL_CRC_GET_IDR(hcrc);
}

HAL_StatusTypeDef bsp_crc_set_polynomial(CRC_HandleTypeDef *hcrc,
                                         uint32_t polynomial,
                                         uint32_t polynomial_length)
{
    if (slot_by_handle(hcrc) == NULL) {
        return HAL_ERROR;
    }

    return HAL_CRCEx_Polynomial_Set(hcrc, polynomial, polynomial_length);
}

HAL_StatusTypeDef bsp_crc_set_input_reverse(CRC_HandleTypeDef *hcrc, uint32_t reverse_mode)
{
    if (slot_by_handle(hcrc) == NULL) {
        return HAL_ERROR;
    }

    return HAL_CRCEx_Input_Data_Reverse(hcrc, reverse_mode);
}

HAL_StatusTypeDef bsp_crc_set_output_reverse(CRC_HandleTypeDef *hcrc, uint32_t reverse_mode)
{
    if (slot_by_handle(hcrc) == NULL) {
        return HAL_ERROR;
    }

    return HAL_CRCEx_Output_Data_Reverse(hcrc, reverse_mode);
}

HAL_CRC_StateTypeDef bsp_crc_get_state(CRC_HandleTypeDef *hcrc)
{
    if (slot_by_handle(hcrc) == NULL) {
        return HAL_CRC_STATE_RESET;
    }

    return HAL_CRC_GetState(hcrc);
}
