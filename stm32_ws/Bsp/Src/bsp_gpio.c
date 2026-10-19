/**
 ******************************************************************************
 * @file    bsp_gpio.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   GPIO BSP 抽象实现。
 *
 * @note    本文件只维护板级 GPIO 引脚表、HAL GPIO 基础操作包装和 EXTI 回调分发。
 * @note    与业务混合写法相比，新版不在 BSP 中处理片选时序、外设复位、电源策略、
 *          按键动作或传感器中断解析。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_gpio.h"

#include <stddef.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/**
 * @brief GPIO 分发槽位。
 *
 * 每个槽位对应一个板级 GPIO 引脚及其 EXTI 回调注册信息。
 */
typedef struct
{
    bsp_gpio_id_t id;                          /**< BSP 层临时暴露给上层使用的 GPIO 标识。 */
    bsp_gpio_pin_t pin;                        /**< GPIO 端口和引脚描述。 */
    bsp_gpio_exti_callback_t exti_callback;    /**< EXTI 回调，可为 NULL。 */
    void *exti_user;                           /**< EXTI 用户上下文。 */
} bsp_gpio_slot_t;

/**
 * @brief 板级 GPIO 引脚注册表。
 *
 * 新增或删除 GPIO 时，本表必须与 bsp_gpio_id_t 和 CubeMX 生成的 GPIO 宏保持一致。
 */
static bsp_gpio_slot_t bsp_gpio_slots[] = {
    {.id = BSP_GPIO_ID_PC13, .pin = {.port = GPIOC, .pin = GPIO_PIN_13}},
    {.id = BSP_GPIO_ID_PC14, .pin = {.port = GPIOC, .pin = GPIO_PIN_14}},
    {.id = BSP_GPIO_ID_PWR_EN_5V, .pin = {.port = PWR_EN_5V_GPIO_Port, .pin = PWR_EN_5V_Pin}},
    {.id = BSP_GPIO_ID_ACC_CS, .pin = {.port = ACC_CS_GPIO_Port, .pin = ACC_CS_Pin}},
    {.id = BSP_GPIO_ID_GYRO_CS, .pin = {.port = GYRO_CS_GPIO_Port, .pin = GYRO_CS_Pin}},
    {.id = BSP_GPIO_ID_CAM_TRIG, .pin = {.port = CAM_TRIG_GPIO_Port, .pin = CAM_TRIG_Pin}},
    {.id = BSP_GPIO_ID_SHOOT_DETECT, .pin = {.port = SHOOT_DETECT_GPIO_Port, .pin = SHOOT_DETECT_Pin}},
    {.id = BSP_GPIO_ID_CS, .pin = {.port = CS_GPIO_Port, .pin = CS_Pin}},
    {.id = BSP_GPIO_ID_ACC_INT, .pin = {.port = ACC_INT_GPIO_Port, .pin = ACC_INT_Pin}},
    {.id = BSP_GPIO_ID_GYRO_INT, .pin = {.port = GYRO_INT_GPIO_Port, .pin = GYRO_INT_Pin}},
    {.id = BSP_GPIO_ID_CLK, .pin = {.port = CLK_GPIO_Port, .pin = CLK_Pin}},
};

#define BSP_GPIO_SLOT_COUNT ((uint32_t)(sizeof(bsp_gpio_slots) / sizeof(bsp_gpio_slots[0])))    /**< bsp_gpio_slots[] 中受 BSP 管理的 GPIO 数量。 */

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */

/**
 * @brief 检查 GPIO 引脚描述是否有效。
 *
 * @param pin GPIO 引脚描述。
 * @return 有效返回 1，否则返回 0。
 */
static uint8_t is_pin_valid(const bsp_gpio_pin_t *pin)
{
    if ((pin == NULL) || (pin->port == NULL) || (pin->pin == 0U)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 根据 BSP GPIO 标识查找槽位。
 *
 * @param id BSP GPIO 标识。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_gpio_slot_t *slot_by_id(bsp_gpio_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_GPIO_SLOT_COUNT; i++) {
        if (bsp_gpio_slots[i].id == id) {
            return &bsp_gpio_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 GPIO 引脚描述查找槽位。
 *
 * @param pin GPIO 引脚描述。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_gpio_slot_t *slot_by_pin(const bsp_gpio_pin_t *pin)
{
    uint32_t i;

    if (is_pin_valid(pin) == 0U) {
        return NULL;
    }

    for (i = 0U; i < BSP_GPIO_SLOT_COUNT; i++) {
        if ((bsp_gpio_slots[i].pin.port == pin->port) && (bsp_gpio_slots[i].pin.pin == pin->pin)) {
            return &bsp_gpio_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 EXTI 引脚查找槽位。
 *
 * HAL GPIO EXTI 回调只提供 pin，不提供 port；CubeMX 同一 EXTI line 只会映射到一个端口。
 *
 * @param pin_mask GPIO 引脚掩码。
 * @return 找到则返回槽位指针，否则返回 `NULL`。
 */
static bsp_gpio_slot_t *slot_by_exti_pin(uint16_t pin_mask)
{
    uint32_t i;

    if (pin_mask == 0U) {
        return NULL;
    }

    for (i = 0U; i < BSP_GPIO_SLOT_COUNT; i++) {
        if (bsp_gpio_slots[i].pin.pin == pin_mask) {
            return &bsp_gpio_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 分发 GPIO EXTI 事件。
 *
 * @param slot 触发事件的 GPIO 槽位。
 */
static void dispatch_exti(bsp_gpio_slot_t *slot)
{
    if ((slot == NULL) || (slot->exti_callback == NULL)) {
        return;
    }

    slot->exti_callback(&slot->pin, slot->exti_user);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_gpio_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_GPIO_SLOT_COUNT; i++) {
        /* @diff: CubeMX 仍负责 GPIO 硬件初始化；新版 GPIO BSP 只清空 EXTI 回调分发表。 */
        bsp_gpio_slots[i].exti_callback = NULL;
        bsp_gpio_slots[i].exti_user = NULL;
    }
}

const bsp_gpio_pin_t *bsp_gpio_pin(bsp_gpio_id_t id)
{
    bsp_gpio_slot_t *slot = slot_by_id(id);

    if (slot == NULL) {
        return NULL;
    }

    return &slot->pin;
}

uint8_t bsp_gpio_is_supported(const bsp_gpio_pin_t *pin)
{
    return (slot_by_pin(pin) != NULL) ? 1U : 0U;
}

GPIO_PinState bsp_gpio_read(const bsp_gpio_pin_t *pin)
{
    if (is_pin_valid(pin) == 0U) {
        return GPIO_PIN_RESET;
    }

    return HAL_GPIO_ReadPin(pin->port, pin->pin);
}

HAL_StatusTypeDef bsp_gpio_write(const bsp_gpio_pin_t *pin, GPIO_PinState state)
{
    if (is_pin_valid(pin) == 0U) {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(pin->port, pin->pin, state);
    return HAL_OK;
}

HAL_StatusTypeDef bsp_gpio_toggle(const bsp_gpio_pin_t *pin)
{
    if (is_pin_valid(pin) == 0U) {
        return HAL_ERROR;
    }

    HAL_GPIO_TogglePin(pin->port, pin->pin);
    return HAL_OK;
}

HAL_StatusTypeDef bsp_gpio_config(GPIO_TypeDef *port, const GPIO_InitTypeDef *config)
{
    if ((port == NULL) || (config == NULL) || (config->Pin == 0U)) {
        return HAL_ERROR;
    }

    HAL_GPIO_Init(port, (GPIO_InitTypeDef *)config);
    return HAL_OK;
}

HAL_StatusTypeDef bsp_gpio_deinit(const bsp_gpio_pin_t *pin)
{
    if (is_pin_valid(pin) == 0U) {
        return HAL_ERROR;
    }

    HAL_GPIO_DeInit(pin->port, pin->pin);
    return HAL_OK;
}

HAL_StatusTypeDef bsp_gpio_lock(const bsp_gpio_pin_t *pin)
{
    if (is_pin_valid(pin) == 0U) {
        return HAL_ERROR;
    }

    return HAL_GPIO_LockPin(pin->port, pin->pin);
}

void bsp_gpio_exti_isr(uint16_t pin_mask)
{
    if (pin_mask == 0U) {
        return;
    }

    HAL_GPIO_EXTI_IRQHandler(pin_mask);
}

HAL_StatusTypeDef bsp_gpio_exti_bind(const bsp_gpio_pin_t *pin,
                                     bsp_gpio_exti_callback_t callback,
                                     void *user)
{
    bsp_gpio_slot_t *slot = slot_by_pin(pin);

    if (slot == NULL) {
        return HAL_ERROR;
    }

    slot->exti_callback = callback;
    slot->exti_user = user;
    return HAL_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    bsp_gpio_slot_t *slot = slot_by_exti_pin(GPIO_Pin);

    if (slot == NULL) {
        return;
    }

    dispatch_exti(slot);
}
