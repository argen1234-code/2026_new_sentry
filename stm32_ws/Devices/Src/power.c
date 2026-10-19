/**
 ******************************************************************************
 * @file    power.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   板载可控电源输出设备实现。
 *
 * @note    本文件把配置 GPIO 组合成原理图中的
 *          OUT2、OUT1 和对外 5V 使能控制。
 * @note    本文件不读取业务状态，不做功率策略，也不接管裁判系统电源逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "power.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/**
 * @brief 单路电源输出描述。
 */
typedef struct
{
    PowerChannel channel; /**< 电源通道。 */
    bsp_gpio_id_t gpio_id; /**< 对应的 BSP GPIO 标识。 */
    uint8_t default_on; /**< 初始化默认使能状态。 */
} PowerSlot;

/**
 * @brief 板载三路可控电源映射表。
 */
static const PowerSlot power_slots[] = {
    {.channel = PowerOut1, .gpio_id = POWER_OUT1_GPIO_ID, .default_on = POWER_OUT1_DEFAULT_ON},
    {.channel = PowerOut2, .gpio_id = POWER_OUT2_GPIO_ID, .default_on = POWER_OUT2_DEFAULT_ON},
    {.channel = Power5v, .gpio_id = POWER_5V_GPIO_ID, .default_on = POWER_5V_DEFAULT_ON},
};

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */
/** @brief 电源设备初始化完成标志。 */
static uint8_t power_is_ready;

/**
 * @brief 根据通道查找电源描述。
 *
 * @param channel 电源通道。
 * @return 成功返回电源描述指针，否则返回 NULL。
 */
static const PowerSlot *find_slot(PowerChannel channel)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)PowerCount; i++) {
        if (power_slots[i].channel == channel) {
            return &power_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 获取通道对应的 GPIO 引脚。
 *
 * @param slot 电源描述。
 * @return 成功返回 GPIO 引脚描述，否则返回 NULL。
 */
static const bsp_gpio_pin_t *slot_pin(const PowerSlot *slot)
{
    if (slot == NULL) {
        return NULL;
    }

    return bsp_gpio_pin(slot->gpio_id);
}

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 将使能状态转换为 GPIO 电平。
 *
 * @param enable 非 0 表示打开。
 * @return 返回 GPIO 输出电平。
 */
static GPIO_PinState level_from_enable(uint8_t enable)
{
    return (enable != 0U) ? POWER_ON_LEVEL : POWER_OFF_LEVEL;
}

/**
 * @brief 写入单路电源输出。
 *
 * @param slot 电源描述。
 * @param enable 非 0 表示打开。
 * @return 返回电源设备状态码。
 */
static PowerStatus write_slot(const PowerSlot *slot, uint8_t enable)
{
    const bsp_gpio_pin_t *pin = slot_pin(slot);

    if (pin == NULL) {
        return PowerIo;
    }

    return (bsp_gpio_write(pin, level_from_enable(enable)) == HAL_OK) ? PowerOk : PowerIo;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
PowerStatus power_init(void)
{
    uint32_t i;
    PowerStatus status;

    power_is_ready = 0U;

    for (i = 0U; i < (uint32_t)PowerCount; i++) {
        status = write_slot(&power_slots[i], power_slots[i].default_on);
        if (status != PowerOk) {
            return status;
        }
    }

    power_is_ready = 1U;
    return PowerOk;
}

uint8_t power_ready(void)
{
    return power_is_ready;
}

PowerStatus power_set(PowerChannel channel, uint8_t enable)
{
    const PowerSlot *slot;

    if (power_is_ready == 0U) {
        return PowerNotReady;
    }

    slot = find_slot(channel);
    if (slot == NULL) {
        return PowerArg;
    }

    return write_slot(slot, enable);
}

PowerStatus power_on(PowerChannel channel)
{
    return power_set(channel, 1U);
}

PowerStatus power_off(PowerChannel channel)
{
    return power_set(channel, 0U);
}

PowerStatus power_toggle(PowerChannel channel)
{
    return power_set(channel, (uint8_t)(power_get(channel) == 0U));
}

uint8_t power_get(PowerChannel channel)
{
    const PowerSlot *slot = find_slot(channel);
    const bsp_gpio_pin_t *pin = slot_pin(slot);

    if ((power_is_ready == 0U) || (pin == NULL)) {
        return 0U;
    }

    return (bsp_gpio_read(pin) == POWER_ON_LEVEL) ? 1U : 0U;
}

PowerStatus power_set_all(uint8_t out1_enable, uint8_t out2_enable, uint8_t v5_enable)
{
    PowerStatus status;

    status = power_set(PowerOut1, out1_enable);
    if (status != PowerOk) {
        return status;
    }

    status = power_set(PowerOut2, out2_enable);
    if (status != PowerOk) {
        return status;
    }

    return power_set(Power5v, v5_enable);
}

PowerStatus power_all_off(void)
{
    return power_set_all(0U, 0U, 0U);
}
