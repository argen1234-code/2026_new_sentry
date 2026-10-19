/**
 ******************************************************************************
 * @file    bsp_gpio.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   GPIO BSP 抽象接口。
 *
 * @note    本文件只提供 GPIO 引脚映射、HAL GPIO 基础操作包装和 EXTI 回调分发。
 * @note    LED、按键、片选时序、外设复位、电源控制策略等语义应放在设备驱动或
 *          板级服务层，不应放在 GPIO BSP。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_GPIO_H
#define PRINTK_BSP_GPIO_H

#include "gpio.h"

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
 * @brief GPIO 引脚描述。
 */
typedef struct
{
    GPIO_TypeDef *port;    /**< GPIO 端口，例如 GPIOA。 */
    uint16_t pin;          /**< GPIO 引脚掩码，例如 GPIO_PIN_0。 */
} bsp_gpio_pin_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief GPIO EXTI 回调函数类型。
 *
 * @param pin 触发 EXTI 的 GPIO 引脚描述，仅在回调期间有效。
 * @param user 注册回调时传入的用户上下文。
 */
typedef void (*bsp_gpio_exti_callback_t)(const bsp_gpio_pin_t *pin, void *user);

/**
 * @brief 板上可由 BSP 管理的 GPIO 标识。
 *
 * 新增或删除板级 GPIO 时，需要同步维护本枚举和 bsp_gpio.c 中的 bsp_gpio_slots[]。
 */
typedef enum
{
    BSP_GPIO_ID_PC13 = 0,     /**< PC13，当前用于 VCC_OUT2 使能。 */
    BSP_GPIO_ID_PC14,         /**< PC14，当前用于 VCC_OUT1 使能。 */
    BSP_GPIO_ID_PWR_EN_5V,    /**< PC15，对外 5V 使能。 */
    BSP_GPIO_ID_ACC_CS,       /**< 加速度计片选。 */
    BSP_GPIO_ID_GYRO_CS,      /**< 陀螺仪片选。 */
    BSP_GPIO_ID_CAM_TRIG,     /**< 相机触发输入。 */
    BSP_GPIO_ID_SHOOT_DETECT, /**< 发射检测输入。 */
    BSP_GPIO_ID_CS,           /**< 板级通用片选。 */
    BSP_GPIO_ID_ACC_INT,      /**< 加速度计中断输入。 */
    BSP_GPIO_ID_GYRO_INT,     /**< 陀螺仪中断输入。 */
    BSP_GPIO_ID_CLK,          /**< 板级时钟输出脚。 */
    BSP_GPIO_ID_COUNT         /**< GPIO 标识数量。 */
} bsp_gpio_id_t;

/**
 * @brief 初始化 GPIO BSP 回调表。
 *
 * @note 差异: CubeMX 的 `MX_GPIO_Init()` 仍负责实际 GPIO 初始化；本函数只清空 EXTI 回调。
 */
void bsp_gpio_init(void);

/**
 * @brief 根据 BSP GPIO 标识获取引脚描述。
 *
 * @param id BSP GPIO 标识。
 * @return 匹配成功返回引脚描述指针，否则返回 `NULL`。
 */
const bsp_gpio_pin_t *bsp_gpio_pin(bsp_gpio_id_t id);

/**
 * @brief 判断指定 GPIO 引脚是否有效。
 *
 * @param pin GPIO 引脚描述。
 * @return 有效返回 1，否则返回 0。
 */
uint8_t bsp_gpio_is_supported(const bsp_gpio_pin_t *pin);

/**
 * @brief 读取 GPIO 输入或输出电平。
 *
 * @param pin GPIO 引脚描述。
 * @return 返回 HAL GPIO 引脚状态；入参非法时返回 `GPIO_PIN_RESET`。
 */
GPIO_PinState bsp_gpio_read(const bsp_gpio_pin_t *pin);

/**
 * @brief 写入 GPIO 输出电平。
 *
 * @param pin GPIO 引脚描述。
 * @param state 目标输出状态。
 * @return 成功返回 `HAL_OK`，入参非法返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef bsp_gpio_write(const bsp_gpio_pin_t *pin, GPIO_PinState state);

/**
 * @brief 翻转 GPIO 输出电平。
 *
 * @param pin GPIO 引脚描述。
 * @return 成功返回 `HAL_OK`，入参非法返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef bsp_gpio_toggle(const bsp_gpio_pin_t *pin);

/**
 * @brief 初始化 GPIO 引脚。
 *
 * @param port GPIO 端口。
 * @param config HAL GPIO 初始化配置。
 * @return 成功返回 `HAL_OK`，入参非法返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef bsp_gpio_config(GPIO_TypeDef *port, const GPIO_InitTypeDef *config);

/**
 * @brief 反初始化 GPIO 引脚。
 *
 * @param pin GPIO 引脚描述。
 * @return 成功返回 `HAL_OK`，入参非法返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef bsp_gpio_deinit(const bsp_gpio_pin_t *pin);

/**
 * @brief 锁定 GPIO 引脚配置。
 *
 * @param pin GPIO 引脚描述。
 * @return 返回 HAL GPIO 状态码。
 */
HAL_StatusTypeDef bsp_gpio_lock(const bsp_gpio_pin_t *pin);

/**
 * @brief 分发 GPIO EXTI 中断处理。
 *
 * @param pin_mask 触发中断的 GPIO 引脚掩码。
 */
void bsp_gpio_exti_isr(uint16_t pin_mask);

/**
 * @brief 绑定 GPIO EXTI 回调。
 *
 * @param pin GPIO 引脚描述。
 * @param callback EXTI 回调，传入 `NULL` 表示取消绑定。
 * @param user 用户上下文。
 * @return 返回 HAL GPIO 状态码。
 */
HAL_StatusTypeDef bsp_gpio_exti_bind(const bsp_gpio_pin_t *pin,
                                     bsp_gpio_exti_callback_t callback,
                                     void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_GPIO_H
