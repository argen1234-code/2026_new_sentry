/**
 ******************************************************************************
 * @file    bsp_pwm.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   PWM 语义包装接口。
 *
 * @note    PWM 底层实现统一委托给 bsp_tim。该文件只保留 PWM 语义，
 *          避免上层设备代码直接散落 htim + channel 操作。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_PWM_H
#define PRINTK_BSP_PWM_H

#include "bsp_tim.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */

/**
 * @brief PWM 通道描述对象。
 *
 * 当新增 PWM 设备时，建议在设备驱动中定义 const bsp_pwm_t，
 * 把具体 htim/channel 绑定在一处，避免上层散落硬件编号。
 */
typedef struct
{
    TIM_HandleTypeDef *htim;    /**< PWM 所属 TIM 句柄。 */
    uint32_t channel;           /**< HAL 定义的 TIM 通道，例如 `TIM_CHANNEL_1`。 */
} bsp_pwm_t;

/**
 * @brief 启动 PWM 输出。
 *
 * @param pwm PWM 通道描述对象。
 * @return 返回 HAL TIM 状态码。
 */
static inline HAL_StatusTypeDef bsp_pwm_start(const bsp_pwm_t *pwm)
{
    if ((pwm == NULL) || (pwm->htim == NULL)) {
        return HAL_ERROR;
    }

    return bsp_tim_start_pwm(pwm->htim, pwm->channel);
}

/**
 * @brief 停止 PWM 输出。
 *
 * @param pwm PWM 通道描述对象。
 * @return 返回 HAL TIM 状态码。
 */
static inline HAL_StatusTypeDef bsp_pwm_stop(const bsp_pwm_t *pwm)
{
    if ((pwm == NULL) || (pwm->htim == NULL)) {
        return HAL_ERROR;
    }

    return bsp_tim_stop_pwm(pwm->htim, pwm->channel);
}

/**
 * @brief 通过 PWM 对象设置比较值。
 *
 * @param pwm PWM 通道描述对象。
 * @param value 需要写入的比较值。
 */
static inline void bsp_pwm_set_compare_value(const bsp_pwm_t *pwm, uint32_t value)
{
    if ((pwm == NULL) || (pwm->htim == NULL)) {
        return;
    }

    bsp_tim_set_compare(pwm->htim, pwm->channel, value);
}

/**
 * @brief 通过千分比设置 PWM 占空比。
 *
 * @param pwm PWM 通道描述对象。
 * @param permille 占空比千分比，范围 0~1000，超过 1000 会裁剪。
 */
static inline void bsp_pwm_set_duty_permille(const bsp_pwm_t *pwm, uint16_t permille)
{
    uint32_t autoreload;
    uint32_t compare;

    if ((pwm == NULL) || (pwm->htim == NULL)) {
        return;
    }

    if (permille > 1000U) {
        permille = 1000U;
    }

    autoreload = bsp_tim_get_autoreload(pwm->htim);
    compare = (uint32_t)((((uint64_t)autoreload + 1ULL) * (uint64_t)permille) / 1000ULL);
    if (compare > autoreload) {
        compare = autoreload;
    }

    bsp_tim_set_compare(pwm->htim, pwm->channel, compare);
}

/**
 * @brief 兼容旧版 htim + channel 形式的 compare 设置接口。
 *
 * @note 差异: 旧版 bsp_PWM.c 直接写 CCR1~CCR4；新版通过 bsp_tim_set_compare() 统一实现。
 *
 * @param htim 目标 TIM 句柄。
 * @param channel HAL 定义的 TIM 通道，例如 `TIM_CHANNEL_1`。
 * @param value 需要写入的比较值。
 */
static inline void bsp_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t value)
{
    bsp_tim_set_compare(htim, channel, value);
}

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_PWM_H
