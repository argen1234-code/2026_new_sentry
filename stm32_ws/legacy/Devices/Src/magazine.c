/**
 ******************************************************************************
 * @file    magazine.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DM-MC02 弹仓舵机设备实现。
 *
 * @note    本文件只保留弹仓开合语义；PWM 底层操作委托给 bsp_pwm/bsp_tim。
 * @note    差异: 旧版直接使用具体 TIM/channel；新版将 PWM 绑定集中到板级配置，
 *          避免上层散落硬件编号。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "magazine.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 获取板载弹仓舵机 PWM 描述对象。
 */
static bsp_pwm_t mag_pwm(void)
{
    bsp_pwm_t pwm;

    pwm.htim = bsp_tim_handle(MAGAZINE_TIM_ID);
    pwm.channel = MAGAZINE_TIM_CHANNEL;

    return pwm;
}

/**
 * @brief 判断弹仓舵机 PWM 资源是否可用。
 */
static int is_pwm_ok(const bsp_pwm_t *pwm)
{
    return (pwm != NULL) && (pwm->htim != NULL);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
HAL_StatusTypeDef magazine_init(void)
{
    bsp_pwm_t pwm = mag_pwm();
    HAL_StatusTypeDef status;

    if (is_pwm_ok(&pwm) == 0) {
        return HAL_ERROR;
    }

    status = bsp_pwm_start(&pwm);
    if (status != HAL_OK) {
        return status;
    }

    return magazine_close();
}

HAL_StatusTypeDef magazine_open(void)
{
    return magazine_set(MAGAZINE_COMPARE_OPEN);
}

HAL_StatusTypeDef magazine_close(void)
{
    return magazine_set(MAGAZINE_COMPARE_CLOSED);
}

HAL_StatusTypeDef magazine_set(uint32_t compare)
{
    bsp_pwm_t pwm = mag_pwm();

    if (is_pwm_ok(&pwm) == 0) {
        return HAL_ERROR;
    }

    bsp_pwm_set_compare_value(&pwm, compare);
    return HAL_OK;
}
