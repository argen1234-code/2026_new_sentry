/**
 ******************************************************************************
 * @file    imu_heat.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   DM-MC02 板载 IMU 加热 PWM 适配实现。
 *
 * @note    本文件将原工程直接写 TIM/CCR 的 IMU 温控输出收敛到 BSP 语义接口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "imu_heat.h"

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
 * @brief 获取 IMU 加热 PWM 描述对象。
 *
 * @return PWM 描述对象。
 */
static bsp_pwm_t heater_pwm(void)
{
    bsp_pwm_t pwm;

    pwm.htim = bsp_tim_handle(IMU_HEAT_TIM_ID);
    pwm.channel = IMU_HEAT_CHANNEL;

    return pwm;
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 判断 IMU 加热 PWM 资源是否可用。
 *
 * @param pwm PWM 描述对象。
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t heater_pwm_ready(const bsp_pwm_t *pwm)
{
    return ((pwm != NULL) && (pwm->htim != NULL)) ? 1U : 0U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
HAL_StatusTypeDef imu_heat_init(void)
{
    bsp_pwm_t pwm = heater_pwm();
    HAL_StatusTypeDef status;

    if (heater_pwm_ready(&pwm) == 0U) {
        return HAL_ERROR;
    }

    bsp_pwm_set_compare_value(&pwm, 0U);
    status = bsp_pwm_start(&pwm);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef imu_heat_set(uint32_t compare)
{
    bsp_pwm_t pwm = heater_pwm();

    if (heater_pwm_ready(&pwm) == 0U) {
        return HAL_ERROR;
    }

    /*
     * 完整原工程 ARR 同为 9999，MAX_OUT=500 本来就是直接写 CCR
     * 后的 5% compare 语义；这里不再做旧 1000 周期映射。
     */
    bsp_pwm_set_compare_value(&pwm, compare);
    return HAL_OK;
}

HAL_StatusTypeDef imu_heat_off(void)
{
    return imu_heat_set(0U);
}

uint32_t imu_heat_max(void)
{
    bsp_pwm_t pwm = heater_pwm();

    if (heater_pwm_ready(&pwm) == 0U) {
        return 0U;
    }

    return bsp_tim_get_autoreload(pwm.htim);
}

uint32_t imu_heat_get(void)
{
    bsp_pwm_t pwm = heater_pwm();

    if (heater_pwm_ready(&pwm) == 0U) {
        return 0U;
    }

    return bsp_tim_get_compare(pwm.htim, pwm.channel);
}
