/**
 ******************************************************************************
 * @file    buzzer.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DM-MC02 蜂鸣器设备实现。
 *
 * @note    本文件只负责把配置 PWM 组合成板载蜂鸣器输出。
 *          具体 PWM 启停和比较值写入由 `bsp_pwm`/`bsp_tim` 完成。
 * @note    差异: 旧版直接操作 `htim12`；新版通过 `bsp_tim_handle()` 获取
 *          TIM 句柄，便于后续迁移定时器资源。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "buzzer.h"

#include "bsp_tim.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/**
 * @brief 获取板载蜂鸣器使用的 TIM 句柄。
 *
 * @return TIM 句柄，失败返回 `NULL`。
 */
static TIM_HandleTypeDef *buzzer_timer(void)
{
    return bsp_tim_handle(BUZZER_TIM_ID);
}

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 根据当前 TIM 句柄生成蜂鸣器 PWM 描述。
 *
 * @return 蜂鸣器 PWM 描述对象。
 */
static bsp_pwm_t buzzer_ch(void)
{
    bsp_pwm_t pwm;

    pwm.htim = buzzer_timer();
    pwm.channel = BUZZER_TIM_CHANNEL;

    return pwm;
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 判断蜂鸣器 TIM 资源是否可用。
 *
 * @return 可用返回 1，否则返回 0。
 */
static uint8_t is_buzzer_ready(void)
{
    return (buzzer_timer() != NULL) ? 1U : 0U;
}

/**
 * @brief 获取蜂鸣器 TIM 实际计数时钟。
 *
 * 当前蜂鸣器定时器位于 APB1，APB 分频不为 1 时定时器时钟为 PCLK1 的 2 倍。
 *
 * @return 蜂鸣器 TIM 计数时钟，单位 Hz。
 */
static uint32_t tim_clock(void)
{
    RCC_ClkInitTypeDef clk;
    uint32_t latency;
    uint32_t clock;

    HAL_RCC_GetClockConfig(&clk, &latency);
    clock = HAL_RCC_GetPCLK1Freq();
    if (clk.APB1CLKDivider != RCC_HCLK_DIV1) {
        clock *= 2U;
    }

    return clock;
}

/**
 * @brief 根据目标频率计算 TIM 预分频和自动重装值。
 *
 * @param freq_hz 目标频率，单位 Hz。
 * @param prescaler 预分频输出。
 * @param autoreload 自动重装输出。
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t calc_pwm(uint32_t freq_hz, uint32_t *prescaler, uint32_t *autoreload)
{
    uint32_t clock;
    uint64_t total_ticks;
    uint64_t div;
    uint64_t denom;
    uint64_t period_ticks;

    if ((freq_hz == 0U) || (prescaler == NULL) || (autoreload == NULL)) {
        return 0U;
    }

    clock = tim_clock();
    if (clock == 0U) {
        return 0U;
    }

    total_ticks = ((uint64_t)clock + ((uint64_t)freq_hz / 2ULL)) / (uint64_t)freq_hz;
    if ((total_ticks < 2ULL) || (total_ticks > (65536ULL * 65536ULL))) {
        return 0U;
    }

    div = (total_ticks + 65535ULL) / 65536ULL;
    if (div == 0ULL) {
        div = 1ULL;
    }
    if (div > 65536ULL) {
        return 0U;
    }

    denom = div * (uint64_t)freq_hz;
    period_ticks = ((uint64_t)clock + (denom / 2ULL)) / denom;
    if (period_ticks < 2ULL) {
        period_ticks = 2ULL;
    }
    if (period_ticks > 65536ULL) {
        period_ticks = 65536ULL;
    }

    *prescaler = (uint32_t)(div - 1ULL);
    *autoreload = (uint32_t)(period_ticks - 1ULL);
    return 1U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
HAL_StatusTypeDef buzzer_init(void)
{
    bsp_pwm_t pwm = buzzer_ch();
    HAL_StatusTypeDef status;

    if (is_buzzer_ready() == 0U) {
        return HAL_ERROR;
    }

    bsp_pwm_set_compare_value(&pwm, 0U);
    status = bsp_pwm_start(&pwm);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef buzzer_start(uint32_t freq_hz)
{
    return buzzer_start_duty(freq_hz, BUZZER_DUTY_DEFAULT);
}

HAL_StatusTypeDef buzzer_start_duty(uint32_t freq_hz, uint16_t duty_permille)
{
    bsp_pwm_t pwm = buzzer_ch();
    TIM_HandleTypeDef *htim = buzzer_timer();
    HAL_StatusTypeDef status;
    uint32_t prescaler;
    uint32_t autoreload;
    uint32_t compare;

    if (htim == NULL) {
        return HAL_ERROR;
    }

    if (duty_permille > 1000U) {
        duty_permille = 1000U;
    }

    if (calc_pwm(freq_hz, &prescaler, &autoreload) == 0U) {
        return HAL_ERROR;
    }

    /* HAL 不允许对已经 BUSY 的 PWM 通道再次 Start，重设频率前先释放通道状态。 */
    (void)bsp_pwm_stop(&pwm);

    compare = (uint32_t)((((uint64_t)autoreload + 1ULL) * (uint64_t)duty_permille) / 1000ULL);
    if (compare > autoreload) {
        compare = autoreload;
    }

    bsp_pwm_set_compare_value(&pwm, compare);
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
    __HAL_TIM_SET_AUTORELOAD(htim, autoreload);
    __HAL_TIM_SET_COUNTER(htim, 0U);
    (void)HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

    status = bsp_pwm_start(&pwm);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef buzzer_off(void)
{
    bsp_pwm_t pwm = buzzer_ch();

    if (is_buzzer_ready() == 0U) {
        return HAL_ERROR;
    }

    /* @diff: 与旧版保持一致，仅清零 compare 使蜂鸣器静音，不强制停止 PWM 通道。 */
    bsp_pwm_set_compare_value(&pwm, 0U);
    return HAL_OK;
}
