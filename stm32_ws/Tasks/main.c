/**
 ******************************************************************************
 * @file    main.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   固件主入口实现。
 *
 * @note    本文件替代 CubeMX 生成的 main 作为最终入口；CubeMX 生成的
 *          `MX_*_Init()`、中断、HAL MSP 和 FreeRTOS 文件仍作为板级事实使用。
 * @note    target 选择只在编译期展开，当前默认 target 为
 *          `sentry_2026`。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "main.h"
#include "firmware_main.h"

#include "adc.h"
#include "bsp.h"
#include "buzzer.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dma.h"
#include "dts.h"
#include "fdcan.h"
#include "gpio.h"
#include "octospi.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/**
 * @brief FreeRTOS 对象初始化入口。
 *
 * 该函数仍由 CubeMX 生成的 `freertos.c` 提供，内部只负责 RTOS 对象创建和
 * 调用 `task_main_init()`。
 */
extern void MX_FREERTOS_Init(void);

void task_main_periph_init(void)
{
#if (TASK_MAIN_TARGET == ROBOT_TARGET_INFANTRY_STEER_2025) || \
    (TASK_MAIN_TARGET == ROBOT_TARGET_SENTRY_2026) || \
    (TASK_MAIN_TARGET == ROBOT_TARGET_SENTRY_2026_BRINGUP) || \
    (TASK_MAIN_TARGET == ROBOT_TARGET_CAN1_DMH6215_TEST)
    /*
     * 当前 target 使用 DM02 STM32H723 板卡。这里集中调用已由 CubeMX 生成的
     * 外设初始化函数，BSP/Devices 后续只消费已初始化好的 HAL 句柄。
     */
    MX_GPIO_Init();
    MX_DMA_Init();

    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();

    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM12_Init();
    MX_TIM24_Init();

    MX_UART5_Init();
    MX_UART7_Init();
    MX_UART8_Init();
    MX_UART9_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART10_UART_Init();

    MX_SPI2_Init();
    MX_SPI6_Init();
    MX_OCTOSPI2_Init();

    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_CRC_Init();
    MX_RNG_Init();
    MX_DTS_Init();
#elif TASK_MAIN_TARGET == ROBOT_TARGET_BOARD_ONLY_DM02
    /*
     * 光板测试 target 只启动板载资源需要的 CubeMX 外设。
     * 不初始化 FDCAN、遥控器、裁判串口和电机相关外设，避免未接负载时产生无意义告警或输出。
     */
    MX_GPIO_Init();
    MX_DMA_Init();

    MX_TIM12_Init();
    MX_TIM24_Init();

    MX_SPI2_Init();
    MX_SPI6_Init();
    MX_OCTOSPI2_Init();

    MX_CRC_Init();
    MX_RNG_Init();
    MX_DTS_Init();
#else
#error "Unknown TASK_MAIN_TARGET. Please add a target branch in Tasks/main.c."
#endif
}

int main(void)
{
    HAL_Init();

    SystemClock_Config();
    PeriphCommonClock_Config();
    task_main_periph_init();

    HAL_Delay(200U);

    if (bsp_init() != BSP_OK) {
        Error_Handler();
    }

#if BSP_INIT_BUZZER != 0U
    (void)buzzer_start(1800U);
    HAL_Delay(100U);
    (void)buzzer_off();
#endif

    osKernelInitialize();
    MX_FREERTOS_Init();
    osKernelStart();

    while (1) {
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef rcc_osc = {0};
    RCC_ClkInitTypeDef rcc_clk = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    rcc_osc.HSEState = RCC_HSE_ON;
    rcc_osc.HSI48State = RCC_HSI48_ON;
    rcc_osc.PLL.PLLState = RCC_PLL_ON;
    rcc_osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc.PLL.PLLM = 2;
    rcc_osc.PLL.PLLN = 40;
    rcc_osc.PLL.PLLP = 1;
    rcc_osc.PLL.PLLQ = 2;
    rcc_osc.PLL.PLLR = 2;
    rcc_osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    rcc_osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    rcc_osc.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK) {
        Error_Handler();
    }

    rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                        RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk.AHBCLKDivider = RCC_HCLK_DIV2;
    rcc_clk.APB3CLKDivider = RCC_APB3_DIV2;
    rcc_clk.APB1CLKDivider = RCC_APB1_DIV2;
    rcc_clk.APB2CLKDivider = RCC_APB2_DIV2;
    rcc_clk.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef periph_clk = {0};

    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_FDCAN;
    periph_clk.PLL2.PLL2M = 24;
    periph_clk.PLL2.PLL2N = 200;
    periph_clk.PLL2.PLL2P = 2;
    periph_clk.PLL2.PLL2Q = 2;
    periph_clk.PLL2.PLL2R = 2;
    periph_clk.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    periph_clk.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    periph_clk.PLL2.PLL2FRACN = 0;
    periph_clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
    periph_clk.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;

    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM6)) {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
