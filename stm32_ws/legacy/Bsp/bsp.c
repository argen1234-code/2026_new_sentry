/**
 ******************************************************************************
 * @file    bsp.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   DM-MC02 板级 BSP 一键初始化实现。
 *
 * @note    本文件只编排初始化顺序，不替代 CubeMX 生成的 `MX_*_Init()`。
 * @note    初始化顺序原则：先建立 Peripheral BSP 回调表和句柄表，再建立时间基准，
 *          随后启动 FDCAN 默认接收，最后初始化依赖这些资源的板载设备。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp.h"

#include "main.h"
#include "bsp_adc.h"
#include "bsp_cdc.h"
#include "bsp_crc.h"
#include "bsp_dts.h"
#include "bsp_dwt.h"
#include "bsp_fdcan.h"
#include "bsp_gpio.h"
#include "bsp_rng.h"
#include "bsp_spi.h"
#include "bsp_tim.h"
#include "bsp_timebase.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "buzzer.h"
#include "flash.h"
#include "imu.h"
#include "magazine.h"
#include "motor.h"
#include "power.h"
#include "rc.h"
#include "robot_board.h"
#include "tfmini.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */

/** @brief 最近一次 BSP 一键初始化结果。 */
static bsp_result_t last_result;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责初始化结果记录、CubeMX 频率读取和 BSP 句柄表初始化。
 * ========================================================================== */

/**
 * @brief 记录并返回 BSP 初始化状态。
 *
 * @param status 初始化状态码。
 * @param step 当前初始化阶段。
 * @param detail 设备或 HAL 返回的补充错误信息。
 * @return 原样返回 status，便于调用处直接 return。
 */
static bsp_status_t set_res(bsp_status_t status, bsp_step_t step, int32_t detail)
{
    last_result.status = status;
    last_result.step = step;
    last_result.detail = detail;
    return status;
}

/**
 * @brief 获取当前 CPU 主频。
 *
 * @return CPU 主频，单位 MHz；频率无效时返回 0。
 */
static uint32_t cpu_freq_mhz(void)
{
    uint32_t hz = HAL_RCC_GetHCLKFreq();

    if (hz == 0U) {
        return 0U;
    }

    return hz / 1000000U;
}

/**
 * @brief 初始化 BSP 外设句柄表和回调分发表。
 *
 * 这里不启动具体业务设备，只让各外设 BSP 建立 CubeMX 句柄到 BSP ID 的映射。
 */
static void init_table(void)
{
    bsp_adc_init();
    bsp_crc_init();
    bsp_dts_init();
    bsp_fdcan_init();
    bsp_gpio_init();
    bsp_rng_init();
    bsp_spi_init();
    bsp_tim_init();
    bsp_uart_init();
    bsp_usb_init();
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
bsp_status_t bsp_init(void)
{
    uint32_t dwt_freq_mhz;

    (void)set_res(BSP_OK, BSP_STEP_NONE, 0);

    init_table();
    (void)set_res(BSP_OK, BSP_STEP_TABLE, 0);

    dwt_freq_mhz = cpu_freq_mhz();
    if (dwt_freq_mhz == 0U) {
        return set_res(BSP_ERR_HAL, BSP_STEP_DWT, 0);
    }
    bsp_dwt_init(dwt_freq_mhz);
    (void)set_res(BSP_OK, BSP_STEP_DWT, (int32_t)dwt_freq_mhz);

    bsp_timebase_init();
    (void)set_res(BSP_OK, BSP_STEP_TIMEBASE, 0);

#if BSP_INIT_POWER
    if (power_init() != PowerOk) {
        return set_res(BSP_ERR_POWER, BSP_STEP_POWER, -1);
    }
    (void)set_res(BSP_OK, BSP_STEP_POWER, 0);
#endif

#if BSP_INIT_CDC
    /*
     * CDC 仅用于临时上机调试。启动失败不阻塞整板初始化，避免无 USB 主机时影响
     * 遥控和云台测试。
     */
    (void)bsp_cdc_init();
#endif

#if BSP_INIT_MOTOR
    if (motor_init() != MotorOk) {
        return set_res(BSP_ERR_MOTOR, BSP_STEP_MOTOR, -1);
    }
#if defined(ROBOT_TARGET_MOTOR_CHASSIS_DM_BIND) && (ROBOT_TARGET_MOTOR_CHASSIS_DM_BIND != 0U)
    {
        uint32_t i;
        for (i = 0U; i < 4U; i++) {
            if (motor_dm_bind(MOTOR_CHASSIS_BUS,
                              (uint16_t)(ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_0 + i),
                              DmMotor6215) != MotorOk) {
                return set_res(BSP_ERR_MOTOR, BSP_STEP_MOTOR, -2);
            }
        }
    }
#endif
    (void)set_res(BSP_OK, BSP_STEP_MOTOR, 0);
#endif

#if BSP_INIT_FDCAN
    if (bsp_fdcan_start_default() != HAL_OK) {
        return set_res(BSP_ERR_HAL, BSP_STEP_FDCAN, -1);
    }
    (void)set_res(BSP_OK, BSP_STEP_FDCAN, 0);
#endif

#if BSP_INIT_BUZZER
    if (buzzer_init() != HAL_OK) {
        return set_res(BSP_ERR_HAL, BSP_STEP_BUZZER, -1);
    }
    (void)set_res(BSP_OK, BSP_STEP_BUZZER, 0);
#endif

#if BSP_INIT_MAGAZINE
    if (magazine_init() != HAL_OK) {
        return set_res(BSP_ERR_HAL, BSP_STEP_MAGAZINE, -1);
    }
    (void)set_res(BSP_OK, BSP_STEP_MAGAZINE, 0);
#endif

#if BSP_INIT_FLASH
    {
        flash_status_t flash_status = flash_init();
        if (flash_status != FLASH_OK) {
            return set_res(BSP_ERR_FLASH, BSP_STEP_FLASH, (int32_t)flash_status);
        }
        (void)set_res(BSP_OK, BSP_STEP_FLASH, 0);
    }
#endif

#if BSP_INIT_IMU
    {
        imu_status_t imu_status = imu_init((uint8_t)BSP_IMU_CALI);
        if (imu_status != IMU_OK) {
            return set_res(BSP_ERR_IMU, BSP_STEP_IMU, (int32_t)imu_status);
        }
        (void)set_res(BSP_OK, BSP_STEP_IMU, 0);
    }
#endif

#if BSP_INIT_TFMINI
    {
        tfmini_status_t tfmini_status = tfmini_init();
        if (tfmini_status != TFMINI_OK) {
            return set_res(BSP_ERR_TFMINI, BSP_STEP_TFMINI, (int32_t)tfmini_status);
        }
        (void)set_res(BSP_OK, BSP_STEP_TFMINI, 0);
    }
#endif

#if BSP_INIT_RC
    if (rc_init() != HAL_OK) {
        return set_res(BSP_ERR_RC, BSP_STEP_RC, -1);
    }
    (void)set_res(BSP_OK, BSP_STEP_RC, 0);
#endif

    return set_res(BSP_OK, BSP_STEP_DONE, 0);
}

const bsp_result_t *bsp_result(void)
{
    return &last_result;
}
