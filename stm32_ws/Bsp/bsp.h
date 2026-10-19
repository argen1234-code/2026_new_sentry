/**
 ******************************************************************************
 * @file    bsp.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   DM-MC02 板级 BSP 一键初始化接口。
 *
 * @note    CubeMX 生成的 `MX_*_Init()` 仍由 main 调用，本文件只负责在 HAL
 *          句柄已创建后，按固定顺序初始化 BSP 外设抽象和板载设备抽象。
 * @note    main 只需要在所有 `MX_*_Init()` 完成后调用 `bsp_init()`，避免上层散落
 *          bsp_uart、bsp_gpio、imu 等初始化顺序。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_H
#define PRINTK_BSP_H

#include "bsp_fdcan.h"
#include "bsp_gpio.h"
#include "bsp_ospi.h"
#include "bsp_spi.h"
#include "bsp_tim.h"
#include "bsp_uart.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 总入口只描述板级初始化状态、阶段和诊断结果，不承载具体机器人业务语义。
 * ========================================================================== */

/**
 * @brief BSP 初始化状态码。
 */
typedef enum
{
    BSP_OK = 0,          /**< 初始化成功。 */
    BSP_ERR_HAL = 1,     /**< HAL 或外设 BSP 初始化失败。 */
    BSP_ERR_POWER = 2,   /**< 电源设备初始化失败。 */
    BSP_ERR_MOTOR = 3,   /**< 电机设备初始化失败。 */
    BSP_ERR_FLASH = 4,   /**< Flash 设备初始化失败。 */
    BSP_ERR_IMU = 5,     /**< IMU 设备初始化失败。 */
    BSP_ERR_TFMINI = 6,  /**< TFmini 设备初始化失败。 */
    BSP_ERR_RC = 7       /**< 遥控器设备初始化失败。 */
} bsp_status_t;

/**
 * @brief BSP 一键初始化阶段。
 */
typedef enum
{
    BSP_STEP_NONE = 0,     /**< 尚未开始初始化。 */
    BSP_STEP_TABLE = 1,    /**< BSP 外设句柄表初始化阶段。 */
    BSP_STEP_DWT = 2,      /**< DWT 时间基准初始化阶段。 */
    BSP_STEP_TIMEBASE = 3, /**< 通用时间基准初始化阶段。 */
    BSP_STEP_POWER = 4,    /**< 电源设备初始化阶段。 */
    BSP_STEP_MOTOR = 5,    /**< 电机设备初始化阶段。 */
    BSP_STEP_FDCAN = 6,    /**< FDCAN 默认接收启动阶段。 */
    BSP_STEP_BUZZER = 7,   /**< 蜂鸣器设备初始化阶段。 */
    BSP_STEP_MAGAZINE = 8, /**< 弹舱舵机设备初始化阶段。 */
    BSP_STEP_FLASH = 9,    /**< Flash 设备初始化阶段。 */
    BSP_STEP_IMU = 10,     /**< IMU 设备初始化阶段。 */
    BSP_STEP_TFMINI = 11,  /**< TFmini 设备初始化阶段。 */
    BSP_STEP_RC = 12,      /**< 遥控器设备初始化阶段。 */
    BSP_STEP_DONE = 13     /**< BSP 一键初始化完成。 */
} bsp_step_t;

/**
 * @brief BSP 最近一次初始化结果。
 */
typedef struct
{
    bsp_status_t status; /**< 初始化状态码。 */
    bsp_step_t step;     /**< 最近一次执行到的初始化阶段。 */
    int32_t detail;      /**< 设备或 HAL 返回的补充错误信息。 */
} bsp_result_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief 执行板级 BSP 一键初始化。
 *
 * 调用前必须已经完成 `HAL_Init()`、系统时钟配置以及 CubeMX 生成的全部
 * `MX_*_Init()`。
 *
 * @return 初始化成功返回 `BSP_OK`，失败返回对应错误码。
 */
bsp_status_t bsp_init(void);

/**
 * @brief 获取最近一次 `bsp_init()` 的详细结果。
 *
 * @return 返回只读初始化结果指针。
 */
const bsp_result_t *bsp_result(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_H
