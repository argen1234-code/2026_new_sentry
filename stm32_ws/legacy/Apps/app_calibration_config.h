/**
 ******************************************************************************
 * @file    app_calibration_config.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   Calibration 应用层配置。
 *
 * @note    本文件只展开当前 target 的校准任务周期、Flash 地址和提示音参数。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_CALIBRATION_CONFIG_H
#define PRINTK_APPS_CALIBRATION_CONFIG_H

#include "cmsis_os.h"
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 任务参数与 target 展开
 * --------------------------------------------------------------------------
 * 配置头只展开任务周期、超时、功能开关和 Robot target 参数，不在这里写运行时逻辑。
 * ========================================================================== */
/** @brief Calibration 任务栈大小，单位 byte。 */

#ifndef APP_CALI_STACK_SIZE
#define APP_CALI_STACK_SIZE ROBOT_APP_CALI_STACK_SIZE
#endif

/** @brief Calibration 任务优先级。 */
#ifndef APP_CALI_PRIORITY
#define APP_CALI_PRIORITY ROBOT_APP_CALI_PRIORITY
#endif

/** @brief Calibration 任务周期，单位 ms。 */
#ifndef APP_CALI_PERIOD_MS
#define APP_CALI_PERIOD_MS ROBOT_APP_CALI_PERIOD_MS
#endif

/** @brief Calibration Flash 存储起始地址。 */
#ifndef APP_CALI_FLASH_ADDR
#define APP_CALI_FLASH_ADDR ROBOT_APP_CALI_FLASH_ADDR
#endif

/** @brief Calibration Flash 存储区域大小。 */
#ifndef APP_CALI_FLASH_SIZE
#define APP_CALI_FLASH_SIZE ROBOT_FLASH_DEVICE_ERASE_SIZE
#endif

/** @brief 校准提示音频率，单位 Hz。 */
#ifndef APP_CALI_BUZZER_HZ
#define APP_CALI_BUZZER_HZ 2000U
#endif

/** @brief 成功保存后的蜂鸣器静音延迟，单位周期。 */
#ifndef APP_CALI_BUZZER_OFF_COUNT
#define APP_CALI_BUZZER_OFF_COUNT 10U
#endif

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_CALIBRATION_CONFIG_H */
