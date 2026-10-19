/**
 ******************************************************************************
 * @file    buzzer.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DM-MC02 蜂鸣器设备接口。
 *
 * @note    本文件只描述本板蜂鸣器资源组合：当前蜂鸣器使用配置 PWM 输出。
 * @note    TIM/PWM 的底层操作由 `bsp_tim` 和 `bsp_pwm` 提供；本层只保留
 *          蜂鸣器语义，避免上层直接散落定时器和通道编号。
 * @note    差异: 旧版直接包含 `tim.h` 并写 `htim12` 的 PSC/CCR；新版通过
 *          当前 TIM/PWM BSP 获取句柄并返回 HAL 状态码。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_BUZZER_H
#define PRINTK_DEVICE_BUZZER_H

#include "bsp_pwm.h"
#include "robot_board.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化板载蜂鸣器输出。
 *
 * 该函数会启动蜂鸣器 PWM 通道，并将比较值清零，使蜂鸣器默认静音。
 *
 * @note 调用前应先完成配置 PWM 的 CubeMX 初始化。
 *
 * @return 成功返回 `HAL_OK`，资源不可用或 HAL 启动失败返回对应 HAL 状态码。
 */
HAL_StatusTypeDef buzzer_init(void);

/**
 * @brief 按频率打开蜂鸣器输出。
 *
 * 本接口使用默认 50% 占空比，上层只需要给出目标频率即可发声。
 *
 * @param freq_hz 目标频率，单位 Hz，必须大于 0。
 * @return 成功返回 `HAL_OK`，参数或资源不可用返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef buzzer_start(uint32_t freq_hz);

/**
 * @brief 按频率和占空比打开蜂鸣器输出。
 *
 * @param freq_hz 目标频率，单位 Hz，必须大于 0。
 * @param duty_permille 占空比千分比，范围 0~1000，超过 1000 会裁剪。
 * @return 成功返回 `HAL_OK`，参数或资源不可用返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef buzzer_start_duty(uint32_t freq_hz, uint16_t duty_permille);

/**
 * @brief 关闭蜂鸣器输出。
 *
 * 该函数仅将 PWM 比较值清零，保持旧版“清零 CCR 静音”的行为。
 *
 * @return 成功返回 `HAL_OK`，资源不可用返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef buzzer_off(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_BUZZER_H
