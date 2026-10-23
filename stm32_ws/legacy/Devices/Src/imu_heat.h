/**
 ******************************************************************************
 * @file    imu_heat.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   DM-MC02 板载 IMU 加热 PWM 适配接口。
 *
 * @note    本文件只描述 IMU 加热片硬件语义，具体 PWM 绑定来自板级配置。
 *          上层不应直接操作 TIM/CCR。
 *
 * @note    温控 PID 和温度策略由 Component/Controller/Thermal 负责，本层只执行 PWM 初始化和输出。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_IMU_HEAT_H
#define PRINTK_DEVICE_IMU_HEAT_H

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
 * @brief 初始化 IMU 加热 PWM。
 *
 * @return 成功返回 `HAL_OK`，资源不可用或 HAL 启动失败返回对应 HAL 状态码。
 */
HAL_StatusTypeDef imu_heat_init(void);

/**
 * @brief 设置 IMU 加热 PWM compare。
 *
 * 当 compare 超过当前 ARR 时，底层会裁剪到 ARR。
 *
 * @param compare PWM compare 值。
 * @return 成功返回 `HAL_OK`，资源不可用返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef imu_heat_set(uint32_t compare);

/**
 * @brief 关闭 IMU 加热输出。
 *
 * @return 成功返回 `HAL_OK`，资源不可用返回 `HAL_ERROR`。
 */
HAL_StatusTypeDef imu_heat_off(void);

/**
 * @brief 获取 IMU 加热 PWM 当前 ARR。
 *
 * @return ARR 值；资源不可用时返回 0。
 */
uint32_t imu_heat_max(void);

/**
 * @brief 读取 IMU 加热 PWM 当前 CCR。
 *
 * @return CCR 值；资源不可用时返回 0。
 */
uint32_t imu_heat_get(void);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_DEVICE_IMU_HEAT_H */
