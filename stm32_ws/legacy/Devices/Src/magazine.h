/**
 ******************************************************************************
 * @file    magazine.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   DM-MC02 弹仓舵机设备接口。
 *
 * @note    本文件代表当前板卡上的弹仓开合舵机实例，不代表通用 PWM/TIM BSP。
 * @note    当前弹仓舵机使用配置 PWM 输出；具体 TIM/channel 绑定来自板级配置。
 * @note    差异: 旧版直接包含 tim.h 并写具体 TIM/channel；新版通过 bsp_tim/bsp_pwm
 *          获取板载 PWM 资源，并返回 HAL 状态码。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_MAGAZINE_H
#define PRINTK_DEVICE_MAGAZINE_H

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
 * @brief 初始化板载弹仓舵机 PWM 输出。
 *
 * 本函数会启动弹仓舵机 PWM 通道，并将弹仓置为关闭位置。调用前应先完成配置 PWM 的 CubeMX 初始化。
 *
 * @return 成功返回 HAL_OK，资源不可用或 HAL 启动失败返回对应 HAL 状态码。
 */
HAL_StatusTypeDef magazine_init(void);

/**
 * @brief 打开弹仓。
 *
 * @return 成功返回 HAL_OK，资源不可用返回 HAL_ERROR。
 */
HAL_StatusTypeDef magazine_open(void);

/**
 * @brief 关闭弹仓。
 *
 * @return 成功返回 HAL_OK，资源不可用返回 HAL_ERROR。
 */
HAL_StatusTypeDef magazine_close(void);

/**
 * @brief 设置弹仓舵机 PWM compare 值。
 *
 * @param compare 需要写入配置 PWM 的 compare 值。
 * @return 成功返回 HAL_OK，资源不可用返回 HAL_ERROR。
 */
HAL_StatusTypeDef magazine_set(uint32_t compare);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_MAGAZINE_H
