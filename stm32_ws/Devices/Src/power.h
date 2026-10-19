/**
 ******************************************************************************
 * @file    power.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   板载可控电源输出设备接口。
 *
 * @note    本文件只描述原理图中的三路可控电源使能：VCC_OUT1、VCC_OUT2
 *          和对外 5V。底层 GPIO 由 BSP 提供。
 * @note    当前硬件为高电平使能：GPIO 输出 SET 时打开电源输出，RESET 时关闭。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_POWER_H
#define PRINTK_DEVICE_POWER_H

#include "robot_board.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */

/**
 * @brief 电源设备状态码。
 */
typedef enum
{
    PowerOk = 0,        /**< 操作成功。 */
    PowerArg = -1,      /**< 参数无效。 */
    PowerIo = -2,       /**< GPIO 操作失败。 */
    PowerNotReady = -3  /**< 电源设备尚未初始化。 */
} PowerStatus;

/**
 * @brief 可控电源通道。
 */
typedef enum
{
    PowerOut1 = 0, /**< XT30 VCC_OUT1。 */
    PowerOut2,     /**< XT30 VCC_OUT2。 */
    Power5v,       /**< 对外 5V。 */
    PowerCount     /**< 电源通道数量。 */
} PowerChannel;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化可控电源输出。
 *
 * 初始化会按默认状态设置三路电源：OUT1/OUT2 默认关闭，对外 5V 默认打开。
 *
 * @return 返回电源设备状态码。
 */
PowerStatus power_init(void);

/**
 * @brief 判断电源设备是否完成初始化。
 *
 * @return 已初始化返回 1，否则返回 0。
 */
uint8_t power_ready(void);

/**
 * @brief 设置指定电源通道使能状态。
 *
 * @param channel 电源通道。
 * @param enable 非 0 表示打开，0 表示关闭。
 * @return 返回电源设备状态码。
 */
PowerStatus power_set(PowerChannel channel, uint8_t enable);

/**
 * @brief 打开指定电源通道。
 *
 * @param channel 电源通道。
 * @return 返回电源设备状态码。
 */
PowerStatus power_on(PowerChannel channel);

/**
 * @brief 关闭指定电源通道。
 *
 * @param channel 电源通道。
 * @return 返回电源设备状态码。
 */
PowerStatus power_off(PowerChannel channel);

/**
 * @brief 翻转指定电源通道状态。
 *
 * @param channel 电源通道。
 * @return 返回电源设备状态码。
 */
PowerStatus power_toggle(PowerChannel channel);

/**
 * @brief 读取指定电源通道当前输出状态。
 *
 * @param channel 电源通道。
 * @return 打开返回 1，关闭或参数无效返回 0。
 */
uint8_t power_get(PowerChannel channel);

/**
 * @brief 同时设置三路电源输出状态。
 *
 * @param out1_enable VCC_OUT1 使能状态。
 * @param out2_enable VCC_OUT2 使能状态。
 * @param v5_enable 对外 5V 使能状态。
 * @return 返回电源设备状态码。
 */
PowerStatus power_set_all(uint8_t out1_enable, uint8_t out2_enable, uint8_t v5_enable);

/**
 * @brief 关闭全部可控电源输出。
 *
 * @return 返回电源设备状态码。
 */
PowerStatus power_all_off(void);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_POWER_H
