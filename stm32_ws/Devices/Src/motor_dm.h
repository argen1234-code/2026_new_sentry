/**
 ******************************************************************************
 * @file    motor_dm.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   达妙电机型号参数和特殊命令接口。
 *
 * @note    达妙 MIT 控制帧复用 MitMotor 通用编解码，本文件只提供型号量程、
 *          模式偏移和厂商特殊命令帧。
 *
 * @note    CAN 发送由 BSP 或上层完成，本文件仅负责生成 8 字节数据。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_MOTOR_DM_H
#define PRINTK_DEVICE_MOTOR_DM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_mit.h"

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */

/**
 * @brief 达妙电机型号。
 */
typedef enum
{
    DmMotor4310 = 0,                /**< 达妙 DM4310。 */
    DmMotor6215                     /**< 达妙 DM6215。 */
} DmMotorModel;

/**
 * @brief 达妙特殊命令类型。
 */
typedef enum
{
    DmMotorEnable = 0,              /**< 电机使能命令。 */
    DmMotorDisable,                 /**< 电机失能命令。 */
    DmMotorSaveZero,                /**< 保存当前位置为零点。 */
    DmMotorClearErr,                /**< 清除电机错误。 */
    DmMotorMitMode,                 /**< 切换到 MIT 模式。 */
    DmMotorPosMode,                 /**< 切换到位置速度模式。 */
    DmMotorSpeedMode                /**< 切换到速度模式。 */
} DmMotorCmdKind;

extern const MitMotorLimit DmMotorLimit4310;        /**< DM4310 MIT 模式量化范围。 */

extern const MitMotorLimit DmMotorLimit6215;        /**< DM6215 MIT 模式量化范围。 */

/**
 * @brief 获取达妙型号对应的 MIT 量化范围。
 *
 * @param model 达妙电机型号。
 * @return const MitMotorLimit* 成功返回量化范围，失败返回空指针。
 */
const MitMotorLimit *dm_motor_limit(DmMotorModel model);

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 打包达妙 MIT 模式控制帧。
 *
 * @param data 8 字节输出缓冲区。
 * @param model 达妙电机型号。
 * @param cmd MIT 控制命令。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus dm_motor_mit(uint8_t *data, DmMotorModel model, const MitMotorCmd *cmd);

/**
 * @brief 解析达妙 MIT-like 反馈帧。
 *
 * @param fb 反馈输出结构体。
 * @param model 达妙电机型号。
 * @param data CAN 数据区。
 * @param len CAN 数据长度，必须为 8。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus dm_motor_decode(MitMotorFb *fb, DmMotorModel model, const uint8_t *data, uint8_t len);

/**
 * @brief 生成达妙特殊命令帧。
 *
 * @param data 8 字节输出缓冲区。
 * @param kind 特殊命令类型。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus dm_motor_cmd(uint8_t *data, DmMotorCmdKind kind);

/**
 * @brief 生成达妙位置速度模式数据帧。
 *
 * @param data 8 字节输出缓冲区。
 * @param pos 目标位置，按达妙原协议以 float 小端发送。
 * @param vel 目标速度，按达妙原协议以 float 小端发送。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus dm_motor_pos(uint8_t *data, float pos, float vel);

/**
 * @brief 生成达妙速度模式数据帧。
 *
 * @param data 8 字节输出缓冲区。
 * @param vel 目标速度，按达妙原协议以 float 小端发送。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus dm_motor_speed(uint8_t *data, float vel);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_MOTOR_DM_H
