/**
 ******************************************************************************
 * @file    motor_rs.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   灵足 / RS 电机私有模式命令实现。
 *
 * @note    命令字节序按旧参考工程 can_bsp.c 对齐。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "motor_rs.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/**
 * @brief RS 电机特殊命令前导字节。
 */
#define RS_MOTOR_CMD_PREFIX             (0xFFU)

/**
 * @brief RS 电机模式命令尾字节。
 */
#define RS_MOTOR_MODE_TAIL              (0xFDU)

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 填充 RS 特殊命令帧。
 *
 * @param data 目标帧缓冲区。
 * @param mode 模式字节。
 */
static void fill_cmd(uint8_t *data, uint8_t mode)
{
    data[0] = RS_MOTOR_CMD_PREFIX;
    data[1] = RS_MOTOR_CMD_PREFIX;
    data[2] = RS_MOTOR_CMD_PREFIX;
    data[3] = RS_MOTOR_CMD_PREFIX;
    data[4] = RS_MOTOR_CMD_PREFIX;
    data[5] = RS_MOTOR_CMD_PREFIX;
    data[6] = mode;
    data[7] = RS_MOTOR_MODE_TAIL;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
MitMotorStatus rs_motor_cmd(uint8_t *data, RsMotorCmdKind kind)
{
    if (data == 0) {
        return MitMotorNull;
    }

    switch (kind) {
    case RsMotorPreMode:
        fill_cmd(data, 0x00U);
        break;
    case RsMotorMitMode:
        fill_cmd(data, 0x02U);
        break;
    default:
        return MitMotorValue;
    }

    return MitMotorOk;
}
