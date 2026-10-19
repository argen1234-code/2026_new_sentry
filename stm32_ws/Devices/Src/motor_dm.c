/**
 ******************************************************************************
 * @file    motor_dm.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   达妙电机型号参数和特殊命令实现。
 *
 * @note    特殊命令字节序按旧参考工程 can_bsp.c 和 dm4310_drv.c 对齐。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "motor_dm.h"

#include <math.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/**
 * @brief 达妙特殊命令固定前导字节。
 */
#define DM_MOTOR_CMD_PREFIX             (0xFFU)

/**
 * @brief 达妙模式切换命令尾字节。
 */
#define DM_MOTOR_MODE_TAIL              (0xFCU)

/**
 * @brief 达妙普通特殊命令尾字节，使能命令除外。
 */
#define DM_MOTOR_CTRL_TAIL              (0xFFU)

const MitMotorLimit DmMotorLimit4310 = {
    -12.5F, 12.5F,
    -30.0F, 30.0F,
    0.0F, 500.0F,
    0.0F, 5.0F,
    -20.0F, 20.0F
};

const MitMotorLimit DmMotorLimit6215 = {
    -12.5F, 12.5F,
    -200.0F, 200.0F,
    0.0F, 500.0F,
    0.0F, 5.0F,
    -10.0F, 10.0F
};

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 判断浮点数是否可用于达妙 float 直传模式。
 *
 * @param value 输入值。
 * @return uint8_t 1 表示可用，0 表示不可用。
 */
static uint8_t valid_f32(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

/**
 * @brief 按旧工程行为把 float 的内存字节复制到 CAN 帧。
 *
 * @param data 目标帧缓冲区。
 * @param offset 写入偏移。
 * @param value 输入浮点数。
 */
static void put_float(uint8_t *data, uint8_t offset, float value)
{
    union
    {
        float f32;
        uint8_t u8[4];
    } conv;

    conv.f32 = value;
    data[offset] = conv.u8[0];
    data[offset + 1U] = conv.u8[1];
    data[offset + 2U] = conv.u8[2];
    data[offset + 3U] = conv.u8[3];
}

/**
 * @brief 将 8 字节缓冲区填充为达妙特殊命令格式。
 *
 * @param data 目标帧缓冲区。
 * @param b6 第 7 字节。
 * @param b7 第 8 字节。
 */
static void fill_cmd(uint8_t *data, uint8_t b6, uint8_t b7)
{
    data[0] = DM_MOTOR_CMD_PREFIX;
    data[1] = DM_MOTOR_CMD_PREFIX;
    data[2] = DM_MOTOR_CMD_PREFIX;
    data[3] = DM_MOTOR_CMD_PREFIX;
    data[4] = DM_MOTOR_CMD_PREFIX;
    data[5] = DM_MOTOR_CMD_PREFIX;
    data[6] = b6;
    data[7] = b7;
}

const MitMotorLimit *dm_motor_limit(DmMotorModel model)
{
    if (model == DmMotor4310) {
        return &DmMotorLimit4310;
    }

    if (model == DmMotor6215) {
        return &DmMotorLimit6215;
    }

    return 0;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
MitMotorStatus dm_motor_mit(uint8_t *data, DmMotorModel model, const MitMotorCmd *cmd)
{
    const MitMotorLimit *limit;

    limit = dm_motor_limit(model);
    if (limit == 0) {
        return MitMotorValue;
    }

    return mit_motor_pack(data, limit, cmd);
}

MitMotorStatus dm_motor_decode(MitMotorFb *fb, DmMotorModel model, const uint8_t *data, uint8_t len)
{
    const MitMotorLimit *limit;

    limit = dm_motor_limit(model);
    if (limit == 0) {
        return MitMotorValue;
    }

    return mit_motor_decode(fb, limit, data, len);
}

MitMotorStatus dm_motor_cmd(uint8_t *data, DmMotorCmdKind kind)
{
    if (data == 0) {
        return MitMotorNull;
    }

    switch (kind) {
    case DmMotorEnable:
        fill_cmd(data, DM_MOTOR_CTRL_TAIL, 0xFCU);
        break;
    case DmMotorDisable:
        fill_cmd(data, DM_MOTOR_CTRL_TAIL, 0xFDU);
        break;
    case DmMotorSaveZero:
        fill_cmd(data, DM_MOTOR_CTRL_TAIL, 0xFEU);
        break;
    case DmMotorClearErr:
        fill_cmd(data, DM_MOTOR_CTRL_TAIL, 0xFBU);
        break;
    case DmMotorMitMode:
        fill_cmd(data, 0x00U, DM_MOTOR_MODE_TAIL);
        break;
    case DmMotorPosMode:
        fill_cmd(data, 0x01U, DM_MOTOR_MODE_TAIL);
        break;
    case DmMotorSpeedMode:
        fill_cmd(data, 0x02U, DM_MOTOR_MODE_TAIL);
        break;
    default:
        return MitMotorValue;
    }

    return MitMotorOk;
}

MitMotorStatus dm_motor_pos(uint8_t *data, float pos, float vel)
{
    if (data == 0) {
        return MitMotorNull;
    }

    if ((valid_f32(pos) == 0U) || (valid_f32(vel) == 0U)) {
        return MitMotorValue;
    }

    put_float(data, 0U, pos);
    put_float(data, 4U, vel);

    return MitMotorOk;
}

MitMotorStatus dm_motor_speed(uint8_t *data, float vel)
{
    uint8_t i;

    if (data == 0) {
        return MitMotorNull;
    }

    if (valid_f32(vel) == 0U) {
        return MitMotorValue;
    }

    put_float(data, 0U, vel);
    for (i = 4U; i < MIT_MOTOR_FRAME_LEN; i++) {
        data[i] = 0U;
    }

    return MitMotorOk;
}
