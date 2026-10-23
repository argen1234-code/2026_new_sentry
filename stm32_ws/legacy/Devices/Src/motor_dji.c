/**
 ******************************************************************************
 * @file    motor_dji.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   DJI RM 电机 CAN 协议编解码实现。
 *
 * @note    本实现只做无状态帧解析和打包，电机实例表和总线绑定由上层维护。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "motor_dji.h"

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 将一个 int16 控制量按大端格式写入 DJI 控制帧。
 *
 * @param data 目标帧缓冲区。
 * @param offset 写入偏移。
 * @param value 控制量。
 */
static void put_i16(uint8_t *data, uint8_t offset, int16_t value)
{
    data[offset] = (uint8_t)(((uint16_t)value) >> 8);
    data[offset + 1U] = (uint8_t)((uint16_t)value);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
DjiMotorStatus dji_motor_decode(DjiMotorFb *fb, const uint8_t *data, uint8_t len, uint32_t now)
{
    if ((fb == 0) || (data == 0)) {
        return DjiMotorNull;
    }

    if (len != DJI_MOTOR_FRAME_LEN) {
        return DjiMotorSize;
    }

    fb->last_ecd = fb->ecd;
    fb->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    fb->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    fb->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    fb->temperature = data[6];
    fb->last_time = now;

    return DjiMotorOk;
}

DjiMotorStatus dji_motor_current(uint8_t *data, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    if (data == 0) {
        return DjiMotorNull;
    }

    put_i16(data, 0U, m1);
    put_i16(data, 2U, m2);
    put_i16(data, 4U, m3);
    put_i16(data, 6U, m4);

    return DjiMotorOk;
}

DjiMotorStatus dji_motor_index(uint16_t std_id, uint8_t *index)
{
    if (index == 0) {
        return DjiMotorNull;
    }

    if ((std_id < DJI_MOTOR_FB_ID_1) || (std_id > DJI_MOTOR_FB_ID_8)) {
        return DjiMotorSize;
    }

    *index = (uint8_t)(std_id - DJI_MOTOR_FB_ID_1);
    return DjiMotorOk;
}
