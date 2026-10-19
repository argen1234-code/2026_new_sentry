/**
 ******************************************************************************
 * @file    motor_mit.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   MIT-like 电机协议通用量化编解码实现。
 *
 * @note    旧参考实现里速度低通属于信号处理，本文件只还原协议解析，不做滤波。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "motor_mit.h"

#include <math.h>

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
 * @brief 判断浮点数是否可用于协议量化。
 *
 * @param value 输入值。
 * @return uint8_t 1 表示可用，0 表示不可用。
 */
static uint8_t valid_f32(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

/**
 * @brief 将浮点数限制到闭区间。
 *
 * @param value 输入值。
 * @param min 最小值。
 * @param max 最大值。
 * @return float 限幅后的值。
 */
static float clamp_f32(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

/**
 * @brief 检查量化范围是否有效。
 *
 * @param min 最小值。
 * @param max 最大值。
 * @return uint8_t 1 表示有效，0 表示无效。
 */
static uint8_t valid_range(float min, float max)
{
    if ((valid_f32(min) == 0U) || (valid_f32(max) == 0U)) {
        return 0U;
    }

    return (max > min) ? 1U : 0U;
}

/**
 * @brief 检查 MIT-like 型号量程是否完整有效。
 *
 * @param limit 型号量化范围。
 * @return uint8_t 1 表示有效，0 表示无效。
 */
static uint8_t valid_limit(const MitMotorLimit *limit)
{
    if (limit == 0) {
        return 0U;
    }

    if ((valid_range(limit->p_min, limit->p_max) == 0U) ||
        (valid_range(limit->v_min, limit->v_max) == 0U) ||
        (valid_range(limit->kp_min, limit->kp_max) == 0U) ||
        (valid_range(limit->kd_min, limit->kd_max) == 0U) ||
        (valid_range(limit->t_min, limit->t_max) == 0U)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 检查量化位宽是否能安全生成掩码。
 *
 * @param bits 量化位宽。
 * @return uint8_t 1 表示有效，0 表示无效。
 */
static uint8_t valid_bits(uint8_t bits)
{
    return ((bits > 0U) && (bits < 32U)) ? 1U : 0U;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
MitMotorStatus mit_motor_to_uint(float value, float min, float max, uint8_t bits, uint32_t *out)
{
    float span;
    float scaled;
    uint32_t max_int;

    if (out == 0) {
        return MitMotorNull;
    }

    if ((valid_f32(value) == 0U) || (valid_range(min, max) == 0U) || (valid_bits(bits) == 0U)) {
        return MitMotorValue;
    }

    value = clamp_f32(value, min, max);
    span = max - min;
    max_int = (uint32_t)((1UL << bits) - 1UL);
    scaled = ((value - min) * (float)max_int) / span;

    if (scaled < 0.0F) {
        scaled = 0.0F;
    }

    if (scaled > (float)max_int) {
        scaled = (float)max_int;
    }

    /*
     * 旧工程使用强制类型转换得到量化值，也就是向零截断。
     * 这里只增加输入限幅，不改变原协议量化语义。
     */
    *out = (uint32_t)scaled;
    return MitMotorOk;
}

MitMotorStatus mit_motor_to_float(uint32_t value, float min, float max, uint8_t bits, float *out)
{
    uint32_t max_int;

    if (out == 0) {
        return MitMotorNull;
    }

    if ((valid_range(min, max) == 0U) || (valid_bits(bits) == 0U)) {
        return MitMotorValue;
    }

    max_int = (uint32_t)((1UL << bits) - 1UL);
    if (value > max_int) {
        value = max_int;
    }

    *out = (((float)value) * (max - min) / (float)max_int) + min;
    return MitMotorOk;
}

MitMotorStatus mit_motor_pack(uint8_t *data, const MitMotorLimit *limit, const MitMotorCmd *cmd)
{
    uint32_t pos;
    uint32_t vel;
    uint32_t kp;
    uint32_t kd;
    uint32_t tor;

    if ((data == 0) || (cmd == 0)) {
        return MitMotorNull;
    }

    if (valid_limit(limit) == 0U) {
        return MitMotorValue;
    }

    if ((mit_motor_to_uint(cmd->pos, limit->p_min, limit->p_max, 16U, &pos) != MitMotorOk) ||
        (mit_motor_to_uint(cmd->vel, limit->v_min, limit->v_max, 12U, &vel) != MitMotorOk) ||
        (mit_motor_to_uint(cmd->kp, limit->kp_min, limit->kp_max, 12U, &kp) != MitMotorOk) ||
        (mit_motor_to_uint(cmd->kd, limit->kd_min, limit->kd_max, 12U, &kd) != MitMotorOk) ||
        (mit_motor_to_uint(cmd->tor, limit->t_min, limit->t_max, 12U, &tor) != MitMotorOk)) {
        return MitMotorValue;
    }

    data[0] = (uint8_t)(pos >> 8);
    data[1] = (uint8_t)pos;
    data[2] = (uint8_t)(vel >> 4);
    data[3] = (uint8_t)(((vel & 0x0FU) << 4) | (kp >> 8));
    data[4] = (uint8_t)kp;
    data[5] = (uint8_t)(kd >> 4);
    data[6] = (uint8_t)(((kd & 0x0FU) << 4) | (tor >> 8));
    data[7] = (uint8_t)tor;

    return MitMotorOk;
}

MitMotorStatus mit_motor_decode(MitMotorFb *fb, const MitMotorLimit *limit, const uint8_t *data, uint8_t len)
{
    if ((fb == 0) || (data == 0)) {
        return MitMotorNull;
    }

    if (len != MIT_MOTOR_FRAME_LEN) {
        return MitMotorSize;
    }

    if (valid_limit(limit) == 0U) {
        return MitMotorValue;
    }

    fb->id = (uint8_t)(data[0] & 0x0FU);
    fb->state = (uint8_t)(data[0] >> 4);
    fb->p_int = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    fb->v_int = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    fb->t_int = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);

    if ((mit_motor_to_float(fb->p_int, limit->p_min, limit->p_max, 16U, &fb->pos) != MitMotorOk) ||
        (mit_motor_to_float(fb->v_int, limit->v_min, limit->v_max, 12U, &fb->vel) != MitMotorOk) ||
        (mit_motor_to_float(fb->t_int, limit->t_min, limit->t_max, 12U, &fb->tor) != MitMotorOk)) {
        return MitMotorValue;
    }

    fb->t_mos = (float)data[6];
    fb->t_motor = (float)data[7];

    return MitMotorOk;
}
