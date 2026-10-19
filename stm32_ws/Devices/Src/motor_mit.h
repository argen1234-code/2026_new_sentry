/**
 ******************************************************************************
 * @file    motor_mit.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   MIT-like 电机协议通用量化编解码接口。
 *
 * @note    本文件只描述 MIT 位置、速度、刚度、阻尼、力矩五元组的 8 字节
 *          打包方式，以及对应反馈帧解析方式。
 *
 * @note    不同型号的量程和特殊命令应放在型号驱动中，例如 DmMotor。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_MOTOR_MIT_H
#define PRINTK_DEVICE_MOTOR_MIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */
#define MIT_MOTOR_FRAME_LEN             (8U)        /**< MIT-like 协议 CAN 帧长度。 */
#define MIT_MOTOR_MODE_MIT              (0x000U)    /**< MIT 模式发送 ID 偏移。 */
#define MIT_MOTOR_MODE_POS              (0x100U)    /**< 位置速度模式发送 ID 偏移。 */
#define MIT_MOTOR_MODE_SPEED            (0x200U)    /**< 速度模式发送 ID 偏移。 */

/**
 * @brief MIT-like 协议状态。
 */
typedef enum
{
    MitMotorOk = 0,                 /**< 操作成功。 */
    MitMotorNull,                   /**< 输入指针为空。 */
    MitMotorSize,                   /**< 数据长度不是 8 字节。 */
    MitMotorValue                   /**< 参数范围或输入值非法。 */
} MitMotorStatus;

/**
 * @brief MIT-like 电机量化范围。
 */
typedef struct
{
    float p_min;                    /**< 位置最小值。 */
    float p_max;                    /**< 位置最大值。 */
    float v_min;                    /**< 速度最小值。 */
    float v_max;                    /**< 速度最大值。 */
    float kp_min;                   /**< 位置刚度最小值。 */
    float kp_max;                   /**< 位置刚度最大值。 */
    float kd_min;                   /**< 速度阻尼最小值。 */
    float kd_max;                   /**< 速度阻尼最大值。 */
    float t_min;                    /**< 力矩最小值。 */
    float t_max;                    /**< 力矩最大值。 */
} MitMotorLimit;

/**
 * @brief MIT-like 电机控制命令。
 */
typedef struct
{
    float pos;                      /**< 目标位置。 */
    float vel;                      /**< 目标速度。 */
    float kp;                       /**< 位置刚度。 */
    float kd;                       /**< 速度阻尼。 */
    float tor;                      /**< 前馈力矩。 */
} MitMotorCmd;

/**
 * @brief MIT-like 电机反馈数据。
 */
typedef struct
{
    uint8_t id;                     /**< 反馈帧中的电机 ID 低 4 位。 */
    uint8_t state;                  /**< 反馈帧中的状态高 4 位。 */
    uint16_t p_int;                 /**< 位置量化原始值。 */
    uint16_t v_int;                 /**< 速度量化原始值。 */
    uint16_t t_int;                 /**< 力矩量化原始值。 */
    float pos;                      /**< 解算后位置。 */
    float vel;                      /**< 解算后速度。 */
    float tor;                      /**< 解算后力矩。 */
    float t_mos;                    /**< MOS 温度，单位由电机协议定义。 */
    float t_motor;                  /**< 电机线圈或本体温度，单位由电机协议定义。 */
} MitMotorFb;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 将浮点数按给定范围量化为无符号整数。
 *
 * @param value 输入浮点数，函数内部会限幅。
 * @param min 最小值。
 * @param max 最大值。
 * @param bits 量化位宽，支持 1 到 31。
 * @param out 输出无符号整数。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus mit_motor_to_uint(float value, float min, float max, uint8_t bits, uint32_t *out);

/**
 * @brief 将无符号整数按给定范围反量化为浮点数。
 *
 * @param value 输入无符号整数。
 * @param min 最小值。
 * @param max 最大值。
 * @param bits 量化位宽，支持 1 到 31。
 * @param out 输出浮点数。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus mit_motor_to_float(uint32_t value, float min, float max, uint8_t bits, float *out);

/**
 * @brief 打包 MIT-like 五元组控制帧。
 *
 * @param data 8 字节输出缓冲区。
 * @param limit 型号量化范围。
 * @param cmd 控制命令。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus mit_motor_pack(uint8_t *data, const MitMotorLimit *limit, const MitMotorCmd *cmd);

/**
 * @brief 解析 MIT-like 电机反馈帧。
 *
 * @param fb 反馈输出结构体。
 * @param limit 型号量化范围。
 * @param data CAN 数据区。
 * @param len CAN 数据长度，必须为 8。
 * @return MitMotorStatus 协议处理状态。
 */
MitMotorStatus mit_motor_decode(MitMotorFb *fb, const MitMotorLimit *limit, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_MOTOR_MIT_H
