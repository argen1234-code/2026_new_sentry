/**
 ******************************************************************************
 * @file    motor_dji.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   DJI RM 电机 CAN 协议编解码接口。
 *
 * @note    本文件只处理 3508、2006、6020 等 DJI RM 电机的公共反馈帧和
 *          四路控制量发送帧，不绑定底盘、云台、摩擦轮等应用语义。
 *
 * @note    CAN 发送和接收分发由 BSP 或上层完成，本协议层不直接包含 HAL 头文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_MOTOR_DJI_H
#define PRINTK_DEVICE_MOTOR_DJI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */
#define DJI_MOTOR_FRAME_LEN             (8U)        /**< DJI RM 电机 CAN 帧长度。 */
#define DJI_MOTOR_GROUP_LOW_ID          (0x200U)    /**< DJI RM 低组控制帧 ID，对应反馈 ID 0x201 到 0x204。 */
#define DJI_MOTOR_GROUP_HIGH_ID         (0x1FFU)    /**< DJI RM 高组控制帧 ID，对应反馈 ID 0x205 到 0x208。 */
#define DJI_MOTOR_FB_ID_1               (0x201U)    /**< DJI RM 电机第一路反馈 ID。 */
#define DJI_MOTOR_FB_ID_8               (0x208U)    /**< DJI RM 电机最后一路反馈 ID。 */

/**
 * @brief DJI RM 电机协议状态。
 */
typedef enum
{
    DjiMotorOk = 0,                 /**< 操作成功。 */
    DjiMotorNull,                   /**< 输入指针为空。 */
    DjiMotorSize                    /**< 数据长度不是 8 字节。 */
} DjiMotorStatus;

/**
 * @brief DJI RM 电机反馈数据。
 */
typedef struct
{
    uint16_t ecd;                   /**< 当前机械角编码器值。 */
    uint16_t last_ecd;              /**< 上一次机械角编码器值。 */
    int16_t speed_rpm;              /**< 反馈转速，单位 rpm。 */
    int16_t given_current;          /**< 反馈电流或给定控制量，语义由具体电机定义。 */
    uint8_t temperature;            /**< 电机温度，单位摄氏度。 */
    uint32_t last_time;             /**< 最近一次反馈时间戳，单位由调用者定义。 */
} DjiMotorFb;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 解析 DJI RM 电机 8 字节反馈帧。
 *
 * @param fb 反馈输出结构体。
 * @param data CAN 数据区。
 * @param len CAN 数据长度，必须为 8。
 * @param now 当前时间戳，单位由调用者定义。
 * @return DjiMotorStatus 协议处理状态。
 */
DjiMotorStatus dji_motor_decode(DjiMotorFb *fb, const uint8_t *data, uint8_t len, uint32_t now);

/**
 * @brief 打包 DJI RM 四路控制量发送帧。
 *
 * @param data 8 字节输出缓冲区。
 * @param m1 第一路控制量。
 * @param m2 第二路控制量。
 * @param m3 第三路控制量。
 * @param m4 第四路控制量。
 * @return DjiMotorStatus 协议处理状态。
 */
DjiMotorStatus dji_motor_current(uint8_t *data, int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/**
 * @brief 根据 DJI RM 反馈 ID 计算 0 到 7 的槽位。
 *
 * @param std_id 标准 CAN 反馈 ID。
 * @param index 输出槽位。
 * @return DjiMotorStatus 协议处理状态。
 */
DjiMotorStatus dji_motor_index(uint16_t std_id, uint8_t *index);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_MOTOR_DJI_H
