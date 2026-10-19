/**
 ******************************************************************************
 * @file    motor.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-04-29
 * @brief   DM-MC02 电机设备接口。
 *
 * @note    本文件负责按板级配置使用 FDCAN 总线、反馈 ID 和发送 ID。
 *          电机协议编解码仍由 Devices/Src 下的协议层负责。
 *
 * @note    本层不实现控制算法、不做速度滤波，也不恢复旧版 can_bsp 的全局可写
 *          电机状态数组。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_MOTOR_H
#define PRINTK_DEVICE_MOTOR_H

#include "motor_dji.h"
#include "motor_dm.h"
#include "motor_mit.h"
#include "motor_rs.h"
#include "bsp_fdcan.h"
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
#define MOTOR_DJI_FB_FIRST          DJI_MOTOR_FB_ID_1          /**< 板端 DJI 反馈起始 ID。 */
#define MOTOR_DJI_FB_LAST           DJI_MOTOR_FB_ID_8          /**< 板端 DJI 反馈结束 ID。 */
#define MOTOR_MIT_FB_FIRST          (0x51U)                    /**< 板端 MIT 反馈起始 ID。 */
#define MOTOR_MIT_FB_LAST           (0x58U)                    /**< 板端 MIT 反馈结束 ID。 */

/**
 * @brief 电机设备状态。
 */
typedef enum
{
    /** 操作成功。 */
    MotorOk = 0,
    /** 输入指针为空。 */
    MotorNull,
    /** 参数非法。 */
    MotorArg,
    /** HAL 或 FDCAN 操作失败。 */
    MotorHal,
    /** 当前槽位还没有收到有效反馈。 */
    MotorNoData,
    /** 协议层打包或解包失败。 */
    MotorCodec
} motor_status_t;

/**
 * @brief MIT 反馈快照。
 */
typedef struct
{
    /** MIT 协议反馈数据。 */
    MitMotorFb fb;
    /** 最近一次反馈时间戳，单位为 HAL tick。 */
    uint32_t last_time;
} motor_mit_fb_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 初始化板端电机绑定。
 *
 * 本函数会清空内部反馈状态，设置默认 MIT 量程，并绑定 FDCAN RX 订阅。
 * 应在 FDCAN start 前调用。
 *
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_init(void);

/**
 * @brief 为某个 MIT 反馈槽设置通用量程。
 *
 * @param bus FDCAN 总线标识。
 * @param rx_id MIT 反馈 ID，范围 0x51 到 0x58。
 * @param limit MIT 量化范围。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_mit_bind(bsp_fdcan_id_t bus, uint16_t rx_id, const MitMotorLimit *limit);

/**
 * @brief 为某个 MIT 反馈槽设置达妙型号量程。
 *
 * @param bus FDCAN 总线标识。
 * @param rx_id MIT 反馈 ID，范围 0x51 到 0x58。
 * @param model 达妙电机型号。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_dm_bind(bsp_fdcan_id_t bus, uint16_t rx_id, DmMotorModel model);

/**
 * @brief 获取 DJI 电机反馈快照。
 *
 * @param bus FDCAN 总线标识。
 * @param rx_id DJI 反馈 ID，范围 0x201 到 0x208。
 * @param out 反馈输出结构体。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_dji_get(bsp_fdcan_id_t bus, uint16_t rx_id, DjiMotorFb *out);

/**
 * @brief 获取 MIT 电机反馈快照。
 *
 * @param bus FDCAN 总线标识。
 * @param rx_id MIT 反馈 ID，范围 0x51 到 0x58。
 * @param out 反馈输出结构体。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_mit_get(bsp_fdcan_id_t bus, uint16_t rx_id, motor_mit_fb_t *out);

/**
 * @brief 发送 DJI 四路控制帧。
 *
 * @param bus FDCAN 总线标识。
 * @param tx_id 发送 ID，通常为 0x200 或 0x1FF。
 * @param m1 第一路控制量。
 * @param m2 第二路控制量。
 * @param m3 第三路控制量。
 * @param m4 第四路控制量。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_dji_send(bsp_fdcan_id_t bus,
                                      uint16_t tx_id,
                                      int16_t m1,
                                      int16_t m2,
                                      int16_t m3,
                                      int16_t m4);

/**
 * @brief 发送达妙 MIT 模式控制帧。
 *
 * @param bus FDCAN 总线标识。
 * @param id 电机接收基础 ID。
 * @param model 达妙电机型号。
 * @param cmd MIT 控制命令。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_dm_mit(bsp_fdcan_id_t bus, uint16_t id, DmMotorModel model, const MitMotorCmd *cmd);

/**
 * @brief 发送达妙特殊命令帧。
 *
 * @param bus FDCAN 总线标识。
 * @param id 电机接收 ID。
 * @param kind 达妙特殊命令。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_dm_cmd(bsp_fdcan_id_t bus, uint16_t id, DmMotorCmdKind kind);

/**
 * @brief 发送 RS 电机特殊命令帧。
 *
 * @param bus FDCAN 总线标识。
 * @param id 电机接收 ID。
 * @param kind RS 特殊命令。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_rs_cmd(bsp_fdcan_id_t bus, uint16_t id, RsMotorCmdKind kind);

/**
 * @brief 发送底盘四路 DJI 电机控制量。
 *
 * @param m1 第一路控制量。
 * @param m2 第二路控制量。
 * @param m3 第三路控制量。
 * @param m4 第四路控制量。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_chassis(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/**
 * @brief 发送底盘舵向四路 DJI 电机控制量。
 *
 * @param m1 第一路控制量。
 * @param m2 第二路控制量。
 * @param m3 第三路控制量。
 * @param m4 第四路控制量。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_chassis_steer(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/**
 * @brief 发送云台 yaw、pitch、trigger 和预留控制量。
 *
 * @param yaw yaw 电机控制量。
 * @param pitch pitch 电机控制量。
 * @param trigger 拨弹电机控制量。
 * @param rev 预留控制量。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_gimbal(int16_t yaw, int16_t pitch, int16_t trigger, int16_t rev);

/**
 * @brief 更新并发送云台 yaw/pitch 槽位。
 *
 * trigger 槽位会沿用最近一次 motor_gimbal_trigger() 写入的值，避免发射 app
 * 与云台 app 共同使用 DJI 0x1FF 发送组时互相清零。
 *
 * @param yaw yaw 电机控制量。
 * @param pitch pitch 电机控制量。
 * @param rev 预留控制量。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_gimbal_yaw_pitch(int16_t yaw, int16_t pitch, int16_t rev);

/**
 * @brief 更新并发送云台 trigger 槽位。
 *
 * yaw/pitch 槽位会沿用最近一次 motor_gimbal_yaw_pitch() 写入的值。
 *
 * @param trigger 拨弹电机控制量。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_gimbal_trigger(int16_t trigger);

/**
 * @brief 发送摩擦轮四路 DJI 电机控制量。
 *
 * @param m1 第一路控制量。
 * @param m2 第二路控制量。
 * @param m3 第三路控制量。
 * @param m4 第四路控制量。
 * @return MotorOk 表示成功，否则返回错误原因。
 */
motor_status_t motor_fric(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_MOTOR_H
