/**
 ******************************************************************************
 * @file    gimbal.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   云台机构公共接口。
 *
 * @note    本文件定义不同云台构型共享的输入、输出、配置和私有上下文绑定语言。
 * @note    具体构型由 GIMBAL_TYPE 宏在编译期选中，不支持运行时切换机械构型。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_GIMBAL_H
#define PRINTK_MECHANISM_GIMBAL_H

#include "arm_math.h"
#include "robot.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief yaw/pitch 直驱云台构型宏。 */
#define GIMBAL_TYPE_DIRECT_YAW_PITCH 1U

/** @brief 双 yaw/pitch 云台构型宏。 */
#define GIMBAL_TYPE_DOUBLE_YAW_PITCH 2U

/** @brief yaw/pitch 联动云台构型宏。 */
#define GIMBAL_TYPE_LINKAGE_YAW_PITCH 3U

/** @brief yaw 角度环纠偏速度默认限幅，单位 rad/s。 */
#ifndef GIMBAL_YAW_CORR_RATE_MAX
#define GIMBAL_YAW_CORR_RATE_MAX 2.0F
#endif

/** @brief 默认云台构型宏，工程可在编译选项或配置头中覆盖。 */
#ifndef GIMBAL_TYPE
#define GIMBAL_TYPE GIMBAL_TYPE_DIRECT_YAW_PITCH
#endif

/** @brief 云台公共轴数量。 */
#define GIMBAL_AXIS_NUM 3U

/**
 * @brief 云台执行状态。
 */
typedef enum
{
    GimbalOk = 0,     /**< 执行成功。 */
    GimbalNull = 1,   /**< 输入指针为空。 */
    GimbalArg = 2,    /**< 输入参数非法。 */
    GimbalState = 3,  /**< 当前上下文状态不允许执行。 */
    GimbalValue = 4   /**< 输入数值非法。 */
} GimbalStatus;

/**
 * @brief 云台构型。
 */
typedef enum
{
    GimbalDirectYawPitch = GIMBAL_TYPE_DIRECT_YAW_PITCH, /**< yaw/pitch 直驱构型。 */
    GimbalDoubleYawPitch = GIMBAL_TYPE_DOUBLE_YAW_PITCH, /**< 双 yaw/pitch 构型。 */
    GimbalLinkageYawPitch = GIMBAL_TYPE_LINKAGE_YAW_PITCH /**< yaw/pitch 联动构型。 */
} GimbalType;

/**
 * @brief 云台轴标识。
 */
typedef enum
{
    GimbalAxisYaw = 0,     /**< yaw 轴。 */
    GimbalAxisPitch = 1,   /**< pitch 轴。 */
    GimbalAxisBigYaw = 2   /**< 大 yaw 或构型扩展轴。 */
} GimbalAxis;

/**
 * @brief 云台行为模式。
 */
typedef enum
{
    GimbalModeZero = 0,   /**< 无力模式，输出为零。 */
    GimbalModeRaw = 1,    /**< 原始电流或原始执行量模式。 */
    GimbalModeGyro = 2,   /**< 陀螺仪绝对角模式。 */
    GimbalModeEncode = 3, /**< 编码器相对角模式。 */
    GimbalModeCali = 4    /**< 机械限位校准模式。 */
} GimbalMode;

/**
 * @brief 云台 PID 参数。
 */
typedef struct
{
    float32_t kp;       /**< 比例增益。 */
    float32_t ki;       /**< 积分增益。 */
    float32_t kd;       /**< 微分增益。 */
    float32_t max_out;  /**< 输出绝对值限幅。 */
    float32_t max_iout; /**< 积分输出绝对值限幅。 */
} GimbalPidCfg;

/**
 * @brief 云台电机反馈。
 */
typedef struct
{
    uint16_t ecd;       /**< 编码器原始值。 */
    int16_t speed_rpm;  /**< 转速反馈，单位 rpm。 */
    int16_t current;    /**< 电流或力矩电流反馈。 */
    uint8_t online;     /**< 非 0 表示反馈在线。 */
} GimbalMotorFb;

/**
 * @brief MIT 执行器反馈。
 */
typedef struct
{
    float32_t pos;  /**< 位置反馈，单位 rad。 */
    float32_t vel;  /**< 速度反馈，单位 rad/s。 */
    float32_t tor;  /**< 力矩反馈，单位 N*m。 */
    uint8_t online; /**< 非 0 表示反馈在线。 */
} GimbalMitFb;

/**
 * @brief MIT 执行器命令。
 */
typedef struct
{
    float32_t pos;  /**< 位置命令，单位 rad。 */
    float32_t vel;  /**< 速度命令，单位 rad/s。 */
    float32_t tor;  /**< 力矩命令，单位 N*m。 */
    float32_t kp;   /**< MIT 位置刚度。 */
    float32_t kd;   /**< MIT 速度阻尼。 */
    uint8_t enable; /**< 非 0 表示允许发送该命令。 */
} GimbalMitCmd;

/**
 * @brief 云台上层控制命令。
 */
typedef struct
{
    float32_t yaw;   /**< yaw 控制增量或目标输入，单位由 mode 决定。 */
    float32_t pitch; /**< pitch 控制增量或目标输入，单位由 mode 决定。 */
    GimbalMode mode; /**< 上层请求的云台模式。 */
    uint8_t online;  /**< 非 0 表示命令源在线。 */
} GimbalCmd;

/**
 * @brief 云台周期输入。
 */
typedef struct
{
    GimbalCmd cmd;          /**< 上层控制命令。 */
    GimbalMotorFb yaw_fb;   /**< yaw 电机反馈。 */
    GimbalMotorFb pitch_fb; /**< pitch 电机反馈。 */
    GimbalMitFb big_yaw_fb; /**< 大 yaw 或构型扩展轴反馈。 */
    float32_t ins_yaw;      /**< INS yaw 连续角，单位 rad。 */
    float32_t ins_pitch;    /**< INS pitch 角，单位 rad。 */
    float32_t gyro_x;       /**< IMU X 轴角速度，单位 rad/s。 */
    float32_t gyro_y;       /**< IMU Y 轴角速度，单位 rad/s。 */
    float32_t gyro_z;       /**< IMU Z 轴角速度，单位 rad/s。 */
    uint32_t tick_ms;       /**< 当前系统 tick，单位 ms。 */
    uint32_t time_us;       /**< 当前采样时间戳低 32 位，单位 us。 */
    uint8_t ins_ready;      /**< 非 0 表示 INS 已可用。 */
} GimbalInput;

/**
 * @brief 云台周期输出。
 */
typedef struct
{
    int16_t yaw_current;      /**< yaw 电机发送电流。 */
    int16_t pitch_current;    /**< pitch 电机发送电流。 */
    int16_t aux_current;      /**< 辅助电机或保留电流。 */
    GimbalMitCmd big_yaw_cmd; /**< 大 yaw 或构型扩展轴命令。 */
    GimbalMode mode;          /**< 本周期实际输出模式。 */
    uint8_t send_enable;      /**< 非 0 表示输出允许发送。 */
} GimbalOutput;

/**
 * @brief 云台校准结果。
 */
typedef struct
{
    uint16_t yaw_offset;       /**< yaw 中值编码器。 */
    uint16_t pitch_offset;     /**< pitch 中值编码器。 */
    float32_t yaw_max;         /**< yaw 最大相对角，单位 rad。 */
    float32_t yaw_min;         /**< yaw 最小相对角，单位 rad。 */
    float32_t pitch_max;       /**< pitch 最大相对角，单位 rad。 */
    float32_t pitch_min;       /**< pitch 最小相对角，单位 rad。 */
    int8_t yaw_polarity;       /**< yaw 输出电流极性。 */
    int8_t pitch_polarity;     /**< pitch 输出电流极性。 */
} GimbalCali;

/**
 * @brief 云台公共配置。
 */
typedef struct
{
    GimbalType type;                /**< 当前构型类型。 */
    GimbalPidCfg yaw_abs_pid;       /**< yaw 绝对角 PID。 */
    GimbalPidCfg yaw_rel_pid;       /**< yaw 相对角 PID。 */
    GimbalPidCfg yaw_speed_pid;     /**< yaw 速度 PID。 */
    GimbalPidCfg pitch_abs_pid;     /**< pitch 绝对角 PID。 */
    GimbalPidCfg pitch_rel_pid;     /**< pitch 相对角 PID。 */
    GimbalPidCfg pitch_speed_pid;   /**< pitch 速度 PID。 */
    float32_t ecd_to_rad;           /**< 编码器 tick 到 rad 的换算系数。 */
    int32_t ecd_range;              /**< 编码器计数范围。 */
    int32_t half_ecd_range;         /**< 编码器半圈计数。 */
    uint8_t yaw_turn;               /**< 非 0 表示 yaw 编码和输出反向。 */
    uint8_t pitch_turn;             /**< 非 0 表示 pitch 编码和输出反向。 */
    int8_t yaw_current_polarity;    /**< yaw 电流发送极性。 */
    int8_t pitch_current_polarity;  /**< pitch 电流发送极性。 */
    uint16_t period_ms;             /**< 控制周期，单位 ms。 */
} GimbalCfg;

/**
 * @brief 云台公共上下文。
 */
typedef struct Gimbal
{
    GimbalCfg cfg;       /**< 公共配置副本。 */
    void *impl;          /**< 当前编译期构型的私有上下文。 */
    uint8_t initialized; /**< 非 0 表示公共上下文已初始化。 */
} Gimbal;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化当前编译期构型的云台上下文。
 *
 * @param gimbal 云台公共上下文。
 * @param cfg 公共配置。
 * @param impl 当前构型私有上下文，可由具体构型决定是否允许为空。
 * @return 执行状态。
 */
GimbalStatus gimbal_init(Gimbal *gimbal, const GimbalCfg *cfg, void *impl);

/**
 * @brief 执行一次完整云台周期。
 *
 * @param gimbal 云台公共上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_step(Gimbal *gimbal, const GimbalInput *input, GimbalOutput *output);

/**
 * @brief 更新云台反馈。
 *
 * @param gimbal 云台公共上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_update(Gimbal *gimbal, const GimbalInput *input);

/**
 * @brief 执行云台控制计算。
 *
 * @param gimbal 云台公共上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
GimbalStatus gimbal_control(Gimbal *gimbal, const GimbalInput *input);

/**
 * @brief 生成云台输出。
 *
 * @param gimbal 云台公共上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
GimbalStatus gimbal_output(const Gimbal *gimbal, GimbalOutput *output);

/**
 * @brief 将控制目标同步到当前反馈状态。
 *
 * @param gimbal 云台公共上下文。
 * @return 执行状态。
 */
GimbalStatus gimbal_sync(Gimbal *gimbal);

/**
 * @brief 写入云台校准结果。
 *
 * @param gimbal 云台公共上下文。
 * @param cali 校准结果。
 * @return 执行状态。
 */
GimbalStatus gimbal_set_cali(Gimbal *gimbal, const GimbalCali *cali);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_GIMBAL_H
