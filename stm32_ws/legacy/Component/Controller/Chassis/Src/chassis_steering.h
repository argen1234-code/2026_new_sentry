/**
 ******************************************************************************
 * @file    chassis_steering.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   舵轮底盘构型接口。
 *
 * @note    本构型只保留舵轮逆解、舵向角度闭环、驱动速度闭环和输出限幅。
 * @note    舵轮零位、几何尺寸、PID 和极性均由 ChassisCfg 或宏默认值提供。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_CHASSIS_STEERING_H
#define PRINTK_MECHANISM_CHASSIS_STEERING_H

#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 舵轮数量。 */
#define CHASSIS_STEERING_WHEEL_NUM 4U

/**
 * @brief 舵轮位置。
 */
typedef enum
{
    ChassisSteerLf = 0, /**< 左前轮。 */
    ChassisSteerLb = 1, /**< 左后轮。 */
    ChassisSteerRb = 2, /**< 右后轮。 */
    ChassisSteerRf = 3  /**< 右前轮。 */
} ChassisSteerPos;

/**
 * @brief 舵轮 PID 运行状态。
 */
typedef struct
{
    ChassisPidCfg cfg;  /**< PID 参数副本。 */
    float32_t set;      /**< 当前目标值。 */
    float32_t get;      /**< 当前反馈值。 */
    float32_t err;      /**< 当前误差。 */
    float32_t last_err; /**< 上一次误差。 */
    float32_t iout;     /**< 积分输出。 */
    float32_t out;      /**< PID 输出。 */
} ChassisSteeringPid;

/**
 * @brief 舵轮驱动轮状态。
 */
typedef struct
{
    float32_t speed;     /**< 当前轮速。 */
    float32_t speed_set; /**< 目标轮速。 */
    int16_t current;     /**< 输出电流。 */
    uint8_t online;      /**< 非 0 表示反馈在线。 */
} ChassisSteeringDrive;

/**
 * @brief 舵轮舵向状态。
 */
typedef struct
{
    float32_t angle;     /**< 当前舵向角。 */
    float32_t speed;     /**< 当前舵向速度。 */
    float32_t angle_set; /**< 目标舵向角。 */
    int16_t current;     /**< 输出电流。 */
    uint8_t online;      /**< 非 0 表示反馈在线。 */
} ChassisSteeringSteer;

/**
 * @brief 舵轮底盘私有上下文。
 */
typedef struct
{
    ChassisMode mode;                                           /**< 当前底盘模式。 */
    ChassisSteeringPid drive_pid[CHASSIS_STEERING_WHEEL_NUM];   /**< 驱动轮速度 PID。 */
    ChassisSteeringPid angle_pid[CHASSIS_STEERING_WHEEL_NUM];   /**< 舵向角度 PID。 */
    ChassisSteeringPid steer_pid[CHASSIS_STEERING_WHEEL_NUM];   /**< 舵向速度 PID。 */
    ChassisSteeringPid yaw_pid;                                 /**< yaw 跟随 PID。 */
    PowerCtrl power;                                             /**< 驱动电机功率预测控制器。 */
    ChassisSteeringDrive drive[CHASSIS_STEERING_WHEEL_NUM];     /**< 驱动轮状态。 */
    ChassisSteeringSteer steer[CHASSIS_STEERING_WHEEL_NUM];     /**< 舵向状态。 */
    int16_t drive_current[CHASSIS_STEERING_WHEEL_NUM];          /**< 驱动输出电流。 */
    int16_t steer_current[CHASSIS_STEERING_WHEEL_NUM];          /**< 舵向输出电流。 */
    uint8_t ready;                                              /**< 非 0 表示构型已初始化。 */
} ChassisSteeringCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化舵轮底盘。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisSteeringCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_steering_init(Chassis *chassis, void *impl);

/**
 * @brief 执行一次舵轮底盘周期。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisSteeringCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_steering_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);

/**
 * @brief 更新舵轮底盘反馈。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisSteeringCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_steering_update(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 执行舵轮底盘控制计算。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisSteeringCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_steering_control(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 生成舵轮底盘输出。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisSteeringCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_steering_output(const Chassis *chassis, void *impl, ChassisOutput *output);

/**
 * @brief 同步舵轮底盘目标。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisSteeringCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_steering_sync(Chassis *chassis, void *impl);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_CHASSIS_STEERING_H
