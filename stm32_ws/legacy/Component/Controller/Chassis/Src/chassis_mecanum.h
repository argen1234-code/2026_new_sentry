/**
 ******************************************************************************
 * @file    chassis_mecanum.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   麦克纳姆轮底盘构型接口。
 *
 * @note    本构型只保留机构运动学、速度闭环和输出限幅。
 * @note    所有命令、反馈、功率预算和时间戳均通过 ChassisInput 注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_CHASSIS_MECANUM_H
#define PRINTK_MECHANISM_CHASSIS_MECANUM_H

#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 麦克纳姆轮数量。 */
#define CHASSIS_MECANUM_WHEEL_NUM 4U

/**
 * @brief 麦轮 PID 运行状态。
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
} ChassisMecanumPid;

/**
 * @brief 麦轮单轮状态。
 */
typedef struct
{
    float32_t speed;     /**< 当前轮速。 */
    float32_t speed_set; /**< 目标轮速。 */
    int16_t current;     /**< 输出电流。 */
    uint8_t online;      /**< 非 0 表示反馈在线。 */
} ChassisMecanumWheel;

/**
 * @brief 麦克纳姆轮底盘私有上下文。
 */
typedef struct
{
    ChassisMode mode;                                      /**< 当前底盘模式。 */
    ChassisMecanumPid drive_pid[CHASSIS_MECANUM_WHEEL_NUM];/**< 驱动轮速度 PID。 */
    ChassisMecanumPid yaw_pid;                             /**< yaw 跟随 PID。 */
    PowerCtrl power;                                       /**< 驱动电机功率预测控制器。 */
    ChassisMecanumWheel wheel[CHASSIS_MECANUM_WHEEL_NUM];  /**< 驱动轮状态。 */
    int16_t current[CHASSIS_MECANUM_WHEEL_NUM];            /**< 输出电流。 */
    float32_t smooth_vx;                                   /**< 平滑后的纵向速度。 */
    float32_t smooth_vy;                                   /**< 平滑后的横向速度。 */
    float32_t smooth_wz;                                   /**< 平滑后的角速度。 */
    uint32_t smooth_tick_ms;                               /**< 上次平滑时间，单位 ms。 */
    uint8_t ready;                                         /**< 非 0 表示构型已初始化。 */
} ChassisMecanumCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化麦克纳姆轮底盘。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisMecanumCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_mecanum_init(Chassis *chassis, void *impl);

/**
 * @brief 执行一次麦克纳姆轮底盘周期。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisMecanumCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_mecanum_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);

/**
 * @brief 更新麦克纳姆轮底盘反馈。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisMecanumCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_mecanum_update(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 执行麦克纳姆轮底盘控制计算。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisMecanumCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_mecanum_control(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 生成麦克纳姆轮底盘输出。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisMecanumCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_mecanum_output(const Chassis *chassis, void *impl, ChassisOutput *output);

/**
 * @brief 同步麦克纳姆轮底盘目标。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisMecanumCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_mecanum_sync(Chassis *chassis, void *impl);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_CHASSIS_MECANUM_H
