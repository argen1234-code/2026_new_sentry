/**
 ******************************************************************************
 * @file    chassis_omni.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   全向轮底盘构型接口。
 *
 * @note    本构型只保留全向轮运动学、速度闭环和输出限幅。
 * @note    所有命令、反馈、功率预算和时间戳均通过 ChassisInput 注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_CHASSIS_OMNI_H
#define PRINTK_MECHANISM_CHASSIS_OMNI_H

#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 全向轮数量。 */
#define CHASSIS_OMNI_WHEEL_NUM 4U

/**
 * @brief 全向轮 PID 运行状态。
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
} ChassisOmniPid;

/**
 * @brief 全向轮单轮状态。
 */
typedef struct
{
    float32_t speed;     /**< 当前轮速。 */
    float32_t speed_set; /**< 目标轮速。 */
    int16_t current;     /**< 输出电流。 */
    uint8_t online;      /**< 非 0 表示反馈在线。 */
} ChassisOmniWheel;

/**
 * @brief 全向轮底盘私有上下文。
 */
typedef struct
{
    ChassisMode mode;                                /**< 当前底盘模式。 */
    ChassisOmniPid drive_pid[CHASSIS_OMNI_WHEEL_NUM];/**< 驱动轮速度 PID。 */
    ChassisOmniPid yaw_pid;                         /**< yaw 跟随 PID。 */
    PowerCtrl power;                                /**< 驱动电机功率预测控制器。 */
    ChassisOmniWheel wheel[CHASSIS_OMNI_WHEEL_NUM]; /**< 驱动轮状态。 */
    int16_t current[CHASSIS_OMNI_WHEEL_NUM];        /**< 输出电流。 */
    uint8_t ready;                                  /**< 非 0 表示构型已初始化。 */
} ChassisOmniCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化全向轮底盘。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisOmniCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_omni_init(Chassis *chassis, void *impl);

/**
 * @brief 执行一次全向轮底盘周期。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisOmniCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_omni_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);

/**
 * @brief 更新全向轮底盘反馈。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisOmniCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_omni_update(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 执行全向轮底盘控制计算。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisOmniCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_omni_control(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 生成全向轮底盘输出。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisOmniCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_omni_output(const Chassis *chassis, void *impl, ChassisOutput *output);

/**
 * @brief 同步全向轮底盘目标。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisOmniCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_omni_sync(Chassis *chassis, void *impl);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_CHASSIS_OMNI_H
