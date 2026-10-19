/**
 ******************************************************************************
 * @file    chassis_balance.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   平衡底盘构型接口。
 *
 * @note    本构型只保留平衡底盘机构控制，不读取任务、INS、CAN 或裁判系统全局状态。
 * @note    IMU、关节、轮毂、腿长、功率预算和命令均通过 ChassisInput 注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_CHASSIS_BALANCE_H
#define PRINTK_MECHANISM_CHASSIS_BALANCE_H

#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 平衡底盘腿数量。 */
#define CHASSIS_BALANCE_LEG_NUM 2U

/**
 * @brief 平衡底盘 PID 运行状态。
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
} ChassisBalancePid;

/**
 * @brief 平衡底盘单腿状态。
 */
typedef struct
{
    float32_t length;      /**< 腿长反馈。 */
    float32_t length_speed;/**< 腿长速度反馈。 */
    float32_t theta;       /**< 腿摆角反馈。 */
    float32_t theta_speed; /**< 腿摆角速度反馈。 */
    float32_t force_set;   /**< 目标支撑力。 */
    uint8_t online;        /**< 非 0 表示该侧反馈在线。 */
} ChassisBalanceLeg;

/**
 * @brief 平衡底盘私有上下文。
 */
typedef struct
{
    ChassisMode mode;                                /**< 当前底盘模式。 */
    ChassisBalancePid leg_pid[CHASSIS_BALANCE_LEG_NUM]; /**< 左右腿长 PID。 */
    ChassisBalancePid roll_pid;                      /**< roll 支撑 PID。 */
    ChassisBalancePid turn_pid;                      /**< 转向 PID。 */
    ChassisBalanceLeg leg[CHASSIS_BALANCE_LEG_NUM];  /**< 左右腿状态。 */
    ChassisMitFb joint[CHASSIS_JOINT_MAX];           /**< 关节反馈副本。 */
    ChassisMitFb wheel[CHASSIS_WHEEL_MAX];           /**< 轮毂反馈副本。 */
    float32_t joint_torque[CHASSIS_JOINT_MAX];       /**< 关节输出力矩。 */
    float32_t wheel_torque[CHASSIS_WHEEL_MAX];       /**< 轮毂输出力矩。 */
    float32_t joint_pos_set[CHASSIS_JOINT_MAX];      /**< 关节位置目标。 */
    uint8_t ready;                                   /**< 非 0 表示构型已初始化。 */
} ChassisBalanceCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化平衡底盘。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisBalanceCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_balance_init(Chassis *chassis, void *impl);

/**
 * @brief 执行一次平衡底盘周期。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisBalanceCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_balance_step(Chassis *chassis, void *impl, const ChassisInput *input, ChassisOutput *output);

/**
 * @brief 更新平衡底盘反馈。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisBalanceCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_balance_update(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 执行平衡底盘控制计算。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisBalanceCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ChassisStatus chassis_balance_control(Chassis *chassis, void *impl, const ChassisInput *input);

/**
 * @brief 生成平衡底盘输出。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisBalanceCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ChassisStatus chassis_balance_output(const Chassis *chassis, void *impl, ChassisOutput *output);

/**
 * @brief 同步平衡底盘目标。
 *
 * @param chassis 底盘公共上下文。
 * @param impl ChassisBalanceCtx 私有上下文。
 * @return 执行状态。
 */
ChassisStatus chassis_balance_sync(Chassis *chassis, void *impl);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_CHASSIS_BALANCE_H
