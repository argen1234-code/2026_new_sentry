/**
 ******************************************************************************
 * @file    power_ctrl.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   底盘功率预测控制器公共接口。
 *
 * @note    本控制器只根据注入的电机转速、电流目标和功率预算计算统一缩放系数。
 * @note    本文件不读取裁判系统、任务全局、电机设备或 CAN，只做纯控制计算。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_POWER_CTRL_H
#define PRINTK_COMPONENT_POWER_CTRL_H

#include "arm_math.h"
#include "robot.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 功率控制器默认参与预测的最大电机数量。 */
#ifndef POWER_CTRL_MOTOR_MAX
/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
#define POWER_CTRL_MOTOR_MAX 4U
#endif

/** @brief 默认速度平方项系数。 */
#ifndef POWER_CTRL_K1_DEFAULT
#ifdef ROBOT_POWER_CHASSIS_K1
#define POWER_CTRL_K1_DEFAULT ROBOT_POWER_CHASSIS_K1
#else
#define POWER_CTRL_K1_DEFAULT 1.453e-07F
#endif
#endif

/** @brief 默认电流平方项系数。 */
#ifndef POWER_CTRL_K2_DEFAULT
#ifdef ROBOT_POWER_CHASSIS_K2
#define POWER_CTRL_K2_DEFAULT ROBOT_POWER_CHASSIS_K2
#else
#define POWER_CTRL_K2_DEFAULT 1.23e-07F
#endif
#endif

/** @brief 默认功率模型静态偏置，单位 W。 */
#ifndef POWER_CTRL_OFFSET_DEFAULT
#ifdef ROBOT_POWER_CHASSIS_OFFSET
#define POWER_CTRL_OFFSET_DEFAULT ROBOT_POWER_CHASSIS_OFFSET
#else
#define POWER_CTRL_OFFSET_DEFAULT 4.081F
#endif
#endif

/** @brief 默认电流到转矩的等效系数。 */
#ifndef POWER_CTRL_TORQUE_DEFAULT
#ifdef ROBOT_POWER_CHASSIS_TORQUE
#define POWER_CTRL_TORQUE_DEFAULT ROBOT_POWER_CHASSIS_TORQUE
#else
#define POWER_CTRL_TORQUE_DEFAULT (0.01562F * 0.001220703125F)
#endif
#endif

/** @brief 默认 rpm 到 rad/s 换算分母。 */
#ifndef POWER_CTRL_RPM_RAD_DEFAULT
#ifdef ROBOT_POWER_CHASSIS_RPM_RAD
#define POWER_CTRL_RPM_RAD_DEFAULT ROBOT_POWER_CHASSIS_RPM_RAD
#else
#define POWER_CTRL_RPM_RAD_DEFAULT 9.55F
#endif
#endif

/** @brief 默认给舵向或其它底盘附属电机预留的功率，单位 W。 */
#ifndef POWER_CTRL_RESERVE_DEFAULT
#ifdef ROBOT_POWER_CHASSIS_RESERVE
#define POWER_CTRL_RESERVE_DEFAULT ROBOT_POWER_CHASSIS_RESERVE
#else
#define POWER_CTRL_RESERVE_DEFAULT 16.0F
#endif
#endif

/** @brief 默认最小缩放系数。 */
#ifndef POWER_CTRL_SCALE_MIN
#ifdef ROBOT_POWER_CHASSIS_SCALE_MIN
#define POWER_CTRL_SCALE_MIN ROBOT_POWER_CHASSIS_SCALE_MIN
#else
#define POWER_CTRL_SCALE_MIN 0.01F
#endif
#endif

/** @brief 判别式异常时的默认缩放系数。 */
#ifndef POWER_CTRL_SCALE_FALLBACK
#ifdef ROBOT_POWER_CHASSIS_SCALE_FALLBACK
#define POWER_CTRL_SCALE_FALLBACK ROBOT_POWER_CHASSIS_SCALE_FALLBACK
#else
#define POWER_CTRL_SCALE_FALLBACK 0.5F
#endif
#endif

/**
 * @brief 功率控制器执行状态。
 */
typedef enum
{
    PowerCtrlOk = 0,     /**< 执行成功。 */
    PowerCtrlNull = 1,   /**< 输入指针为空。 */
    PowerCtrlArg = 2,    /**< 输入参数非法。 */
    PowerCtrlValue = 3,  /**< 输入数值非法。 */
    PowerCtrlUninit = 4  /**< 控制器尚未初始化。 */
} PowerCtrlStatus;

/**
 * @brief 单个参与功率预测的电机输入。
 */
typedef struct
{
    float32_t speed_rpm; /**< 电机反馈转速，单位 rpm。 */
    float32_t current;   /**< 本周期准备输出的电流命令。 */
    uint8_t online;      /**< 非 0 表示该电机反馈有效。 */
} PowerCtrlMotor;

/**
 * @brief 功率预测模型配置。
 */
typedef struct
{
    float32_t k1;             /**< 速度平方项系数。 */
    float32_t k2;             /**< 电流平方项系数。 */
    float32_t offset;         /**< 静态功率偏置，单位 W。 */
    float32_t torque_coeff;   /**< 电流到转矩的等效系数。 */
    float32_t rpm_to_rad;     /**< rpm 到 rad/s 换算分母。 */
    float32_t reserve_power;  /**< 预留功率，单位 W。 */
    float32_t min_scale;      /**< 最小缩放系数。 */
    float32_t fallback_scale; /**< 求解异常时的缩放系数。 */
} PowerCtrlCfg;

/**
 * @brief 功率控制器周期输入。
 */
typedef struct
{
    PowerCtrlMotor motor[POWER_CTRL_MOTOR_MAX]; /**< 参与预测的电机数组。 */
    uint8_t motor_count;                        /**< 有效电机槽数量。 */
    float32_t power_limit;                      /**< 当前功率上限，单位 W。 */
    float32_t measured_power;                   /**< 当前实测功率，单位 W，仅用于快照记录。 */
    float32_t buffer_energy;                    /**< 当前缓冲能量，单位 J，仅用于快照记录。 */
} PowerCtrlInput;

/**
 * @brief 功率控制器周期输出。
 */
typedef struct
{
    float32_t scale;           /**< 本周期电流缩放系数。 */
    float32_t predicted_power; /**< 未缩放前预测功率，单位 W。 */
    float32_t available_power; /**< 扣除预留后的可用功率，单位 W。 */
} PowerCtrlOutput;

/**
 * @brief 功率控制器上下文。
 */
typedef struct
{
    PowerCtrlCfg cfg;       /**< 功率模型配置副本。 */
    PowerCtrlOutput last;   /**< 最近一次输出快照。 */
    float32_t measured_power; /**< 最近一次注入的实测功率。 */
    float32_t buffer_energy;  /**< 最近一次注入的缓冲能量。 */
    uint8_t initialized;    /**< 非 0 表示已初始化。 */
} PowerCtrl;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化功率控制器。
 *
 * @param ctrl 功率控制器上下文。
 * @param cfg 功率模型配置，允许为空，空指针时使用默认值。
 * @return 执行状态。
 */
PowerCtrlStatus power_ctrl_init(PowerCtrl *ctrl, const PowerCtrlCfg *cfg);

/**
 * @brief 执行一次功率预测和电流缩放计算。
 *
 * @param ctrl 功率控制器上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
PowerCtrlStatus power_ctrl_step(PowerCtrl *ctrl, const PowerCtrlInput *input, PowerCtrlOutput *output);

/**
 * @brief 计算并返回本周期缩放系数。
 *
 * @param ctrl 功率控制器上下文。
 * @param input 周期输入。
 * @return 缩放系数，异常时返回 1.0。
 */
float32_t power_ctrl_scale(PowerCtrl *ctrl, const PowerCtrlInput *input);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_COMPONENT_POWER_CTRL_H */
