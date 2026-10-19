/**
 ******************************************************************************
 * @file    thermal.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   温控控制器公共接口。
 *
 * @note    本组件只负责 temperature + target + PID 到 PWM compare 的闭环控制。
 * @note    温度采样来自设备层，compare 写入也由设备层完成，本组件不访问硬件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_CONTROLLER_THERMAL_H
#define PRINTK_COMPONENT_CONTROLLER_THERMAL_H

#include <stdint.h>

#include "arm_math.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 默认温控目标温度，单位摄氏度。 */

#ifndef THERMAL_TARGET_DEFAULT
#define THERMAL_TARGET_DEFAULT 40.0F
#endif

/** @brief 默认温控关断死区，单位摄氏度。 */
#ifndef THERMAL_DEADBAND_DEFAULT
#define THERMAL_DEADBAND_DEFAULT 0.25F
#endif

/** @brief 默认温控计算周期，单位 s。 */
#ifndef THERMAL_PERIOD_DEFAULT
#define THERMAL_PERIOD_DEFAULT 1.0F
#endif

/** @brief 默认 PWM compare 下限。 */
#ifndef THERMAL_COMPARE_MIN_DEFAULT
#define THERMAL_COMPARE_MIN_DEFAULT 0.0F
#endif

/** @brief 默认 PWM compare 上限。 */
#ifndef THERMAL_COMPARE_MAX_DEFAULT
#define THERMAL_COMPARE_MAX_DEFAULT 9999.0F
#endif

/** @brief 默认 PWM compare 单秒最大变化量，0 表示不限制。 */
#ifndef THERMAL_SLEW_DEFAULT
#define THERMAL_SLEW_DEFAULT 0.0F
#endif

/** @brief 默认温控 PID KP。 */
#ifndef THERMAL_PID_KP_DEFAULT
#define THERMAL_PID_KP_DEFAULT 1800.0F
#endif

/** @brief 默认温控 PID KI。 */
#ifndef THERMAL_PID_KI_DEFAULT
#define THERMAL_PID_KI_DEFAULT 80.0F
#endif

/** @brief 默认温控 PID KD。 */
#ifndef THERMAL_PID_KD_DEFAULT
#define THERMAL_PID_KD_DEFAULT 0.0F
#endif

/**
 * @brief 温控控制器执行状态。
 */
typedef enum
{
    ThermalOk = 0,      /**< 执行成功。 */
    ThermalNull = 1,    /**< 输入指针为空。 */
    ThermalArg = 2,     /**< 配置参数非法。 */
    ThermalValue = 3,   /**< 输入数值非法。 */
    ThermalUninit = 4,  /**< 上下文未初始化。 */
    ThermalPid = 5      /**< PID 初始化或计算失败。 */
} ThermalStatus;

/**
 * @brief 温控 PID 参数。
 */
typedef struct
{
    float32_t kp;       /**< 比例增益。 */
    float32_t ki;       /**< 积分增益。 */
    float32_t kd;       /**< 微分增益。 */
    float32_t max_out;  /**< PID 输出绝对值限幅。 */
    float32_t max_iout; /**< PID 积分输出绝对值限幅。 */
} ThermalPidCfg;

/**
 * @brief 温控控制器配置。
 */
typedef struct
{
    ThermalPidCfg pid;       /**< 温控 PID 参数。 */
    float32_t target;        /**< 目标温度，单位摄氏度。 */
    float32_t deadband;      /**< 关断死区，温度达到 target-deadband 后输出 0。 */
    float32_t period;        /**< PID 计算周期，单位 s；0 表示每次更新都计算。 */
    float32_t compare_min;   /**< PWM compare 下限。 */
    float32_t compare_max;   /**< PWM compare 上限。 */
    float32_t compare_slew;  /**< PWM compare 单秒最大变化量；0 表示不限制。 */
} ThermalCfg;

/**
 * @brief 温控控制器周期输入。
 */
typedef struct
{
    float32_t temperature; /**< 当前温度，单位摄氏度。 */
    float32_t dt;          /**< 本周期时间间隔，单位 s。 */
    uint8_t online;        /**< 非 0 表示温度反馈有效。 */
} ThermalInput;

/**
 * @brief 温控控制器周期输出。
 */
typedef struct
{
    uint32_t compare;      /**< PWM compare 输出。 */
    float32_t compare_f;   /**< 浮点 compare 输出，便于调试。 */
    float32_t target;      /**< 当前目标温度，单位摄氏度。 */
    float32_t temperature; /**< 当前反馈温度，单位摄氏度。 */
    float32_t pid_raw;     /**< PID 原始限幅后输出。 */
    uint8_t enable;        /**< 非 0 表示本周期需要加热。 */
} ThermalOutput;

/**
 * @brief 温控控制器上下文。
 */
typedef struct
{
    ThermalCfg cfg;          /**< 温控配置副本。 */
    pid_ctrl_t pid;          /**< PID 控制器上下文。 */
    ThermalOutput output;    /**< 最近一次输出。 */
    float32_t period_time;   /**< PID 计算周期累计时间，单位 s。 */
    float32_t compare_state; /**< 斜率限制后的浮点 compare 状态。 */
    uint8_t initialized;     /**< 非 0 表示已经初始化。 */
} Thermal;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化温控控制器。
 *
 * @param thermal 温控控制器上下文。
 * @param cfg 温控控制器配置。
 * @return 执行状态。
 */
ThermalStatus thermal_init(Thermal *thermal, const ThermalCfg *cfg);

/**
 * @brief 复位温控控制器动态状态。
 *
 * @param thermal 温控控制器上下文。
 * @return 执行状态。
 */
ThermalStatus thermal_reset(Thermal *thermal);

/**
 * @brief 执行一次完整温控周期。
 *
 * @param thermal 温控控制器上下文。
 * @param input 本周期输入。
 * @param output 本周期输出。
 * @return 执行状态。
 */
ThermalStatus thermal_step(Thermal *thermal, const ThermalInput *input, ThermalOutput *output);

/**
 * @brief 更新温控控制器。
 *
 * @param thermal 温控控制器上下文。
 * @param input 本周期输入。
 * @return 执行状态。
 */
ThermalStatus thermal_update(Thermal *thermal, const ThermalInput *input);

/**
 * @brief 获取最近一次温控输出。
 *
 * @param thermal 温控控制器上下文。
 * @param output 输出对象。
 * @return 执行状态。
 */
ThermalStatus thermal_output(const Thermal *thermal, ThermalOutput *output);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_CONTROLLER_THERMAL_H
