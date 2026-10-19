/**
 ******************************************************************************
 * @file    pid.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   PID 控制器 C 语言组合接口。
 *
 * @note    pid_core 提供标准离散 PID 主干，本文件提供常用策略组合入口。
 *
 * @note    默认线性策略覆盖底盘当前使用的位置式 PID；角度策略用于 wrap 误差和外部微分。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_PID_H
#define PRINTK_ALGORITHM_PID_H

#include "pid_angle.h"
#include "pid_linear.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 类型与策略配置
 * --------------------------------------------------------------------------
 * 本组类型只描述 PID core 与策略需要的输入输出，不携带具体机构业务含义。
 * ========================================================================== */
/**
 * @brief PID 策略类型。
 */
typedef enum
{
    PID_LINEAR = 0, /**< 线性位置式 PID。 */
    PID_ANGLE = 1   /**< 角度位置式 PID，误差按 [-pi, pi) 归一化。 */
} pid_mode_t;

/**
 * @brief PID 控制器配置。
 */
typedef struct
{
    float32_t kp;       /**< 比例增益。 */
    float32_t ki;       /**< 积分增益。 */
    float32_t kd;       /**< 微分增益。 */
    float32_t max_out;  /**< 输出绝对值限幅。 */
    float32_t max_iout; /**< 积分项绝对值限幅。 */
    pid_mode_t mode;    /**< PID 策略类型。 */
    pid_angle_t angle;  /**< 角度策略参数，仅 PID_ANGLE 使用。 */
} pid_cfg_t;

/**
 * @brief PID 单步输入。
 */
typedef struct
{
    float32_t set;  /**< 设定值。 */
    float32_t get;  /**< 反馈值。 */
    float32_t d_in; /**< 外部微分输入，仅部分策略使用。 */
    float32_t ff;   /**< 前馈输入。 */
} pid_in_t;

/**
 * @brief PID 单步输出项。
 */
typedef pid_core_out_t pid_out_t;

/**
 * @brief PID 控制器上下文。
 */
typedef struct
{
    pid_core_t core;   /**< PID 核心上下文。 */
    pid_mode_t mode;   /**< 当前策略类型。 */
    pid_angle_t angle; /**< 角度策略参数。 */
} pid_ctrl_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 对外接口保持短小，具体控制语义由调用方选择线性或角度策略后注入。
 * ========================================================================== */
/**
 * @brief 初始化 PID 控制器。
 *
 * @param pid PID 控制器上下文。
 * @param cfg PID 控制器配置。
 * @return 初始化状态。
 */
pid_core_status_t pid_init(pid_ctrl_t *pid, const pid_cfg_t *cfg);

/**
 * @brief 清除 PID 动态状态。
 *
 * @param pid PID 控制器上下文。
 * @return 执行状态。
 */
pid_core_status_t pid_reset(pid_ctrl_t *pid);

/**
 * @brief 执行一次 PID 更新。
 *
 * @param pid PID 控制器上下文。
 * @param in 本次输入。
 * @param out 本次输出项，可为空。
 * @return 执行状态。
 */
pid_core_status_t pid_step(pid_ctrl_t *pid,
                           const pid_in_t *in,
                           pid_out_t *out);

/**
 * @brief 以 set/get 形式执行一次 PID 更新。
 *
 * @param pid PID 控制器上下文。
 * @param get 反馈值。
 * @param set 设定值。
 * @return PID 限幅后输出，参数错误时返回 0。
 */
float32_t pid_calc(pid_ctrl_t *pid, float32_t get, float32_t set);

/**
 * @brief 以 set/get/外部微分形式执行一次 PID 更新。
 *
 * @param pid PID 控制器上下文。
 * @param get 反馈值。
 * @param set 设定值。
 * @param d_in 外部微分输入。
 * @return PID 限幅后输出，参数错误时返回 0。
 */
float32_t pid_calc_d(pid_ctrl_t *pid,
                     float32_t get,
                     float32_t set,
                     float32_t d_in);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_ALGORITHM_PID_H
