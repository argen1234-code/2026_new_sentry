/**
 ******************************************************************************
 * @file    pid_core.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   标准离散 PID 核心接口。
 *
 * @note    本文件只定义 PID 主干差分方程和 hook 插点，不包含具体业务策略。
 *
 * @note    默认行为为离散位置式 PID：err=set-get，I+=ki*err，D=kd*(err-last_err)。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_PID_CORE_H
#define PRINTK_ALGORITHM_PID_CORE_H

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 类型与策略配置
 * --------------------------------------------------------------------------
 * 本组类型只描述 PID core 与策略需要的输入输出，不携带具体机构业务含义。
 * ========================================================================== */
/**
 * @brief PID 核心执行状态。
 */
typedef enum
{
    PID_CORE_OK = 0,       /**< 执行成功。 */
    PID_CORE_ERR_ARG = 1,  /**< 输入指针无效。 */
    PID_CORE_ERR_VALUE = 2 /**< 输入数值非法。 */
} pid_core_status_t;

/**
 * @brief PID 核心单步输入。
 */
typedef struct
{
    float32_t set;  /**< 设定值。 */
    float32_t get;  /**< 反馈值。 */
    float32_t d_in; /**< 外部微分输入，默认策略不使用。 */
    float32_t ff;   /**< 前馈输入，直接加到输出项。 */
} pid_core_in_t;

/**
 * @brief PID 核心单步输出项。
 */
typedef struct
{
    float32_t err;  /**< 本次误差。 */
    float32_t p;    /**< 比例项。 */
    float32_t i;    /**< 积分项。 */
    float32_t d;    /**< 微分项。 */
    float32_t ff;   /**< 前馈项。 */
    float32_t raw;  /**< 限幅前输出。 */
    float32_t out;  /**< 限幅后输出。 */
} pid_core_out_t;

/**
 * @brief PID 核心上下文前置声明。
 */
typedef struct pid_core pid_core_t;

/**
 * @brief PID 误差 hook。
 *
 * @param set 设定值。
 * @param get 反馈值。
 * @param arg 用户上下文。
 * @return 参与 PID 主干计算的误差。
 */
typedef float32_t (*pid_core_err_fn)(float32_t set, float32_t get, void *arg);

/**
 * @brief PID 积分 hook。
 *
 * hook 可直接修改 pid->iout。为空时使用 I += ki * err。
 *
 * @param pid PID 核心上下文。
 * @param err 本次误差。
 * @param in 本次输入。
 * @param arg 用户上下文。
 */
typedef void (*pid_core_i_fn)(pid_core_t *pid,
                              float32_t err,
                              const pid_core_in_t *in,
                              void *arg);

/**
 * @brief PID 微分 hook。
 *
 * @param pid PID 核心上下文。
 * @param err 本次误差。
 * @param in 本次输入。
 * @param arg 用户上下文。
 * @return 微分项输出，不再额外乘 kd。
 */
typedef float32_t (*pid_core_d_fn)(const pid_core_t *pid,
                                   float32_t err,
                                   const pid_core_in_t *in,
                                   void *arg);

/**
 * @brief PID hook 组合。
 */
typedef struct
{
    pid_core_err_fn err; /**< 误差计算 hook，为空时使用 set-get。 */
    pid_core_i_fn i;     /**< 积分更新 hook，为空时使用 I+=ki*err。 */
    pid_core_d_fn d;     /**< 微分计算 hook，为空时使用 kd*(err-last_err)。 */
} pid_core_ops_t;

/**
 * @brief PID 核心配置。
 */
typedef struct
{
    float32_t kp;              /**< 比例增益。 */
    float32_t ki;              /**< 积分增益。 */
    float32_t kd;              /**< 微分增益。 */
    float32_t max_out;         /**< 输出绝对值限幅。 */
    float32_t max_iout;        /**< 积分项绝对值限幅。 */
    const pid_core_ops_t *ops; /**< hook 组合，为空时使用标准位置式 PID。 */
    void *arg;                 /**< hook 用户上下文。 */
} pid_core_cfg_t;

/**
 * @brief PID 核心上下文。
 */
struct pid_core
{
    float32_t kp;              /**< 比例增益。 */
    float32_t ki;              /**< 积分增益。 */
    float32_t kd;              /**< 微分增益。 */
    float32_t max_out;         /**< 输出绝对值限幅。 */
    float32_t max_iout;        /**< 积分项绝对值限幅。 */
    float32_t set;             /**< 最近一次设定值。 */
    float32_t get;             /**< 最近一次反馈值。 */
    float32_t err[3];          /**< 最近三次误差，err[0] 为本次误差。 */
    float32_t pout;            /**< 比例项输出。 */
    float32_t iout;            /**< 积分项输出。 */
    float32_t dout;            /**< 微分项输出。 */
    float32_t ff;              /**< 前馈项输出。 */
    float32_t out;             /**< 总输出。 */
    const pid_core_ops_t *ops; /**< hook 组合。 */
    void *arg;                 /**< hook 用户上下文。 */
};

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 对外接口保持短小，具体控制语义由调用方选择线性或角度策略后注入。
 * ========================================================================== */
/**
 * @brief 对称限幅。
 *
 * @param value 输入值。
 * @param limit 绝对值上限，必须非负。
 * @return 限幅后的值。
 */
float32_t pid_core_limit(float32_t value, float32_t limit);

/**
 * @brief 初始化 PID 核心。
 *
 * @param pid PID 核心上下文。
 * @param cfg PID 核心配置。
 * @return 初始化状态。
 */
pid_core_status_t pid_core_init(pid_core_t *pid, const pid_core_cfg_t *cfg);

/**
 * @brief 清除 PID 动态状态。
 *
 * @param pid PID 核心上下文。
 * @return 执行状态。
 */
pid_core_status_t pid_core_reset(pid_core_t *pid);

/**
 * @brief 执行一次 PID 更新。
 *
 * @param pid PID 核心上下文。
 * @param in 本次输入。
 * @param out 本次输出项，可为空。
 * @return 执行状态。
 */
pid_core_status_t pid_core_step(pid_core_t *pid,
                                const pid_core_in_t *in,
                                pid_core_out_t *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_ALGORITHM_PID_CORE_H
