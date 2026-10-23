/**
 ******************************************************************************
 * @file    angle_pid.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-10-20
 * @brief   角度环 PID 组合接口。
 *
 * @note    复用 pid_core/pid_angle 的角度误差 wrap、积分和外部微分策略。
 * @note    target 是持久目标，不会被短路径临时 ±2π 改写。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ALGORITHM_ANGLE_PID_H
#define PRINTK_ALGORITHM_ANGLE_PID_H

#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 角度环 PID 配置。
 */
typedef struct
{
    float32_t kp;         /**< 比例增益。 */
    float32_t ki;         /**< 积分增益。 */
    float32_t kd;         /**< 微分增益。 */
    float32_t max_out;    /**< 输出绝对值限幅。 */
    float32_t max_iout;   /**< 积分绝对值限幅。 */
    float32_t deadband;   /**< 误差死区，rad；0 表示关闭。 */
    float32_t freeze_err; /**< 积分泄漏误差阈值，rad。 */
    float32_t freeze_d;   /**< 积分泄漏外部微分阈值。 */
    float32_t active_err; /**< 积分活动误差上限，rad。 */
    float32_t leak;       /**< 积分泄漏系数，1.0 表示不泄漏。 */
} angle_pid_cfg_t;

/**
 * @brief 角度环 PID 上下文。
 */
typedef struct
{
    pid_ctrl_t pid;         /**< 底层 PID 控制器。 */
    angle_pid_cfg_t cfg;    /**< 配置副本。 */
    float32_t target;       /**< 持久目标角，rad。 */
    uint8_t in_deadband;    /**< 非 0 表示当前处于死区。 */
} angle_pid_t;

/**
 * @brief 初始化角度环 PID。
 *
 * @param pid 角度 PID 上下文。
 * @param cfg 配置。
 * @return 初始化状态。
 */
pid_core_status_t angle_pid_init(angle_pid_t *pid, const angle_pid_cfg_t *cfg);

/**
 * @brief 清除角度 PID 动态状态（保留配置）。
 *
 * @param pid 角度 PID 上下文。
 * @return 执行状态。
 */
pid_core_status_t angle_pid_reset(angle_pid_t *pid);

/**
 * @brief 设置持久目标角。
 *
 * @param pid 角度 PID 上下文。
 * @param target 目标角，rad；不会被内部改写。
 */
void angle_pid_set_target(angle_pid_t *pid, float32_t target);

/**
 * @brief 读取持久目标角。
 *
 * @param pid 角度 PID 上下文。
 * @return 目标角，rad。
 */
float32_t angle_pid_get_target(const angle_pid_t *pid);

/**
 * @brief 将角度归一化到 [-pi, pi)。
 *
 * @param angle 输入角度，rad。
 * @return 归一化角度，rad。
 */
float32_t angle_pid_wrap(float32_t angle);

/**
 * @brief 执行一次角度 PID 更新。
 *
 * @param pid 角度 PID 上下文。
 * @param actual 当前角度反馈，rad。
 * @return 限幅后的 PID 输出。
 */
float32_t angle_pid_calc(angle_pid_t *pid, float32_t actual);

/**
 * @brief 执行一次带外部微分的角度 PID 更新。
 *
 * @param pid 角度 PID 上下文。
 * @param actual 当前角度反馈，rad。
 * @param d_in 外部微分输入，通常为 -gyro。
 * @return 限幅后的 PID 输出。
 */
float32_t angle_pid_calc_d(angle_pid_t *pid, float32_t actual, float32_t d_in);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ALGORITHM_ANGLE_PID_H */
