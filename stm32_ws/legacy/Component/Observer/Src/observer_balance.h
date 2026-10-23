/**
 ******************************************************************************
 * @file    observer_balance.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   平衡腿观测内部接口。
 *
 * @note    本文件只声明 Observer 内部平衡腿几何和离地判断入口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_OBSERVER_BALANCE_H
#define PRINTK_COMPONENT_OBSERVER_BALANCE_H

#include "observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化平衡腿观测子模块。
 *
 * @param observer 观测器上下文。
 * @return 执行状态。
 */
ObserverStatus observer_balance_init(Observer *observer);

/**
 * @brief 复位平衡腿观测动态状态。
 *
 * @param observer 观测器上下文。
 * @return 执行状态。
 */
ObserverStatus observer_balance_reset(Observer *observer);

/**
 * @brief 更新平衡腿观测状态。
 *
 * @param observer 观测器上下文。
 * @param input 本周期观测器输入。
 * @return 执行状态。
 */
ObserverStatus observer_balance_update(Observer *observer, const ObserverInput *input);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_OBSERVER_BALANCE_H
