/**
 ******************************************************************************
 * @file    observer_ins.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   INS 姿态观测内部接口。
 *
 * @note    本文件只声明 Observer 内部 INS 子模块入口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_OBSERVER_INS_H
#define PRINTK_COMPONENT_OBSERVER_INS_H

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
 * @brief 初始化 INS 姿态观测子模块。
 *
 * @param observer 观测器上下文。
 * @return 执行状态。
 */
ObserverStatus observer_ins_init(Observer *observer);

/**
 * @brief 复位 INS 姿态观测动态状态。
 *
 * @param observer 观测器上下文。
 * @return 执行状态。
 */
ObserverStatus observer_ins_reset(Observer *observer);

/**
 * @brief 更新 INS 姿态观测状态。
 *
 * @param observer 观测器上下文。
 * @param input 本周期观测器输入。
 * @return 执行状态。
 */
ObserverStatus observer_ins_update(Observer *observer, const ObserverInput *input);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_OBSERVER_INS_H
