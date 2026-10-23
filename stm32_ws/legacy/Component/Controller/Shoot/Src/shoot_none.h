/**
 ******************************************************************************
 * @file    shoot_none.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   无发射机构构型接口。
 *
 * @note    本构型用于明确禁用发射机构，所有电机输出均为 0。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_SHOOT_NONE_H
#define PRINTK_MECHANISM_SHOOT_NONE_H

#include "shoot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief 无发射机构私有上下文。
 */
typedef struct
{
    uint8_t ready; /**< 非 0 表示构型已初始化。 */
} ShootNoneCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化无发射机构。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootNoneCtx 私有上下文。
 * @return 执行状态。
 */
ShootStatus shoot_none_init(Shoot *shoot, void *impl);

/**
 * @brief 执行一次无发射机构周期。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootNoneCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ShootStatus shoot_none_step(Shoot *shoot, void *impl, const ShootInput *input, ShootOutput *output);

/**
 * @brief 更新无发射机构反馈。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootNoneCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ShootStatus shoot_none_update(Shoot *shoot, void *impl, const ShootInput *input);

/**
 * @brief 执行无发射机构控制计算。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootNoneCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ShootStatus shoot_none_control(Shoot *shoot, void *impl, const ShootInput *input);

/**
 * @brief 生成无发射机构输出。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootNoneCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ShootStatus shoot_none_output(const Shoot *shoot, void *impl, ShootOutput *output);

/**
 * @brief 同步无发射机构目标。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootNoneCtx 私有上下文。
 * @return 执行状态。
 */
ShootStatus shoot_none_sync(Shoot *shoot, void *impl);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_SHOOT_NONE_H
