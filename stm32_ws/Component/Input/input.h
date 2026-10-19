/**
 ******************************************************************************
 * @file    input.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   统一输入组件公共接口。
 *
 * @note    本文件定义输入源聚合、优先级仲裁和机构命令输出接口。
 * @note    遥控器协议解析位于 Devices/Rc；本层负责把物理输入解释为机构参考。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_INPUT_H
#define PRINTK_COMPONENT_INPUT_H

#include "Src/input_chassis_planner.h"
#include "Src/input_chassis_rc.h"
#include "Src/input_chassis_slam.h"
#include "Src/input_cali_rc.h"
#include "Src/input_gimbal_planner.h"
#include "Src/input_gimbal_rc.h"
#include "Src/input_gimbal_vision.h"
#include "Src/input_shoot_planner.h"
#include "Src/input_shoot_rc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */

/**
 * @brief Input 组件配置。
 */
typedef struct
{
    InputGimbalRcMap gimbal_rc;   /**< RC 到云台参考的映射配置。 */
    InputChassisRcMap chassis_rc; /**< RC 到底盘参考的映射配置。 */
    InputShootRcMap shoot_rc;     /**< RC 到发射参考的映射配置。 */
    InputCaliRcMap cali_rc;       /**< RC 到校准参考的映射配置。 */
} InputCfg;

/**
 * @brief 单周期输入源集合。
 */
typedef struct
{
    const RcDbusState *rc;                       /**< DBUS 风格 RC 物理输入，可为空。 */
    const InputVisionState *vision;              /**< 视觉输入，可为空。 */
    const InputSlamState *slam;                  /**< SLAM 输入，可为空。 */
    const InputGimbalPlannerState *gimbal_plan;  /**< 云台 planner 输入，可为空。 */
    const InputChassisPlannerState *chassis_plan;/**< 底盘 planner 输入，可为空。 */
    const InputShootPlannerState *shoot_plan;    /**< 发射 planner 输入，可为空。 */
    float32_t gimbal_yaw;                        /**< 当前云台 yaw 角，供人工参考生成使用，单位 rad。 */
    uint32_t tick_ms;                            /**< 当前聚合时间戳，单位 ms。 */
    uint8_t gimbal_yaw_valid;                    /**< 非 0 表示 gimbal_yaw 可用。 */
} InputFrame;

/**
 * @brief Input 组件输出。
 */
typedef struct
{
    InputRef ref;       /**< 仲裁后的统一参考。 */
    GimbalCmd gimbal;   /**< 云台机构命令。 */
    ChassisCmd chassis; /**< 底盘机构命令。 */
    ShootCmd shoot;     /**< 发射机构命令。 */
    InputMagCmd mag;    /**< 弹舱命令。 */
    InputCaliCmd cali;  /**< 校准命令。 */
} InputOutput;

/**
 * @brief Input 组件上下文。
 */
typedef struct
{
    InputCfg cfg;              /**< 配置副本。 */
    InputOutput output;        /**< 最近一次输出。 */
    InputGimbalRcCtx gimbal_rc;/**< RC 云台映射上下文。 */
    InputChassisRcCtx chassis_rc; /**< RC 底盘映射上下文。 */
    InputShootRcCtx shoot_rc;  /**< RC 发射映射上下文。 */
    InputCaliRcCtx cali_rc;    /**< RC 校准映射上下文。 */
    uint8_t initialized;       /**< 非 0 表示已初始化。 */
} Input;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 Input 组件。
 *
 * @param input Input 组件上下文。
 * @param cfg Input 配置。
 * @return 执行状态。
 */
InputStatus input_init(Input *input, const InputCfg *cfg);

/**
 * @brief 更新输入聚合和命令输出。
 *
 * @param input Input 组件上下文。
 * @param frame 单周期输入源集合。
 * @return 执行状态。
 */
InputStatus input_update(Input *input, const InputFrame *frame);

/**
 * @brief 获取最近一次 Input 输出。
 *
 * @param input Input 组件上下文。
 * @param out 输出快照。
 * @return 执行状态。
 */
InputStatus input_get(const Input *input, InputOutput *out);

/**
 * @brief 执行一次完整输入周期。
 *
 * @param input Input 组件上下文。
 * @param frame 单周期输入源集合。
 * @param out 输出快照。
 * @return 执行状态。
 */
InputStatus input_step(Input *input, const InputFrame *frame, InputOutput *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_INPUT_H
